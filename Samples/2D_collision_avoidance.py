from doctest import debug
from locale import currency
from airsim.types import DrivetrainType, VelocityControllerGains, YawMode
import setup_path
import airsim
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp


class DecisionMaker:
    
    def __init__(self, airsim_client, swarm_data, drone_names_list):
        self.airsim_client = airsim_client     
        self.swarm_data = swarm_data
        self.drone_names = drone_names_list
        self.swarm_options = {
            "square": 1,
            "line": 2,
            "vee": 3,
            "diaganol": 4,
            "echelon": 5
            }

    # ################# #
    # Utility Functions #
    # ################# #
    def get_drone_position(self, drone_name):
        # Retrieve the drone's position from AirSim
        state = self.airsim_client.simGetObjectPose(drone_name)
        return np.array([state.position.x_val, state.position.y_val, state.position.z_val])
    
    
    # Get Drone Desired Positions
    def get_desired_position(self):
        desired_final_positions_K = np.array([  # NED Coordinate system
                                    [10, 0, -5], # Drone 1
                                    [0, 0, -5],
                                    [0, 10, -5],
                                    [10, 10, -5] # DroneN ...
                                ])
        return desired_final_positions_K

    
    def reorder_list(self, original_list, new_order):
         return [original_list[i] for i in new_order]
    

    # ######################### #
    # Formation Selection Logic #
    # ######################### #
    def select_formation(self):
        # Select A Formation 
        #  1). apply core logic for formation selection
        return "square"
    

    def calculate_distance_matrix(self, drone_positions, target_positions):
        num_drones = len(drone_positions)
        num_targets = len(target_positions)
        distance_matrix = np.zeros((num_drones, num_targets))
        # Creates a N by N matrix where rows are specific drones and columns represent the target location the value inside is distance
        for i in range(num_drones):
            for j in range(num_targets):
                distance_matrix[i, j] = np.linalg.norm(np.array(drone_positions[i]) - np.array(target_positions[j]))
        return distance_matrix


    def assign_drones_to_targets(self, distance_matrix):
        row_ind, col_ind = linear_sum_assignment(distance_matrix)
        return row_ind, col_ind

    
    def generate_laplacian(self, drone_indices, target_indices):
        pass


    def swarm_position_selection(self):
        # Generate Laplacian Matrix For a Formation
        selected_formation = self.select_formation()
        # Parameters coordinates of current drones and coordinates of desired position
        # Get Drone Current Positions
        current_drone_positions = []
        for drone_index in range(len(self.drone_names)):
            position = self.get_drone_position(self.drone_names[drone_index])
            current_drone_positions.append(position) 
        print("Current Drone Positions:", current_drone_positions)
        # Get Drone Desired Positions
        desired_swarm_positions = self.get_desired_position()
        print("Desired Drone Positions:", desired_swarm_positions)
        # Calculate Distances from each point and store results for further processing
        distance_matrix = self.calculate_distance_matrix(current_drone_positions, desired_swarm_positions)
        # Apply Hungarian Algorithm to assign drones to smallest distance
        drone_indices, target_indices = self.assign_drones_to_targets(distance_matrix)
        updated_desired_swarm_positions = self.reorder_list(desired_swarm_positions.tolist(), target_indices)
        print("Distance Matrix:", distance_matrix)
        print("drone_indices:", drone_indices, "target distances:", target_indices)
        print("Updated Desired Drone Positions:", updated_desired_swarm_positions)
 
        # 2). It will need to assign drones to specific positions in the swarm that are close to their desired positions to minimize collisions
        # 3). measure distance between drones current positions andd their desired positions identify the minimum distance amongst them and positions drones based on this
        # 4). If there is a tie then we will select based on what points have the smaller next distance etc.


class CollisionAvoidance:


    def __init__(self, formation_size):
        # Drone Information
        self.formation_size = formation_size
        self.drone_names_list = []        
        self.max_velocity = 16

        # Setup Environment Parameters
        self.airsim_setup()
        self.initial_position = np.array([])
        self.flight_operation_altitude = -5 # NED Z direction altitude
    

    def airsim_setup(self):
        """
            Setup the airsim connection and initialize drones
        """
        # start drone client
        self.airsim_client = airsim.MultirotorClient()
        # Check connectivity
        self.airsim_client.confirmConnection()   
        for drone_id in range(1, self.formation_size+1):
            drone_name = "Drone" + str(drone_id)
            self.drone_names_list.append(drone_name)
            self.airsim_client.enableApiControl(True, drone_name)
            self.airsim_client.armDisarm(True, drone_name)
           

    def get_absolute_poses_NED(self):
        """
            Gets the the global position of all the drones in 
            reference to an absolute global frame of reference.
            Returns a list of all drones ordered from Drone1 - DroneN
        """
        drone_poses_NED = []
        for drone_name in self.drone_names_list:
             # simgetobjectpose returns a pose in the world frame
            drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone_name)
            drone_poses_NED.append( drone_pose_WFNED )
            
        # print("Test: Verifying printing of drones positions in global frame - ", drone_poses_NED)
        return drone_poses_NED
    

    def get_pose_position(self, pose):
        """
        Get the positional information from a pose as numpy array 
        """
        return np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val])
        
    
    def get_pose_altitude(self, pose):
        """
        Get the positional information from a pose as numpy array 
        """
        return np.array([pose.position.z_val])
        

    def get_current_drone_positions(self):
        """ Retruns the current drone positions in the NED global coord frame """
        pose_list = self.get_absolute_poses_NED()
        position_matrix = []
        for pose in pose_list:
            position = self.get_pose_position(pose)
            position_matrix.append(position)
        position_matrix = np.array(position_matrix)
        print(position_matrix)
        return position_matrix
        

    def get_laplacian(self):
        laplacian = np.array([
                [2, -1, -1, 0],
                [-1, 2, 0, -1],
                [-1, 0, 2, -1],
                [0, -1, -1, 2]
            ]) 
        return laplacian
    
    
    def get_desired_position(self):
        desired_final_positions_K = np.array([  
                                      [2, 0, -5], # Drone 1
                                      [0, 0, -5],
                                      [0, 2, -5],
                                      [2, 2, -5]
                                    ])
        return desired_final_positions_K
        

    def drone_motion_control(self, tau_dot): # altitude_control_input
        """Assigns a set velocity for the drones to reach their desired destination. """
        for index, drone_name in enumerate(self.drone_names_list):
            self.get_drone_velocity(drone_name)
            drone_new_velocity = self.check_velocity(tau_dot[index]) 
            xv = drone_new_velocity[0]
            yv = drone_new_velocity[1] 
            zv = drone_new_velocity[2] # altitude_control_input[index]
            speed = np.linalg.norm(drone_new_velocity)
            print("command velocity vector: ", xv, yv, zv, "Speed is", speed)
            self.airsim_client.moveByVelocityAsync(xv, yv, zv, 0.1, vehicle_name=drone_name) # not working


    def has_converged(self):
        """ Tells us the distance each drone is apart from one another """
        desired_positions = self.get_desired_position()
        current_position = self.get_current_drone_positions()
        total_distance = sum( [np.linalg.norm(current_position[i] - desired_positions[i]) for i in range(self.formation_size)]  )
        convergence_threshold = 0.01
        print("Distance Error", total_distance)
        if total_distance <= convergence_threshold:
            print("has converged")
            return False
        else:
            return True
        
            
    def takeoff_swarm(self):
        print(self.drone_names_list)
        for drone_name in self.drone_names_list:
            self.airsim_client.takeoffAsync(vehicle_name=drone_name).join()
    
    # ########################################### #
    # Implemetation of Collision Avoidance Begins #
    # ########################################### #

    def check_velocity(self, velocity_vector):
        speed = np.linalg.norm(velocity_vector)
        if speed > self.max_velocity:
            # Clamp
            return ( velocity_vector / np.linalg.norm(velocity_vector) ) * self.max_velocity
        else:
            # return velocity vector
            return velocity_vector


    def get_drone_position(self, drone_name):
        # Retrieve the drone's position from AirSim
        state = self.airsim_client.simGetVehiclePose(drone_name)
        return np.array([state.position.x_val, state.position.y_val, state.position.z_val])


    def get_drone_velocity(self, drone_name):
        # Retrieve the drone's velocity from AirSim
        velocity = self.airsim_client.getMultirotorState(drone_name).kinematics_estimated.linear_velocity
        velocity_vector = np.array([velocity.x_val, velocity.y_val, velocity.z_val])
        print("velocity_vector_multirotor", velocity_vector, "Speed:", np.linalg.norm(velocity_vector) )
        return velocity_vector


    def predict_future_positions(self, delta_t ):
        future_positions = {}
        for drone in self.drone_names_list:
            current_position = self.get_drone_position(drone)
            current_velocity = self.get_drone_velocity(drone)
            future_positions[drone] = current_position + current_velocity * delta_t # verify this is correct
        return future_positions
    
    
    def detect_collisions(self, future_positions, collision_threshold): # 1.5
        potential_collisions = []
        for drone1, pos1 in future_positions.items():
            for drone2, pos2 in future_positions.items():
                if drone1 != drone2:
                    distance = np.linalg.norm(pos1 - pos2)
                    if distance < collision_threshold:
                        potential_collisions.append((drone1, drone2))
        return potential_collisions
    
    
    def collision_avoidance_new_velocities(self, control_signal, current_position):
        # Get Data For Agents
        agents = []
        max_speed = 15 # m/s
        radius = 1 # meter
        print("Collision Avoidance Values")
        self.get_drone_velocity
        for index, drone_name in enumerate(self.drone_names_list):
            curr_position = (current_position[index][0], current_position[index][1])
            drone_velocity = self.get_drone_velocity(drone_name)
            curr_velocity = (drone_velocity[0], drone_velocity[1])
            pref_velocity = (control_signal[index][0], control_signal[index][1])
            agents.append( Agent(curr_position, curr_velocity, radius, max_speed, pref_velocity) )
            print("Position:", curr_position, 
                  "\nVelocity", curr_velocity,
                  "\nPrefered Velocity", pref_velocity,
                  "\nDrone Name", drone_name
                  )
        dt = 1
        tau = 1
        new_vels = [None] * len(agents)
        for i, agent in enumerate(agents):
            candidates = agents[:i] + agents[i + 1:]
            print("Candidates:", candidates)
            new_vels[i], _ = orca(agent, candidates, tau, dt)
            new_vels[i] = np.append(new_vels[i], [control_signal[index][2]])
            
            print("New Velocity", new_vels)
        return new_vels
         

    def shape_controller(self):
        
        # Create a Matplotlib figure for visualization
        plt.figure()
        plt.ion()
        time_step = 0.1
        iteration = 0
        gain_P = 1.0
        p_gain = 1
        d_gain = 1
        centroid_gain_P = 1.0
        desired_final_positions_K = self.get_desired_position()
        reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]), -5 ]) # xy plane
        # Start Simulation
        self.takeoff_swarm()

        decision_maker = DecisionMaker(self.airsim_client, self.formation_size, self.drone_names_list)
        decision_maker.swarm_position_selection()
        while True: # self.has_converged()
            # Calculate Delta between current position - desired_position
            time.sleep(time_step)

            position = self.get_current_drone_positions()
            displacement = position - desired_final_positions_K
            print("displacement: ", displacement)
            
            # Change Refrence Centroid
            #if iteration == 40:
            #    reference_centroid = np.array([10,20, -5])

            # Calculate tau_dot aka the velocity adjustment
            laplacian = self.get_laplacian()
            tau_dot = np.dot( (-1 * laplacian), displacement)
            
            print("tau_dot:", tau_dot)
            
            tau = ((tau_dot * time_step) + position)
            print("tau:", tau)
            
            # Centroid Tracking: enables drones to converge more accurately in desired location
            current_swarm_centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]),  np.mean(position[:, 2])]) #xy centroid, z height
            centroid_error = reference_centroid - current_swarm_centroid
            control_signal = (gain_P * tau_dot) + (centroid_gain_P * centroid_error) 
            print("control_signal:", control_signal)

            # Collision Avoidance
            new_control_signals = self.collision_avoidance_new_velocities(control_signal, position)
            # Motion Control
            self.drone_motion_control(new_control_signals)
            
            iteration = iteration + 1
            plt.clf()
            plt.scatter(position[:, 0], position[:, 1], label='Current Agent Positions', color='red')
            plt.scatter(desired_final_positions_K[:, 0], desired_final_positions_K[:,1], label='Desired Agent Positions', color='blue')
            plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            plt.xlim(-25, 25)  # Adjust the x-axis limits if needed
            plt.ylim(-25, 25)  # Adjust the y-axis limits if needed
            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.title('Agent Positions (Iteration {})'.format(iteration))
            plt.grid(True)
            plt.draw()
            plt.pause(0.1)
            
        plt.ioff()
        plt.show()
        

if "__main__"  == __name__: 
    fleet_size = 4
    ca = CollisionAvoidance(fleet_size)
    ca.shape_controller()