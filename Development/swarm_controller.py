import setup_path # Running with Python Version 3. .. .. tba
import airsim

import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.optimize import linear_sum_assignment
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp

import decision_maker as dm


class SwarmController:
    
    DEBUG = True

    def __init__(self, swarm_size):
        self.airsim_client = airsim.MultirotorClient()
        self.swarm_size = swarm_size
        self.drone_name_list = self.airsim_client.listVehicles()
        self.max_velocity = 16
        self.initial_position = np.array([])
        self.flight_operation_altitude = -5 # NED Z direction altitude
        
        # Drone setup
        self.airsim_setup()
        

    def airsim_setup(self):
        """
            Setup the Airsim Connection with Unreal Environment
            and arm drones/api control
        """
        # start drone client
        self.airsim_client = airsim.MultirotorClient()
        # Check connectivity
        self.airsim_client.confirmConnection()   
        # List of Drones in Simulation
        self.drone_names_list = self.airsim_client.listVehicles()
        self.swarm_size = len(self.drone_names_list)
        # Enable API/Arm drones 
        for drone in self.drone_names_list:
            self.airsim_client.enableApiControl(True, drone)
            self.airsim_client.armDisarm(True, drone)


    # Getter Functions: To grab needed values
    
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
        Get the positional information from a pose and returns the
        coordinates as a numpy array 
        """
        return np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val])
        
    
    def get_current_drone_positions(self):
        """ Retruns the current drone positions in the NED global coord frame """
        pose_list = self.get_absolute_poses_NED()
        position_matrix = []
        for pose in pose_list:
            position = self.get_pose_position(pose)
            position_matrix.append(position)
        position_matrix = np.array(position_matrix)
        # print(position_matrix)
        return position_matrix
        

    def get_laplacian(self, formation_id): # TODO make this dynamically generate formations for 5 quads 
                                           # if possible but also assign quads in posiitons
        laplacian = None
        if formation_id == 1:
            laplacian = np.array([
                    [2, -1, -1, 0],
                    [-1, 2, 0, -1],
                    [-1, 0, 2, -1],
                    [0, -1, -1, 2]
                ])
        elif formation_id == 2:
                laplacian = np.array([
                    [1, -1, 0, 0],
                    [-1, 2, -1, 0],
                    [0, -1, 2, -1],
                    [0, 0, -1, 1]
                ])
        return laplacian
    
    
    def get_desired_position(self, desired_formation_id):
        desired_final_positions_K = None
        if desired_formation_id == 1:
            desired_final_positions_K = np.array([  
                                      [2, 0, -5], # Drone 1
                                      [0, 0, -5],
                                      [0, 2, -5],
                                      [2, 2, -5]
                                    ])
        elif desired_formation_id == 2:
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [10, 0, self.flight_operation_altitude],
                                [20, 0, self.flight_operation_altitude],
                                [30, 0, self.flight_operation_altitude]
                            ])
        return desired_final_positions_K
    
    # Motion Control Commands that execute actions to the drones

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
            self.airsim_client.moveByVelocityAsync(xv, yv, zv, 0.1, vehicle_name=drone_name) 


    def takeoff_swarm(self):
        print(self.drone_names_list)
        for drone_name in self.drone_names_list:
            self.airsim_client.takeoffAsync(vehicle_name=drone_name).join()    

    # Check Functions

    def has_converged(self):
        """ Tells us the distance each drone is apart from one another """
        desired_positions = self.get_desired_position()
        current_position = self.get_current_drone_positions()
        total_distance = sum( [np.linalg.norm(current_position[i] - desired_positions[i]) for i in range(self.swarm_size)]  )
        convergence_threshold = 0.01
        print("Distance Error", total_distance)
        if total_distance <= convergence_threshold:
            print("has converged")
            return False
        else:
            return True
    
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
        

    # Controllers

    def formation_control_loop(self):
        """
            
        """
        # Create a Matplotlib figure for visualization
        plt.figure()
        plt.ion()
        
        # Simulation Parameters
        time_step = 0.1
        iteration = 0
        
        # Controller Gains
        gain_P = 1.0
        p_gain = 1
        d_gain = 1
        centroid_gain_P = 1.0
        
        # Formation Parameters
        desired_formation_id = 2
        desired_final_positions_K = self.get_desired_position(desired_formation_id)
        reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]), self.flight_operation_altitude ])
       
        # Start Simulation #

        # Have drones get airborne
        self.takeoff_swarm()
        
        # Decision Maker
        decision_maker = dm.DecisionMaker(self.swarm_size)
        decision_maker.swarm_position_selection()
        
        # Once all airborne formation controller will take over
        while True:
            # Control System Delay
            time.sleep(time_step)
            
            # Formation transformation should be decided here 
            #if iteration == 40: 
            #    desired_formation_id = 1
            #    desired_final_positions_K = self.get_desired_position(desired_formation_id)
            #    reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]) ])
            
            # Calculate Delta between current position - desired_position
            position = self.get_current_drone_positions()
            displacement = position - desired_final_positions_K
            # print("displacement: ", displacement)
            
            # Controller calculate desired velocity and new position
            laplacian = self.get_laplacian(desired_formation_id)
            tau_dot = np.dot( (-1 * laplacian), displacement)
            tau = ((tau_dot * time_step) + position)
            # print("tau_dot:", tau_dot)
            # print("tau:", tau)
            
            # Centroid Tracking: enables drones to converge more accurately in desired location
            current_swarm_centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]), np.mean(position[:, 2]) ])
            centroid_error = reference_centroid - current_swarm_centroid
            
            # Compute Control Signal
            control_signal = (gain_P * tau_dot) + (centroid_gain_P * centroid_error) 
            #print("control_signal:", control_signal)
            
            # Collision Avoidance 
            new_control_signals = self.collision_avoidance_new_velocities(control_signal, position)
            
            # Move swarm to desired location
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
   
        
if __name__ == "__main__":
    swarm_size = 4
    sc = SwarmController(swarm_size)
    sc.formation_control_loop()