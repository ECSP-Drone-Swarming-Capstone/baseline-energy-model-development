import setup_path # Running with Python Version 3. .. .. tba
import airsim

import numpy as np
import matplotlib.pyplot as plt
import time
from threading import Thread
from datetime import datetime
import csv
# from scipy.optimize import linear_sum_assignment
from pyorca import Agent, get_avoidance_velocity, orca# normalized, perp
from queue import Queue

import decision_maker as dm
from wind_model import WindModel
from DroneSwarmEnergyModel import DroneSwarmPowerModel 

class SwarmController:
    
    DEBUG = True

    def __init__(self, swarm_size):
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_data_collection_client = airsim.MultirotorClient()
        self.swarm_size = swarm_size
        self.drone_name_list = self.airsim_client.listVehicles()
        self.max_velocity = 16
        self.initial_position = np.array([])
        self.flight_operation_altitude = -5 # NED Z direction altitude
        wind_vector = [5,0,0] #(NED), East (0,1,0), North (1,0,0), West (0,-1,0), South (-1, 0, 0)
        # self.wind_direction = "East+" 
        self.wind = WindModel(wind_vector)
        self.path = np.array([])
        
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
            #laplacian = np.array([
            #        [2, -1, -1],
            #        [-1, 2, 0],
            #        [-1, 0, 2]
            #    ])
            laplacian = np.array([ # Square
                    [2, -1, -1, 0],
                    [-1, 2, 0, -1],
                    [-1, 0, 2, -1],
                    [0, -1, -1, 2]
                ])
        elif formation_id == 2: # Front
                laplacian = np.array([
                    [1, -1, 0, 0, 0],
                    [-1, 2, -1, 0, 0],
                    [0, -1, 2, -1, 0],
                    [0, 0, -1, 2, -1],
                    [0, 0, 0, -1, 1]
                ])
        elif formation_id == 3: # Echelon
                laplacian = np.array([
                    [1, -1, 0, 0, 0],
                    [-1, 2, -1, 0, 0],
                    [0, -1, 2, -1, 0],
                    [0, 0, -1, 2, -1],
                    [0, 0, 0, -1, 1]
                ])
        elif formation_id == 4: # Diamond
                laplacian = np.array([
                    [4, -1, -1, -1, -1],
                    [-1, 1, 0, 0, 0],
                    [-1, 0, 1, 0, 0],
                    [-1, 0, 0, 1, 0],
                    [-1, 0, 0, 0, 1]
                ])
        elif formation_id == 5: # Column
                laplacian = np.array([
                    [1, -1, 0, 0, 0],
                    [-1, 2, -1, 0, 0],
                    [0, -1, 2, -1, 0],
                    [0, 0, -1, 2, -1],
                    [0, 0, 0, -1, 1]
                ])
        elif formation_id == 6: # Vee
                laplacian = np.array([
                    [2, -1, -1, 0, 0],
                    [-1, 2, 0, -1, 0],
                    [-1, 0, 2, 0, -1],
                    [0, -1, 0, 1, 0],
                    [0, 0, -1, 0, 1]
                ])
                #laplacian = np.array([
                #    [2, -1, -1],
                #    [-1, 2, 0],
                #    [-1, 0, 2]
                #])
                
        return laplacian
    
    
    def get_desired_position(self, desired_formation_id):
        desired_final_positions_K = None
        scale = 1
        if desired_formation_id == 1: # Square
            desired_final_positions_K = np.array([  
                                      [2, 0, self.flight_operation_altitude], # Drone 1
                                      [0, 0, self.flight_operation_altitude],
                                      [0, 2, self.flight_operation_altitude],
                                      [2, 2, self.flight_operation_altitude]
                                    ])
        elif desired_formation_id == 2: # Front
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [2, 0, self.flight_operation_altitude],
                                [4, 0, self.flight_operation_altitude],
                                [6, 0, self.flight_operation_altitude],
                                [8, 0, self.flight_operation_altitude]
                            ])
        elif desired_formation_id == 3: # Echelon
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [2, 2, self.flight_operation_altitude],
                                [4, 4, self.flight_operation_altitude],
                                [6, 6, self.flight_operation_altitude],
                                [8, 8, self.flight_operation_altitude]
                            ])  
        elif desired_formation_id == 4: # Diamond
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [0, 2, self.flight_operation_altitude],
                                [2, 0, self.flight_operation_altitude],
                                [0, -2, self.flight_operation_altitude],
                                [-2, 0, self.flight_operation_altitude]
                            ])
        elif desired_formation_id == 5: # Column
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [0, 2, self.flight_operation_altitude],
                                [0, 4, self.flight_operation_altitude],
                                [0, 6, self.flight_operation_altitude],
                                [0, 8, self.flight_operation_altitude]
                            ])
        elif desired_formation_id == 6: # Vee
            desired_final_positions_K = np.array([  
                                [0, 0, self.flight_operation_altitude], # Drone 1
                                [2, -2, self.flight_operation_altitude],
                                [-2, -2, self.flight_operation_altitude],
                                [4, -4, self.flight_operation_altitude],
                                [-4, -4, self.flight_operation_altitude]
                            ])
        return scale * desired_final_positions_K
    
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
            yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=0) 
            self.airsim_client.moveByVelocityAsync(xv, yv, zv, 0.1, vehicle_name=drone_name, yaw_mode=yaw_mode) 


    def takeoff_swarm(self):
        print(self.drone_names_list)
        m = []
        for drone_name in self.drone_names_list:
            h = self.airsim_client.takeoffAsync(vehicle_name=drone_name)  
            m.append(h)
        
        for i in m:
            i.join()  

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
    
    
    def detect_collisions(self, future_positions, collision_threshold): # 1.2
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
        max_speed = 17 # m/s
        radius = 0.5 # meter
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
    
    # ############### #
    # Data Collection #
    # ############### #

    def copy_drone_swarm_data(self):
        """ 
            Copies data of current drone swarm to be used by power model.
            This is to prevent blocking slow down in the control system.
            General Structure:
                drone_state_dict["drone"] = {
                                        "MultiRotorState": multirotor_state,
                                        "RotorState": rotor_state_info,
                                        "PoseWFNED": drone_pose_WFNED,
                                        "AirDensity": air_density
                                        }
        """
        multirotor_states = None
        rotor_states = None
        drone_pose_WFNED = None
        air_density = None        
        drone_state_dict = {}
        for drone in self.drone_names_list:
            multirotor_state = self.airsim_data_collection_client.getMultirotorState(vehicle_name=drone)
            rotor_state_info = self.airsim_data_collection_client.getRotorStates(vehicle_name=drone)
            drone_pose_WFNED = self.airsim_data_collection_client.simGetObjectPose(drone)
            air_density = self.airsim_data_collection_client.simGetGroundTruthEnvironment(drone).air_density
            drone_state_dict[drone] = {
                                        "MultiRotorState": multirotor_state,
                                        "RotorState": rotor_state_info,
                                        "PoseWFNED": drone_pose_WFNED,
                                        "AirDensity": air_density
                                        }
        return drone_state_dict            


    def detect_swarm_mode_of_flight(self, drone_data):
        mean_velocity = np.array([0.0,0.0,0.0])
        drone_list_length = len(self.drone_names_list)
        for drone_name in self.drone_names_list:
            drone_data[drone_name] 
            kinematics = drone_data[drone_name]['MultiRotorState'].kinematics_estimated
            mean_velocity += np.array([kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val])
        mean_velocity = mean_velocity/drone_list_length
        
        if SwarmController.DEBUG:
            print(
                "Flight Mode",
                "\nMean Velocity:", mean_velocity
            )

        # Check the mode of flight the drones are in
        # if North (x), East (y) average is less than 0.15 m/s then we say we are hovering, we could also add position has not changed a lot as well
        if abs(mean_velocity[0]) == 0 and abs(mean_velocity[1]) == 0:
            return "Hover"
        else:
            return "Flight"
   
    
    def save_data_to_csv(self):
        # Get current date and time
        current_time = datetime.now()
        # Format the time in a file-friendly format (e.g., '2023-03-21_15-30-00')
        timestamp = current_time.strftime('%Y-%m-%d_%H-%M-%S')
        filename = f'data_log_{timestamp}.csv'
        dpm = DroneSwarmPowerModel()
        try:
            while True:
                with open(filename, 'a+', newline='') as file:
                    writer = csv.writer(file)
                    if file.tell() == 0:
                        writer.writerow(['Time', 'Power (Watts)', 'Flight Mode', 'Wind Speed (m/s)', 'Wind Direction (NESW)', 'Velocity m/s'])  # Writing header
            
                    # Data collection
                    current_time = time.time()
                    drone_data = self.copy_drone_swarm_data()
                    drone_names_list = drone_data.keys()
                    kinematics = drone_data["Drone1"]['MultiRotorState'].kinematics_estimated
                    velocity_drone_vector = np.array([kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val])
                    speed_drone = np.linalg.norm(velocity_drone_vector)
                    #direction_drone_wind = self.wind.get_direction_of_wind_relative_to_drone("Drone1", drone_data)
                    direction_swarm_wind = self.wind.get_direction_of_wind_relative_to_swarm(self.drone_names_list, drone_data)
                    mode_of_flight = self.detect_swarm_mode_of_flight(drone_data)
                    wind_vector = self.wind.get_wind_vector()
                    wind_speed = np.linalg.norm(wind_vector)
                    #power_usage = dpm.drone_power_consumption_model(mode_of_flight, wind_vector, direction_drone_wind, "Drone1", drone_data)
                    print("Power is Dead ------------------------------------------------------------------------------------------------------")
                    power_usage = dpm.swarm_power_consumption_model( mode_of_flight, "Vee", wind_vector, direction_swarm_wind, drone_names_list, drone_data)
                
                    # Record Data
                    writer.writerow([current_time, power_usage, mode_of_flight, wind_speed, direction_swarm_wind, speed_drone])
                    if SwarmController.DEBUG:
                        print("Data Collection",
                        "\nWind Direction:", direction_swarm_wind,
                        "\nMode of Flight:", mode_of_flight,
                        )
                    file.close()
                time.sleep(2)  # Wait for 1 second
        except Exception as e:
            print(f"An error occurred: {e}")  


    def save_individual_drone_data_to_csv(self):
        # Get current date and time
        current_time = datetime.now()
        # Format the time in a file-friendly format (e.g., '2023-03-21_15-30-00')
        timestamp = current_time.strftime('%Y-%m-%d_%H-%M-%S')
        filename = f'data_log_{timestamp}.csv'
        dpm = DroneSwarmPowerModel()
        try:
            while True:
                with open(filename, 'a+', newline='') as file:
                    writer = csv.writer(file)
                    if file.tell() == 0:
                        writer.writerow(['Time', 'Net Power (Watts)', 'Flight Mode', 'Wind Speed (m/s)', 'Wind Direction (NESW)', 'Velocity m/s'])  # Writing header
            
                    # Data collection
                    
                    current_time = time.time()
                    drone_data = self.copy_drone_swarm_data()
                    drone_names_list = drone_data.keys()
                    kinematics = drone_data["Drone1"]['MultiRotorState'].kinematics_estimated
                    velocity_drone_vector = np.array([kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val])
                    speed_drone = np.linalg.norm(velocity_drone_vector)
                    direction_drone_wind = self.wind.get_direction_of_wind_relative_to_drone("Drone1", drone_data)
                    #direction_swarm_wind = self.wind.get_direction_of_wind_relative_to_swarm(self.drone_names_list, drone_data)
                    mode_of_flight = self.detect_swarm_mode_of_flight(drone_data)
                    wind_vector = self.wind.get_wind_vector()
                    wind_speed = np.linalg.norm(wind_vector)
                    net_power_usage = 0
                    for drone in drone_names_list:
                        power_usage = dpm.drone_power_consumption_model(mode_of_flight, wind_vector, direction_drone_wind, drone, drone_data)
                        net_power_usage += power_usage
                    print("Power is Dead ------------------------------------------------------------------------------------------------------")
                    # power_usage = dpm.swarm_power_consumption_model( mode_of_flight, "Vee", wind_vector, direction_swarm_wind, drone_names_list, drone_data)
                
                    # Record Data
                    writer.writerow([current_time, net_power_usage, mode_of_flight, wind_speed, direction_drone_wind, speed_drone])
                    if SwarmController.DEBUG:
                        print("Data Collection",
                        "\nWind Direction:", direction_drone_wind,
                        "\nMode of Flight:", mode_of_flight,
                        )
                    file.close()
                time.sleep(1)  # Wait for 1 second
        except Exception as e:
            print(f"An error occurred: {e}") 

    # Controllers
    def simple_drone_control(self):
        distance_to_travel = np.array([100, 0, 0]) # meters 
        
        # Begin Data Recording
        # Create and start the data collection thread
        data_thread = Thread(target=self.save_data_to_csv)
        data_thread.daemon = True  # Daemonize thread
        data_thread.start()        

        initial_positions = self.get_current_drone_positions()
        desired_positions = initial_positions + distance_to_travel
        print("Desired_Positions", desired_positions)
        moving = []
        self.takeoff_swarm()        

        for index, drone_name in enumerate(self.drone_name_list):
            drone_desired_position = desired_positions[index]
            v = self.airsim_client.moveToPositionAsync(drone_desired_position[0], drone_desired_position[1], self.flight_operation_altitude, 5, vehicle_name=drone_name)
            moving.append(v)
        
        for item in moving:
            item.join()
        data_thread.join()


    def linear_motion_plan(self, distance, dist_step, current_ref):
        path = Queue()
        for x in range(0, distance, dist_step):
            path.put( current_ref + np.array([ 0, x, 0 ]) )
        return path      


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
        # d_gain = 1
        centroid_gain_P = 1.0
        
        # Decision Maker
        decision_maker = dm.DecisionMaker(self.swarm_size)
        

        # Formation Parameters
        desired_formation_id = 6
        desired_final_positions_K = decision_maker.swarm_position_selection(desired_formation_id) # self.get_desired_position(desired_formation_id)
        reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]), self.flight_operation_altitude])
       
        # Start Simulation #

        # Have drones get airborne
        self.takeoff_swarm()

        # Motion Plan
        self.path = self.linear_motion_plan(100, 10, reference_centroid)
        
        # Collision Avoidance Parameters
        delta_t = 0.025
        collision_distance = 1.25

        # Begin Data Recording
        # Create and start the data collection thread
        data_thread = Thread(target=self.save_data_to_csv)
        data_thread.daemon = True  # Daemonize thread
        data_thread.start()
        
        # Keep Track of Trajectory
        previous_target = desired_final_positions_K

        # Once all airborne formation controller will take over
        while True:
            # Control System Delay
            time.sleep(time_step)
            
            # Formation transformation should be decided here 
            #if iteration == 50: 
            #    desired_formation_id = 1
            #    desired_final_positions_K = self.get_desired_position(desired_formation_id)
            #    reference_centroid = np.array([0,20, self.flight_operation_altitude]) #np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]), self.flight_operation_altitude ])
                #current_swarm_centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]), np.mean(position[:, 2]) ])
                #centroid_displacement = reference_centroid - current_swarm_centroid
                #translation = np.array([ centroid_displacement[0], centroid_displacement[1], 0 ])
                #previous_target = previous_target + translation
                #desired_final_positions_K = self.get_desired_position(desired_formation_id)
            if iteration > 111:
                desired_formation_id = 4
                desired_final_positions_K = decision_maker.swarm_position_selection(desired_formation_id) # self.get_desired_position(desired_formation_id)
                
            if iteration > 100 and not self.path.empty():
                current_swarm_centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]), np.mean(position[:, 2]) ])
                if np.linalg.norm(reference_centroid - current_swarm_centroid) <= 5:
                    reference_centroid = self.path.get()
                    print("new_point:", reference_centroid)
                
                    
                
            #if iteration == 40:
            #    reference_centroid = np.array([0,10, self.flight_operation_altitude])

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
            print("Centroid Error:", centroid_error, "=", reference_centroid, "-", current_swarm_centroid)
            
            # Compute Control Signal
            control_signal = (gain_P * tau_dot) + (centroid_gain_P * centroid_error) 
            #print("control_signal:", control_signal)
            
            # Collision Avoidance
            predicted_position = self.predict_future_positions(delta_t) 
            collisions = self.detect_collisions(predicted_position, collision_distance)
            if len(collisions) > 0:
                print("Collision Detected")
                control_signal = self.collision_avoidance_new_velocities(control_signal, position)
            
            # Move swarm to desired location
            # self.drone_motion_control(new_control_signals)
            self.drone_motion_control(control_signal)
            iteration = iteration + 1
            plt.clf()
            plt.scatter(position[:, 0], position[:, 1], label='Current Agent Positions', color='red')
            plt.scatter(desired_final_positions_K[:, 0] + reference_centroid[0], desired_final_positions_K[:,1] + reference_centroid[1], label='Desired Agent Positions', color='blue')
            plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            plt.xlim(-30, 30)  # Adjust the x-axis limits if needed
            plt.ylim(-30, 30)  # Adjust the y-axis limits if needed
            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.title('Agent Positions (Iteration {})'.format(iteration))
            plt.grid(True)
            plt.draw()
            plt.pause(0.1)
            
        plt.ioff()
        plt.show()
   
        
if __name__ == "__main__":
    swarm_size = 5
    sc = SwarmController(swarm_size)
    sc.formation_control_loop()
    # sc.simple_drone_control()