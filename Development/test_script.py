import profile
import setup_path
import airsim
import numpy as np
import time
import matplotlib.pyplot as plt

from wind_model import WindModel
from DroneSwarmEnergyModel import DroneSwarmPowerModel 

# D:\Documents\AirSim contains settings.json folder

class QuadController:
    """ Utilized to collect data on quadcopter dynamics """
    
    DEBUG = True

    def __init__(self):
        # Drone Information
        self.drone_names_list = []        
        self.formation_size = 4
        # Setup Environment Parameters
        self.airsim_setup()
        self.initial_position = np.array([])
        wind_vector = [0,-1,0] #(NED), East (0,1,0), North (1,0,0), West (0,-1,0), South (-1, 0, 0)
        # self.wind_direction = "East+" 
        self.wind = WindModel(wind_vector)
    

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
            
    # ################ #
    # Getter Functions #   
    # ################ #

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
        return np.array([pose.position.x_val, pose.position.y_val])
    
    
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
        # print(position_matrix)
        return position_matrix
    
    
    def get_desired_position(self):
        desired_final_positions_K = np.array([  
                                      [10, 0], # Drone 1
                                      [0, 0],
                                      [0, 10],
                                      [10, 10]
                                    ])
        return desired_final_positions_K
    

    def get_desired_altitude_NED(self):
        """ Returns Desired NED Altitude """
        desired_altitude = np.array([  
                                -5,
                                -5,
                                -5,
                                -5
                            ])
        return desired_altitude


    def get_drone_altitude(self):
        pose_list = self.get_absolute_poses_NED()
        altitude_list = []
        for pose in pose_list:
            altitude = self.get_pose_altitude(pose)
            print(altitude)
            altitude_list.append(altitude[0])
        altitude_list = np.array(altitude_list)
        return altitude_list
            
    
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
            multirotor_state = self.airsim_client.getMultirotorState(vehicle_name=drone)
            rotor_state_info = self.airsim_client.getRotorStates(vehicle_name=drone)
            drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone)
            air_density = self.airsim_client.simGetGroundTruthEnvironment(drone).air_density
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
        
        if QuadController.DEBUG:
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
    

    def drone_data_collection(self):
        """ Prints out data to the console of Drone1. """
        #drone_name = "Drone1"
        #multirotor_state = self.airsim_client.getMultirotorState(vehicle_name=drone_name)
        #rotor_state_info = self.airsim_client.getRotorStates(vehicle_name=drone_name)
        #drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone_name)
        
        dpm = DroneSwarmPowerModel()
        # This Power Calculation should be executed once a second
        copied_airsim_state_data = self.copy_drone_swarm_data()
        # State: Hover or Flight
        #power_calculation_thread = Thread(target=dpm.power_model, args=("Flight", "Vee", self.wind_vector,"East+", self.drone_names_list, copied_airsim_state_data))
        #power_calculation_thread.start()
        #power_usage = dpm.swarm_power_consumption_model("Hover", "Vee", self.wind_vector,"East+", self.drone_names_list, copied_airsim_state_data)
        direction_drone_wind = self.wind.get_direction_of_wind_relative_to_drone("Drone1", copied_airsim_state_data)
        direction_swarm_wind = self.wind.get_direction_of_wind_relative_to_swarm(self.drone_names_list, copied_airsim_state_data)
        mode_of_flight = self.detect_swarm_mode_of_flight(copied_airsim_state_data)
        wind_vector = self.wind.get_wind_vector()
        
        power_usage = dpm.drone_power_consumption_model(mode_of_flight, wind_vector, direction_drone_wind, "Drone1", copied_airsim_state_data)
        if QuadController.DEBUG:
            print("Data Collection",
                  "\nWind Direction:", direction_drone_wind,
                  "\nMode of Flight:", mode_of_flight,
                )
        return power_usage
        #print("MultiRotor State Information:", multirotor_state)
        #print("************************************************\n")
        #print("Rotor State Information:", rotor_state_info, "\n")
        #print("**********************************************************")
        #print("WNED of drone", drone_pose_WFNED, "\n")
        #print("**********************************************************")
        # print("Drone Energy Usage in Watts", power_usage, "\n")
        #print("**********************************************************")
   
    # ################## #
    # Plotting Functions #
    # ################## #
    def update_plot(self, x, y1, y2):
        """
            Updates plots
        """
        # , self.position_data, self.power_data
        self.iteration_time_step.append(x)
        self.position_data.append(y1)
        self.power_data.append(y2)
        self.line1.set_data(self.iteration_time_step, self.position_data)
        self.line2.set_data(self.iteration_time_step, self.power_data)

        # Adjust limits
        self.position_ax1.relim()
        self.position_ax1.autoscale_view()
        self.power_ax2.relim()
        self.power_ax2.autoscale_view()

        self.fig.canvas.draw()
        plt.pause(0.1)
        self.fig.canvas.flush_events()

    # ############## #
    # Motion Control #
    # ############## #

    def drone_motion_control_alt(self, altitude_control_signal):
        """Assigns a set velocity for the drones to reach their desired destination. """
        #for index, drone_name in enumerate(self.drone_names_list):
        drone_name = "Drone1"
        self.airsim_client.moveByVelocityAsync(17, 0, altitude_control_signal[0], 0.1, vehicle_name=drone_name) # not working
        
        #worker_thread = Thread(target=self.drone_data_collection)
        #worker_thread.start()
        

    def takeoff_swarm(self):
        print(self.drone_names_list)
        for drone_name in self.drone_names_list:
            self.airsim_client.takeoffAsync(vehicle_name=drone_name).join()
       
            
    def drone_motion_control(self, tau_dot):
        """Assigns a set velocity for the drones to reach their desired destination. """
        for index, drone_name in enumerate(self.drone_names_list):
            drone_new_velocity = tau_dot[index] 
            xv = drone_new_velocity[0]
            yv = drone_new_velocity[1] 
            print("velocityL", xv, yv)
            self.airsim_client.moveByVelocityAsync(xv, yv, 0, 0.1, vehicle_name=drone_name) # not working


    def altitude_controller(self):
        # Plotting Setup
        plt.ion()
        plt.grid(True)
        #self.fig, (self.position_ax1, self.power_ax2) = plt.subplots(2,1)
        #self.iteration_time_step, self.position_data, self.power_data = [], [], []
        #self.line1, = self.position_ax1.plot([], [], 'r-', animated=True)
        #self.line2, = self.power_ax2.plot([], [], 'b-', animated=True)
        
        # Set labels and titles for each subplot
        #self.position_ax1.set_xlabel('Time (s)')
        #self.position_ax1.set_ylabel('Energy (Watts)')
        #self.position_ax1.set_title('Energy Output vs. Time')

        #self.power_ax2.set_xlabel('Time (s)')
        #self.power_ax2.set_ylabel('Position (meters)')
        #self.power_ax2.set_title('Position vs. Time')

        # Simulation and Control Parameters
        time_step = 0.1
        iteration = 0
        p_gain = 1
        d_gain = 1
        #self.takeoff_swarm()
        while True:
            # Wait for a time 
            time.sleep(time_step)
            desired_altitude = self.get_desired_altitude_NED()
            #current_drone_pose = self.get_current_drone_positions()
            current_drone_altitude = self.get_drone_altitude()
            print("current drone alti:", current_drone_altitude)
            # PID Altitude Controller 
            error = desired_altitude - current_drone_altitude # Whatever the size of the swarm desired should match
            print("Error", error)
            perror = p_gain * error
            derror = d_gain * (error/time_step)
            control_signal_altitude = perror + derror
            self.drone_motion_control_alt(control_signal_altitude)
            power = self.drone_data_collection()
            iteration = iteration + 1
            #self.update_plot(iteration, current_drone_altitude[0], power)
            
            #plt.clf()
            # Power
            plt.scatter(iteration, power, label='Power Consumption Hover (Watts)', color='red')
            #plt.scatter(5, desired_altitude[0], label='Desired Agent Altitude', color='blue')
            #plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            #plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            #plt.xlim(-25, 25)  # Adjust the x-axis limits if needed
            #plt.ylim(-25, 25)  # Adjust the y-axis limits if needed
            plt.xlabel('Iteration')
            plt.ylabel('Power Consumption Watts')
            plt.title('Power Consumption')
            plt.grid(True)
            # Position
            #plt.scatter(5,  current_drone_altitude[0], label='Current Agent Altitude', color='red')
            #plt.scatter(5, desired_altitude[0], label='Desired Agent Altitude', color='blue')
            #plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            #plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            #plt.xlim(-25, 25)  # Adjust the x-axis limits if needed
            #plt.ylim(-25, 25)  # Adjust the y-axis limits if needed
            #plt.xlabel('X Position')
            #plt.ylabel('Y Position')
            #plt.title('Agent Positions (Iteration {})'.format(iteration))
            #plt.grid(True)
            plt.draw()
            plt.pause(0.1)


if __name__ == "__main__":
    qc = QuadController()
    qc.altitude_controller()