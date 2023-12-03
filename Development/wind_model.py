
import numpy as np


class WindModel:

    DEBUG = True
    
    def __init__(self, wind_vector=[0,0,0]):
        """  """
        self.wind_vector = np.array(wind_vector)
    

    def calculate_relative_wind_direction(self, wind_velocity, drone_velocity):
        """"""
        # calc relative wind vector
        relative_wind_vector = wind_velocity - drone_velocity
        relative_wind_vector = relative_wind_vector[0:2]
        # calculate angle of relative wind vector
        relative_wind_angle_radians = np.arctan2(relative_wind_vector[1], relative_wind_vector[0])     
        relative_wind_angle_degrees = np.degrees(relative_wind_angle_radians) # Range is -180 to 180
        
        # Adjust angle to be in the range of 0-360 degrees
        if relative_wind_angle_degrees < 0:
            relative_wind_angle_degrees += 360 # 0 N, 90 E, 180 S, 270 W 
        
        if WindModel.DEBUG:
            print(
                "Relative Wind Vector:", relative_wind_vector,
                "\nRelative Windd Degrees:", relative_wind_angle_degrees
                )        

        if relative_wind_angle_degrees <= 45 and relative_wind_angle_degrees > 315:
            # North
            wind_direction = "N"
            return wind_direction
        elif relative_wind_angle_degrees <= 135 and relative_wind_angle_degrees > 45:
            # East
            wind_direction = "E"
            return wind_direction
        elif relative_wind_angle_degrees <= 225 and relative_wind_angle_degrees > 135:        
            # South
            wind_direction = "S"
            return wind_direction
        else:
            # West
            wind_direction = "W"
            return wind_direction
        
    
    def get_direction_of_wind_relative_to_drone(self, drone_name, drone_data):
        """
        :param: drone_data is the dictionary of drone, position, velocity data etc
        """
        
        # Getting velocity data needed to find direction 
        wind = self.wind_vector # (NED)
        drone_data[drone_name] 
        kinematics = drone_data[drone_name]['MultiRotorState'].kinematics_estimated
        drone_velocity = np.array([kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val])
        # computing wind direction
        wind_direction = self.calculate_relative_wind_direction(wind, drone_velocity)
        return wind_direction
    

    def get_direction_of_wind_relative_to_swarm(self, drone_names_list, drone_data):
        # Getting velocity data needed to find direction 
        wind = self.wind_vector # (NED)
        
        mean_velocity = np.array([0.0,0.0,0.0])
        drone_list_length = len(drone_names_list)
        for drone_name in drone_names_list:
            drone_data[drone_name] 
            kinematics = drone_data[drone_name]['MultiRotorState'].kinematics_estimated
            mean_velocity += np.array([kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val])
        # Compute Mean
        swarm_mean_velocity = mean_velocity/drone_list_length
        # computing wind direction
        wind_direction = self.calculate_relative_wind_direction(wind, swarm_mean_velocity)
        return wind_direction
    

    def set_wind_vector(self, wind_vector):
        self.wind_vector = wind_vector
        

    def get_wind_vector(self):
        return self.wind_vector