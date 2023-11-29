from tkinter import CURRENT
from turtle import speed
import setup_path
import airsim
import numpy as np
import networkx as nx
import time
import matplotlib.pyplot as plt
import geopy.distance


class FlightFormationController():
    

    def __init__(self, formation_size):
        # Initialize class instance variables
        
        # Flight Parameters
        self.flight_speed = 5 # verify units
        self.drone_altitude = -10 # NED must be negative to represent height
        self.formaation_size = formation_size
        self.minimum_distance = 1 # not sure what units, they are in SI though km
        
        # Setup Environment Parameters
        self.drone_names_list = []
        self.airsim_setup()
    

    def airsim_setup(self):
        """
            Setup the airsim connection and initialize drones
        """
        self.airsim_client = airsim.MultirotorClient()
        # Check connectivity
        self.airsim_client.confirmConnection()   
        for drone_id in range(1, self.formaation_size+1):
            drone_name = "Drone" + str(drone_id)
            self.drone_names_list.append(drone_name)
            self.airsim_client.enableApiControl(True, drone_name)
            self.airsim_client.armDisarm(True, drone_name)
    
    def get_drone_position_geopoint(self, drone_name):
        """
            
        """ 
        latitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.latitude
        longitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.longitude
        self.drone_altitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.altitude
        # print(latitude, y, z)
        return [float(latitude), float(longitude)]

    def get_drone_position_NED(self, drone_name):
        """
            Given a drone will return the actual position of the drone (truth)
        """
        #print("Vehicle states ", drone_name, self.airsim_client.getMultirotorState(vehicle_name=drone_name))
        x = self.airsim_client.simGetGroundTruthKinematics(vehicle_name=drone_name).position.x_val
        y = self.airsim_client.simGetGroundTruthKinematics(vehicle_name=drone_name).position.y_val
        z = self.airsim_client.simGetGroundTruthKinematics(vehicle_name=drone_name).position.z_val
        print(drone_name, x, y, z)
        return [round(x, 2), round(y, 2), round(z, 2)]


    def take_off(self):
        print(self.drone_names_list)
        for drone in self.drone_names_list:
            self.airsim_client.moveByVelocityZAsync(0, 0, self.drone_altitude, 5, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90), vehicle_name=drone)
        # Verify all drones reach flight height before attempting reformation
        

    def desired_formation_geometry(self, shape_name):
        """
            Assigns correct laplacian matrix for the specified formation desired
        """
        if (shape_name == "square"):
            laplacian_matrix_square = np.array([
                [2, -1, -1, 0],
                [-1, 2, 0, -1],
                [-1, 0, 2, -1],
                [0, -1, -1, 2]
            ])
            return laplacian_matrix_square
        return None
        

    def initial_positions(self):
        """
            Gets the current positions when called of all the drones in the swarm
        """
        current_position = []
        for drone in self.drone_names_list:
            current_coordinate = self.get_drone_position_geopoint(drone)
            current_position.append(current_coordinate)
            print(current_position)
        numpy_current_position_array = np.array(current_position)
        print(numpy_current_position_array)
        return numpy_current_position_array 

    '''def initial_positions(self):
        """
            Gets the current positions when called of all the drones in the swarm
        """
        current_position = []
        for drone in self.drone_names_list:
            current_coordinate = self.get_drone_position_NED(drone)
            #initial_position_dictionary[drone] = coordinate
            current_position.append(current_coordinate)
            print(current_position)
        numpy_current_position_array = np.array(current_position)
        print(numpy_current_position_array)
        return numpy_current_position_array'''


    def initialize_formation_controller(self):
        self.take_off()
        self.consensus_control_loop()
        

    def consensus_control_loop(self):
        airsim.wait_key("Press any key to execute formation controller")
        iteration = 0
        max_loop_iterations = 300
        position = self.initial_positions()
        lap_matrix = self.desired_formation_geometry("square")
        time.sleep(0.1)
        while iteration < max_loop_iterations:
           
            laplacian_term = np.dot(lap_matrix, position)
            for i in range(self.formaation_size):
                for j in range(self.formaation_size):
                    if i != j:
                        current_distance_km = geopy.distance.geodesic(position[i], position[j]).kilometers
                        correction = 0.2 * (self.minimum_distance - current_distance_km) * (position[j] - position[i])
                        position[i] += laplacian_term[i] + correction/2
                        position[j] -= laplacian_term[j] - correction/2

            '''laplacian_term = np.dot(lap_matrix, position)
            pairwise_distance = np.linalg.norm(position[:, np.newaxis] - position, axis=2)
            correction = 0.1 * (pairwise_distance - self.minimum_distance)
            position -= laplacian_term - np.sum(correction, axis = 2)
            desired_position = position
            time.sleep(0.1)'''
            time.sleep(0.1)
            # Move to desired position
            for index, drone in enumerate(self.drone_names_list):
                self.airsim_client.moveToGPSAsync(position[index, 0], position[index, 1], self.drone_altitude, 1.5, 5, vehicle_name=drone)
                # self.airsim_client.moveToPositionAsync(desired_position[index, 0], desired_position[index, 1], self.drone_altitude, 1.5, 5, vehicle_name=drone)
            iteration += 1
        airsim.wait_key("Ended")


if __name__ == "__main__":
    fleet_size = 4
    fc = FlightFormationController(fleet_size)
    fc.initialize_formation_controller()