from tkinter import CURRENT
from turtle import speed
import setup_path
import airsim
import numpy as np
import networkx as nx
import time
import matplotlib.pyplot as plt
import geopy.distance


class FlightFormationUtilities():


    def get_drone_position_geopoint(self, drone_name):
        """
            Returns a drones gps position
        """ 
        world_frame_pose_NED = self.airsim_client.simGetObjectPose(drone_name)
        #print("World Frame Pose of Drone:", drone_name, world_frame_pose)
        
        # Old Way of getting positions globally
        #latitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.latitude
        #longitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.longitude
        #self.drone_altitude = self.airsim_client.getMultirotorState(vehicle_name=drone_name).gps_location.altitude
        # print(latitude, y, z)
        #return [float(latitude), float(longitude)]


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
        print("Test: Verifying printing of drones positions in global frame - ", drone_poses_NED)
        return drone_poses_NED
    
            
    def get_distances(self):
        pass