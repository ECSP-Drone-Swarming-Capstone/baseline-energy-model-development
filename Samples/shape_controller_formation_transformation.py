from tkinter import CURRENT
from turtle import speed
from airsim.types import DrivetrainType, YawMode
import setup_path
import airsim
import numpy as np
import networkx as nx
import time
import matplotlib.pyplot as plt
import geopy.distance
import threading


class ShapeController:
    """Simple Airsim Implementation of a discrete shape_controller for static motion transformation"""
    

    def __init__(self, formation_size):
        # Drone Information
        self.formation_size = formation_size
        self.drone_names_list = []        

        # Setup Environment Parameters
        self.airsim_setup()
        self.initial_position = np.array([])
    

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
        return np.array([pose.position.x_val, pose.position.y_val])
        
    
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
        

    def get_laplacian(self, formation_id):
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
                                          [10, 0], # Drone 1
                                          [0, 0],
                                          [0, 10],
                                          [10, 10]
                                        ])
        elif desired_formation_id == 2:
            desired_final_positions_K = np.array([  
                                [0, 0], # Drone 1
                                [10, 0],
                                [20, 0],
                                [30, 0]
                            ])
        return desired_final_positions_K


    def drone_motion_control(self, tau_dot):
        """Assigns a set velocity for the drones to reach their desired destination. """
        for index, drone_name in enumerate(self.drone_names_list):
            drone_new_velocity = tau_dot[index] 
            xv = drone_new_velocity[0]
            yv = drone_new_velocity[1] 
            print("velocityL", xv, yv)
            self.airsim_client.moveByVelocityAsync(xv, yv, 0, 0.1, vehicle_name=drone_name) # not working


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
        


    def shape_controller(self):
        
        # Create a Matplotlib figure for visualization
        plt.figure()
        plt.ion()
        time_step = 0.1
        iteration = 0
        gain_P = 1.0
        centroid_gain_P = 1.0
        desired_formation_id = 2
        desired_final_positions_K = self.get_desired_position(desired_formation_id)
        reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]) ])
       
        # Start Simulation
        self.takeoff_swarm()
        #my_takeoff_thread = threading.Thread(self.takeoff_swarm())
        #my_takeoff_thread.start()
        #my_takeoff_thread.join() # wait for it to finish then do shape formation
        while True: # self.has_converged()
            # Calculate Delta between current position - desired_position
            if iteration == 40: # Change Formation
                desired_formation_id = 1
                desired_final_positions_K = self.get_desired_position(desired_formation_id)
                reference_centroid = np.array([np.mean(desired_final_positions_K[:, 0]), np.mean(desired_final_positions_K[:, 1]) ])
                #reference_centroid = np.array([10,20])
            time.sleep(time_step)
            position = self.get_current_drone_positions()
            displacement = position - desired_final_positions_K
            print("displacement: ", displacement)
            # Calculate tau_dot aka the velocity adjustment
            laplacian = self.get_laplacian(desired_formation_id)
            tau_dot = np.dot( (-1 * laplacian), displacement)
            
            print("tau_dot:", tau_dot)
            
            tau = ((tau_dot * time_step) + position)
            print("tau:", tau)
            
            # Centroid: enables drones to converge more accurately in desired location
            current_swarm_centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]) ])
            # centroid_ref = desired_destination_to_move
            centroid_error = reference_centroid - current_swarm_centroid
            
            control_signal = (gain_P * tau_dot) + (centroid_gain_P * centroid_error) 
            print("control_signal:", control_signal)

            self.drone_motion_control(control_signal)
            
            # position = tau + centroid_error
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
    fleet_size = 4
    sc = ShapeController(fleet_size)
    sc.shape_controller()