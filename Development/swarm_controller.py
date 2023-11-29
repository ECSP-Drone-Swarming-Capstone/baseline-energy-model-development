import numpy as np


class SwarmController:
    

    def __init__(self, airsim_client_interface, swarm_size, drone_name_list):
        self.airsim_client = airsim_client_interface
        self.swarm_size = swarm_size
        self.drone_name_list = drone_name_list
        
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
    
    # Motion Control Commands that execute actions to the drones

    def drone_motion_control(self, tau_dot):
        """Assigns a set velocity for the drones to reach their desired destination. """
        for index, drone_name in enumerate(self.drone_names_list):
            drone_new_velocity = tau_dot[index] 
            xv = drone_new_velocity[0]
            yv = drone_new_velocity[1] 
            print("velocityL", xv, yv)
            self.airsim_client.moveByVelocityAsync(xv, yv, 0, 0.1, vehicle_name=drone_name) # not working

    def takeoff_swarm(self):
        print(self.drone_names_list)
        for drone_name in self.drone_names_list:
            self.airsim_client.takeoffAsync(vehicle_name=drone_name).join()    

    # Check Functions

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
        
    # Controllers
