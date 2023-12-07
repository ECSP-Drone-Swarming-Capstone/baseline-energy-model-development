
import setup_path
import airsim
import numpy as np
from scipy.optimize import linear_sum_assignment


class DecisionMaker:
    
    DEBUG = True

    def __init__(self, swarm_data):
        self.airsim_client = airsim.MultirotorClient()   
        self.swarm_data = swarm_data
        self.drone_names = self.airsim_client.listVehicles()
        self.swarm_options = {
            "square": 1,
            "line": 2,
            "vee": 3,
            "diaganol": 4,
            "echelon": 5
            }
        self.flight_operation_altitude = -5

    # ################# #
    # Utility Functions #
    # ################# #
    def get_drone_position(self, drone_name):
        # Retrieve the drone's position from AirSim
        state = self.airsim_client.simGetObjectPose(drone_name)
        return np.array([state.position.x_val, state.position.y_val, state.position.z_val])
    
    
    # Get Drone Desired Positions
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


    def swarm_position_selection(self, formation_id):
        # Generate Laplacian Matrix For a Formation
        # selected_formation = self.select_formation()
        # Parameters coordinates of current drones and coordinates of desired position
        # Get Drone Current Positions
        current_drone_positions = []
        for drone_index in range(len(self.drone_names)):
            position = self.get_drone_position(self.drone_names[drone_index])
            current_drone_positions.append(position) 
        print("Current Drone Positions:", current_drone_positions)
        # Get Drone Desired Positions
        desired_swarm_positions = self.get_desired_position(formation_id)
        print("Desired Drone Positions:", desired_swarm_positions)
        # Calculate Distances from each point and store results for further processing
        distance_matrix = self.calculate_distance_matrix(current_drone_positions, desired_swarm_positions)
        # Apply Hungarian Algorithm to assign drones to smallest distance
        drone_indices, target_indices = self.assign_drones_to_targets(distance_matrix)
        updated_desired_swarm_positions = np.array( self.reorder_list( desired_swarm_positions.tolist(), target_indices) )
        print("Distance Matrix:", distance_matrix)
        print("drone_indices:", drone_indices, "target distances:", target_indices)
        print("Updated Desired Drone Positions:", updated_desired_swarm_positions)
        return updated_desired_swarm_positions
 
        # 2). It will need to assign drones to specific positions in the swarm that are close to their desired positions to minimize collisions
        # 3). measure distance between drones current positions andd their desired positions identify the minimum distance amongst them and positions drones based on this
        # 4). If there is a tie then we will select based on what points have the smaller next distance etc.