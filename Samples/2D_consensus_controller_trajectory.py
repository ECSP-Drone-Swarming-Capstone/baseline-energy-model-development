import numpy as np
import time
import matplotlib.pyplot as plt
import geopy.distance
from scipy.integrate import solve_ivp

# This version is a timestep driven controller but also enables translation

# Define the number of agents and their initial + desired positions
num_agents = 4  

initial_positions = np.array([
                              [20.0, 20.0], # Drone 1
                              [22.0, 25.0],
                              [24.0, 21.0],
                              [23.0, 20.0]
                            ])

desired_final_positions_K = np.array([  
                                      [20.0, 20.0], # Drone 1
                                      [22.0, 20.0],
                                      [22.0, 22.0],
                                      [20.0, 22.0]
                                    ])

desired_destination_to_move = np.array([
                                        [-5, -5] # centroid should shift too
                                        ])


# Hard coded, ideally will be generated before mission
#path = np.array([
    
 #   ])

# Define desired inter-agent distances (for simplicity, let's assume equal distances)
desired_distance = 2.0

# Define control gain parameter
k = 0.1

# Define simulation parameters
num_iterations = 80
time_step = 0.1

# Initialize agent velocities
velocities = np.zeros((num_agents, 2))
    
# Updated Laplacian 
laplacian = np.array([
                [2, -1, -1, 0],
                [-1, 2, 0, -1],
                [-1, 0, 2, -1],
                [0, -1, -1, 2]
            ]) 
# Assign Position
position = initial_positions
scale_factor = 1

# Create a Matplotlib figure for visualization
plt.figure()

# Start Simulation
for iteration in range(0, num_iterations):
    # Calculate Delta between current position - desired_position
    displacement = position - desired_final_positions_K
    print("displacement: ", displacement)
    # Calculate tau_dot aka the velocity adjustment
    tau_dot = np.dot( (-1 * laplacian), displacement)
    print("tau_dot:", tau_dot)
    tau = scale_factor * ((tau_dot * time_step) + position) # Missing constant when taking derivative this is the initial position
    print("tau:", tau)
    
    centroid = np.array([np.mean(position[:, 0]), np.mean(position[:, 1]) ])
    centroid_ref = desired_destination_to_move
    centroid_error = centroid_ref - centroid
    print("Ref Centroid: ", centroid_ref)
    print("centroid error: ", centroid_error)
    position = tau + centroid_error
    
    # Should I change our position
    if iteration == 40:
        print("Iteration 40 --------------------- \n")
        desired_destination_to_move = np.array([
                            [-10, -10] # centroid should shift too
                            ])
    # if yes lets move the desired centroid position

    #plt.clf()
    plt.scatter(position[:, 0], position[:, 1], label='Agent Positions', color='red')
    plt.scatter(centroid[0], centroid[1], label='centroid', color='blue')
    plt.xlim(-100, 100)  # Adjust the x-axis limits if needed
    plt.ylim(-100, 100)  # Adjust the y-axis limits if needed
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Agent Positions (Iteration {})'.format(iteration))
    plt.grid(True)
    plt.pause(0.1)  
        


plt.show()