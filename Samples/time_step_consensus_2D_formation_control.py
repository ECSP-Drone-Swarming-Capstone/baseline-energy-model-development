import numpy as np
import time
import matplotlib.pyplot as plt
import geopy.distance
from scipy.integrate import solve_ivp

# Define the number of agents and their initial + desired positions
num_agents = 4  

initial_positions = np.array([
                              [1.0, 0.0], # Drone 1
                              [1.0, 5.0],
                              [3.0, 2.0],
                              [1.0, 3.0]
                            ])

desired_final_positions_K = np.array([  
                                      [0, 0], # Drone 1
                                      [2.0, 0],
                                      [0, 2.0],
                                      [2.0, 2.0]
                                    ])


# Define desired inter-agent distances (for simplicity, let's assume equal distances)
desired_distance = 2.0

# Define control gain parameter
k = 0.1

# Define simulation parameters
num_iterations = 100
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
    position = tau 
    
    plt.clf()
    plt.scatter(position[:, 0], position[:, 1], label='Agent Positions', color='red')
    plt.xlim(-10, 10)  # Adjust the x-axis limits if needed
    plt.ylim(-10, 10)  # Adjust the y-axis limits if needed
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Agent Positions (Iteration {})'.format(iteration))
    plt.grid(True)
    plt.pause(0.1)  
        
plt.show()