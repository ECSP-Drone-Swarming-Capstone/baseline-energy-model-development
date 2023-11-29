import numpy as np
import matplotlib.pyplot as plt

# Define the Laplacian matrix for consensus-based control
laplacian = np.array([[2, -1, -1, 0],
                     [-1, 2, 0, -1],
                     [-1, 0, 2, -1],
                     [0, -1, -1, 2]])

# Define the desired formation (relative positions)
desired_formation = np.array([[0, 0],
                              [1, 0],
                              [0, 1],
                              [1, 1]])

# Define the trajectory to point B
trajectory_to_B = np.array([[3, 3],
                            [3.5, 3.5],
                            [4, 4]])

# Initialize agent positions and velocities
agent_positions = np.random.rand(4, 2) * 5
agent_velocities = np.zeros((4, 2))

# Simulation parameters
num_iterations = 100
time_step = 0.1

# Control gain for consensus-based control
k = 0.1

# Control gain for trajectory following
k_trajectory = 0.5

# Create a Matplotlib figure for visualization
plt.figure()

# Simulation loop
for iteration in range(num_iterations):
    # Calculate consensus control inputs based on Laplacian matrix
    consensus_inputs = -np.dot(laplacian, agent_positions)
    
    # Calculate trajectory-following control inputs
    trajectory_error = trajectory_to_B - agent_positions
    trajectory_inputs = k_trajectory * trajectory_error
    
    # Combine consensus and trajectory-following control inputs
    control_inputs = k * consensus_inputs + trajectory_inputs
    
    # Update agent velocities
    agent_velocities += control_inputs * time_step
    
    # Update agent positions
    agent_positions += agent_velocities * time_step
    
    # Plot current agent positions
    plt.clf()
    plt.scatter(agent_positions[:, 0], agent_positions[:, 1], label='Agent Positions')
    plt.scatter(trajectory_to_B[:, 0], trajectory_to_B[:, 1], c='red', marker='x', label='Point B')
    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Agent Positions (Iteration {})'.format(iteration))
    plt.legend()
    plt.grid(True)
    plt.pause(0.1)  # Pause to allow for visualization (adjust if needed)

# At the end of the simulation, 'agent_positions' will represent the converged positions.

# Keep the plot window open until the user closes it
plt.show()
