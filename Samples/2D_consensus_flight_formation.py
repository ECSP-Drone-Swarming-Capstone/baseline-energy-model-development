from tkinter import NE
import numpy as np
import networkx as nx
import time
import matplotlib.pyplot as plt
import geopy.distance
from scipy.integrate import solve_ivp

# Define the number of agents and their initial + desired positions
num_agents = 4  

initial_positions = np.array([
                              1.0, # X
                              1.0, 
                              3.0,
                              1.0, 
                              0.0, # Y
                              5.0,
                              2.0,
                              3.0
                            ])

desired_final_positions_K = np.array([  
                                      0, # X 
                                      2.0, 
                                      0,
                                      2.0,
                                      0, # Y
                                      0,
                                      2.0,
                                      2.0
                                    ])

# Define simulation parameters
num_iterations = 40
time_step = 0.1
    
# Updated Laplacian for Square Formation 

laplacian = np.array([
                [2, -1, -1, 0],
                [-1, 2, 0, -1],
                [-1, 0, 2, -1],
                [0, -1, -1, 2]
            ]) 

# 4 x 4 Zero matrix for A Matrix
zero = np.zeros((4,4)) 

# Generates A metrix used for the 8 x 1 positions
A = np.block([[-1*laplacian, zero],
              [zero, -1*laplacian]])

print("A matrix:", A)

# Create a Matplotlib figure for visualization
plt.figure()

# Create the ODE we want to solve for in order to get the trajectory of the desired formation
tspan = (0, 10)

def tau_ode(t, x): 
    displacement =  x - desired_final_positions_K 
    tau_dot = np.dot(A, displacement)
    return tau_dot

# Solves ODE
solution = solve_ivp(tau_ode, tspan, initial_positions, t_eval = np.linspace(0, 10, 100), method='RK45')

t = solution.t
X_solution = solution.y
print("results:", X_solution)
# Simulation loop
'''for iteration in range(num_iterations):
    # Calculate Tau from: X (current position) = X (Final Position) - Tau (Translation)
    displacement = desired_final_positions_K - initial_positions
    # print("Verify Displacement Calc for Tau:", displacement)
    # break

    # Implement Consensus for Tau_dot: Tau_dot(t) = -L (Laplacian) * Tau(t)
    # as T -> inf we should see Tau get smaller and become zero when this happens Tau_dot will be zero
    tau_dot = -np.dot(laplacian, displacement)
    print("Verify Tau_dot:", tau_dot)
    
    initial_positions = tau_dot * time_step
    print("Verify new position:", initial_positions)
    
    # Calculate the consensus control inputs based on the Laplacian matrix
    consensus_inputs = -np.dot(laplacian, (initial_positions - desired_positions))
    print("Consensus Input: ", consensus_inputs)
    # Compute control inputs for each agent based on consensus algorithm
    control_inputs = k * (consensus_inputs - initial_positions)
    
    # Calculate the pairwise distances between agents
    distances = np.linalg.norm(initial_positions[:, np.newaxis, :] - initial_positions, axis=2)
    print("distance: ", distances)
    # Update agent velocities
    velocities += np.dot((desired_distance - distances), control_inputs) # / distances
    #velocities += control_inputs
    # Update agent positions
    initial_positions += velocities * time_step
    '''
# Plot current agent positions
for index in range(0, 100):
    initial_positions = np.array([
                                [X_solution[0][index], X_solution[4][index]],
                                [X_solution[1][index], X_solution[5][index]],
                                [X_solution[2][index], X_solution[6][index]],
                                [X_solution[3][index], X_solution[7][index]]
                            ])
    plt.scatter(initial_positions[:, 0], initial_positions[:, 1], label='Agent Positions', color='red')
    plt.xlim(-5, 5)  # Adjust the x-axis limits if needed
    plt.ylim(-5, 5)  # Adjust the y-axis limits if needed
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Agent Positions (Iteration {})'.format(index))
    plt.grid(True)
    plt.pause(0.1)  # Pause to allow for visualization (adjust if needed)

# At the end of the simulation, 'initial_positions' will represent the converged formation.

# Keep the plot window open until the user closes it
plt.show()
