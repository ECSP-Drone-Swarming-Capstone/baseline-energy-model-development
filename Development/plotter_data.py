import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = 'data_log_2023-12-06_08-36-38.csv'  # Replace with your file path
data = pd.read_csv(file_path)

# Extract the 'Power (Watts)' column
power = data['Power (Watts)']

# Create a plot of Power (Watts) versus column numbers without labels or title
plt.figure(figsize=(10, 6))
plt.grid(True)
plt.plot(power.index, power, marker='o', color='b')
plt.tick_params(labelsize=26)  # Set font size for the ticks
plt.show()