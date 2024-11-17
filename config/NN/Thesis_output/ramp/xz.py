import numpy as np
import matplotlib.pyplot as plt

# Load the logged data from the txt file
# Replace 'your_file_path.txt' with the actual path to your txt file
data = np.loadtxt('/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/ramp/Pf.txt', delimiter=',')

# Extract x and z positions (ignore time which is the first column)
x_positions = data[:, 1]
z_positions = data[:, 2]

# Generate the terrain line (real slope)
x_terrain = np.linspace(0, 3, 1000)  # X range from 0 to 3 for visualization
z_terrain = np.zeros_like(x_terrain)  # Initialize with zeros (flat)

# Define the ramp section
ramp_start_x = 0.558
ramp_end_x = 2.487385
ramp_slope = np.tan(np.radians(20))  # 20 degrees in radians

# Fill the terrain array with ramp slope
for i, x in enumerate(x_terrain):
    if x >= ramp_start_x and x <= ramp_end_x:
        z_terrain[i] = ramp_slope * (x - ramp_start_x)
    elif x > ramp_end_x:
        z_terrain[i] = ramp_slope * (ramp_end_x - ramp_start_x)  # Continue flat after ramp

# Plot the x-z positions of the foot and terrain
plt.figure(figsize=(10, 6))
plt.plot(x_positions, z_positions, label='Footstep Planner Output', color='red', linewidth=0.8)
plt.plot(x_terrain, z_terrain, 'k--', label='Terrain (Real Slope)', color='blue')

# Set labels and title
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.legend()

# Show plot
plt.grid(True)
plt.show()
