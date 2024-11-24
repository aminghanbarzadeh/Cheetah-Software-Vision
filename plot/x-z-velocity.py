import pandas as pd
import matplotlib.pyplot as plt

# Load the logged velocity data from text files
x_velocity_file = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/v_des_x.txt'
z_velocity_file = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/v_des_z.txt'

# Read data
x_velocity_data = pd.read_csv(x_velocity_file, header=None, names=['time', 'x_velocity'])
z_velocity_data = pd.read_csv(z_velocity_file, header=None, names=['time', 'z_velocity'])

# Merge the data based on time (assuming both files have matching time values)
merged_velocity_data = pd.merge(x_velocity_data, z_velocity_data, on='time')

# Plotting the x and z velocities in one plot
plt.figure(figsize=(10, 6))
plt.plot(merged_velocity_data['time'], merged_velocity_data['x_velocity'], label='X Velocity')
plt.plot(merged_velocity_data['time'], merged_velocity_data['z_velocity'], label='Z Velocity')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()
