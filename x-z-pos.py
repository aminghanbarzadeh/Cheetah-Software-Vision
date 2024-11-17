import pandas as pd
import matplotlib.pyplot as plt

# Load the logged data from text files
x_position_file = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/position_x.txt'
z_position_file = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/position_z.txt'

# Read data
x_position_data = pd.read_csv(x_position_file, header=None, names=['time', 'x_position'])
z_position_data = pd.read_csv(z_position_file, header=None, names=['time', 'z_position'])

# Merge the data based on time (assuming both files have matching time values)
merged_data = pd.merge(x_position_data, z_position_data, on='time')

# Plotting the x and z positions in one plot
plt.figure(figsize=(10, 6))
plt.plot(merged_data['time'], merged_data['x_position'], label='X Position')
plt.plot(merged_data['time'], merged_data['z_position'], label='Z Position')

plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid(True)
plt.show()
