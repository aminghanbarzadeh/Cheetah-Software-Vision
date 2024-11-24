import pandas as pd
import matplotlib.pyplot as plt

# Load the file by splitting on commas first
file_path = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/f_ff.txt'  # Replace with the correct path to your file
force_data = pd.read_csv(file_path, header=None, delimiter=',')

# Split the forces for each leg (they are space-separated within each comma-separated group)
for col in range(1, 5):  # Columns 1 to 4 are for the forces
    force_data[[f'leg_{col}_x_force', f'leg_{col}_y_force', f'leg_{col}_z_force']] = force_data[col].str.split(expand=True)

# Drop the original combined columns (1 to 4)
force_data = force_data.drop(columns=[1, 2, 3, 4])

# Rename the first column to 'time'
force_data.columns = ['time'] + [f'leg_{leg}_{axis}_force' for leg in range(1, 5) for axis in ['x', 'y', 'z']]

# Convert the data to numeric
force_data = force_data.apply(pd.to_numeric)

# Plot Y forces for all legs
plt.figure(figsize=(10, 6))
plt.plot(force_data['time'], force_data['leg_1_y_force'], label='Leg 1', color='blue')
plt.plot(force_data['time'], force_data['leg_2_y_force'], label='Leg 2', color='orange')
plt.plot(force_data['time'], force_data['leg_3_y_force'], label='Leg 3', color='green')
plt.plot(force_data['time'], force_data['leg_4_y_force'], label='Leg 4', color='red')

plt.xlabel('Time')
plt.ylabel('y Force (N)')
plt.legend()
plt.tight_layout()
plt.show()
