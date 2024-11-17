import pandas as pd
import matplotlib.pyplot as plt

# Load the pitch angle data from the text file
pitch_file = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/Thesis_output/rpy_pitch.txt'

# Read data
pitch_data = pd.read_csv(pitch_file, header=None, names=['time', 'pitch_angle'])

# Apply the absolute value function to the pitch angle data
pitch_data['pitch_angle'] = pitch_data['pitch_angle'].abs()

# Plotting the absolute pitch angle over time
plt.figure(figsize=(10, 6))
plt.plot(pitch_data['time'], pitch_data['pitch_angle'], label='Pitch Angle')

plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (rad)')

plt.legend()
plt.grid(True)
plt.show()
