import os
import numpy as np
import random
import pandas as pd

# Directory containing the .txt files
directory = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/datas'

# Initialize lists to hold input and output data
X_list = []
y_list = []

# Check if directory exists
if not os.path.exists(directory):
    raise FileNotFoundError(f"Directory not found: {directory}")

# Get list of .txt files
file_list = [filename for filename in os.listdir(directory) if filename.endswith(".txt")]

# Shuffle the list to process files in random order
random.shuffle(file_list)

# Read and process each .txt file
for filename in file_list:
    filepath = os.path.join(directory, filename)
    print(f"Processing file: {filepath}")  # Debug statement
    with open(filepath, 'r') as file:
        lines = file.readlines()[1:]  # Skip the first line
        for line in lines:
            # Remove extra spaces and split by comma
            values = line.strip().split(',')
            
            # Remove extra spaces within each segment
            values = [v.strip() for v in values]
            
            # Flatten Pf and des_vel, since they are concatenated with spaces
            pf_des_vel_values = ' '.join(values[6:]).split()
            
            # Convert all values to float
            try:
                values = list(map(float, values[:6])) + list(map(float, pf_des_vel_values))
            except ValueError as e:
                print(f"Error processing line: {line}")
                raise e

            # Extract inputs and outputs
            inputs = values[:6]  # First 6 values are inputs
            pf_values = values[6:18]  # Next 12 values are the Pf values
            des_vel = values[18:21]  # Last 3 values are the des_vel
            
            # Combine Pf values and des_vel to form the outputs
            outputs = pf_values + des_vel
            
            X_list.append(inputs)
            y_list.append(outputs)

        # Add marker to indicate end of this run
        X_list.append([np.nan] * 6)
        y_list.append([np.nan] * 15)

# Convert lists to pandas DataFrames
X_df = pd.DataFrame(X_list, columns=[f"input_{i+1}" for i in range(6)])
y_df = pd.DataFrame(y_list, columns=[f"output_{i+1}" for i in range(15)])

# Save DataFrames to CSV files
X_df.to_csv('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/X_raw.csv', index=False)
y_df.to_csv('/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/y_raw.csv', index=False)

print("Data saved to CSV files successfully!")
