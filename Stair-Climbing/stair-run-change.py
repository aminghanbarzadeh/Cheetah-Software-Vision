import os

def update_fourth_column(folder_path):
    # Loop through all txt files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)
            
            # Open the file and read the lines
            with open(file_path, 'r') as file:
                lines = file.readlines()
                
            # Open the file again in write mode to update it
            with open(file_path, 'w') as file:
                # Write the header (first line) back without changes
                file.write(lines[0])
                
                # Process each remaining line (data lines)
                for line in lines[1:]:
                    columns = line.strip().split(',')
                    
                    # Check if the fourth column is zero and update it to 0.2
                    if columns[3] == '5':  # Check for zero in the fourth column
                        columns[3] = '0.2'
                    
                    # Write the updated line back
                    file.write(','.join(columns) + '\n')
            
            print(f"Updated: {filename}")

# Set the folder path where the txt files are stored
folder_path = '/home/aminghanbarzadeh/Cheetah-Software-Vision/config/NN/datas'

# Call the function to update the files
update_fourth_column(folder_path)
