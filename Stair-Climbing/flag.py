import pandas as pd

def add_end_of_run_flag(input_csv, output_csv, modified_input_csv, modified_output_csv):
    # Load your input and output data
    X_df = pd.read_csv(input_csv, header=0)
    y_df = pd.read_csv(output_csv, header=0)

    # Initialize the end_of_run_flag column with 0
    X_df['end_of_run_flag'] = 0
    y_df['end_of_run_flag'] = 0
    
    # Loop through the dataframe to detect rows where the first column is empty
    for i in range(1, len(X_df)):
        if pd.isna(X_df.iloc[i, 0]) or str(X_df.iloc[i, 0]).strip() == '':
            # If the first column is empty, flag the previous row
            X_df.at[i - 1, 'end_of_run_flag'] = 1
            y_df.at[i - 1, 'end_of_run_flag'] = 1

            # Mark the current row for removal by setting the first column to NaN
            X_df.iloc[i] = None
            y_df.iloc[i] = None

    # Remove the rows that were marked for removal (those with None values)
    X_df = X_df.dropna(how='all')
    y_df = y_df.dropna(how='all')

    # Save the modified dataframes back to CSV files
    X_df.to_csv(modified_input_csv, index=False)
    y_df.to_csv(modified_output_csv, index=False)

    print(f"Modified input saved to: {modified_input_csv}")
    print(f"Modified output saved to: {modified_output_csv}")

# Paths to your original and modified files
input_csv = '//home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/X_raw.csv'
output_csv = '/home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/y_raw.csv'
modified_input_csv = '//home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/x_modified.csv'
modified_output_csv = '//home/aminghanbarzadeh/Cheetah-Software-Vision/Stair-Climbing/y_modified.csv'

# Call the function to add the flag and save the modified CSV files
add_end_of_run_flag(input_csv, output_csv, modified_input_csv, modified_output_csv)
