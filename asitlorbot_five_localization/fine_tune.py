# from pyDOE2 import ff2n
# import numpy as np
# import pandas as pd

# # Define factor names and levels
# factor_names = ["P1", "P2", "P3", "P4", "P5"]
# num_factors = len(factor_names)
# level_values = [0.001, 0.01, 0.1, 1, 10, 100, 1000]

# # Generate a half fraction of the full factorial design
# fractional_design = ff2n(num_factors * 2)

# # Map the levels explicitly
# scaled_design = np.tile(level_values, (fractional_design.shape[0], 1)) * (fractional_design + 1) / 2

# # Create a DataFrame with factor names
# design_df = pd.DataFrame(scaled_design, columns=factor_names)

# # Print the resulting fractional design with factor names and specified levels
# print("Fractional Design:")
# print(design_df)


######################################################################################
# import pandas as pd
# from ruamel.yaml import YAML

# file_path = "/home/asimkumar/asitlor_ws/src/asitlorbot_five_localization/config/ekf.yaml"
# xlsx_file_path = "/home/asimkumar/Documents/data.xlsx"

# # Load the YAML file with ruamel.yaml
# yaml = YAML()
# with open(file_path, 'r') as file:
#     data = yaml.load(file)

# row_index = 0  # Start with the first row

# while True:
#     # Load data from Excel file
#     df = pd.read_excel(xlsx_file_path)

#     # Get the values from the specified row of the Excel file
#     new_values = df.iloc[row_index, :].tolist()

#     # Update the specific elements in the process_noise_covariance matrix
#     indices_to_update = [80, 96, 112, 176, 192]

#     for index, value in zip(indices_to_update, new_values):
#         data['ekf_filter_node']['ros__parameters']['process_noise_covariance'][index] = value

#     # Save the updated YAML file while preserving the original formatting
#     with open(file_path, 'w') as file:
#         yaml.dump(data, file)

#     print("Process noise covariance matrix updated successfully for row", row_index + 1)

#     move_to_next_row = input("Do you want to move to the next row? (y/n): ")

#     if move_to_next_row.lower() != 'y':
#         break

#     row_index += 1


#########
# CALL the Process noise covariance
import yaml
import numpy as np

def extract_process_noise_covariance(file_path):
    try:
        # Load the YAML file
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        # Print the entire loaded YAML data for debugging
        # print(yaml_data)

        # Extract the process noise covariance from the loaded YAML data
        process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

        if process_noise_covariance is not None:
            # Convert the list elements to float
            process_noise_covariance = [float(val) for val in process_noise_covariance]

            return np.array(process_noise_covariance)
        else:
            print('Error: Missing or invalid process_noise_covariance data in ekf.yaml.')
            return None
    except Exception as e:
        print(f'Error: {str(e)}')
        return None


# Specify the full file path to ekf.yaml
file_path = '/home/asimkumar/asitlor_ws/src/asitlorbot_five_localization/config/ekf.yaml'

# Extract process noise covariance
process_noise_covariance = extract_process_noise_covariance(file_path)

if process_noise_covariance is not None:
    # print('Process Noise Covariance:')
    print(process_noise_covariance)




