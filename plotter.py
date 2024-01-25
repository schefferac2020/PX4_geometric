import matplotlib.pyplot as plt
import numpy as np
import os

import datetime

def create_directory_with_timestamp(base_path):
    # Get the current timestamp
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    # Construct the directory path
    directory_path = os.path.join(base_path, timestamp)
    # Create the directory
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' was created.")
    else:
        print(f"Directory '{directory_path}' already exists.")
    return directory_path

def parse_file(filename, line_starts_with):
    extracted_numbers = []
    with open(filename, 'r') as file:
        for line in file:
            if line.startswith(line_starts_with):
                parts = line.split()
                numbers = parts[-3:]  # Extract first three numbers
                try:
                    numbers = tuple(map(float, numbers))
                    extracted_numbers.append(numbers)
                except ValueError:
                    print(f"Warning: Could not convert '{numbers}' to numbers.")
    return np.array(extracted_numbers)

def plot_data(time, data, title, y_label, file_name):
    plt.figure(figsize=(10, 6))
    for i, label in enumerate(['X', 'Y', 'Z']):
        plt.plot(time, data[:, i], label=f'{label} {y_label}')
    plt.xlabel('Time')
    plt.ylabel(y_label)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.savefig(file_name)

# Create a dummy file with sample data (For demonstration)
dummy_filename = 'out.txt'
# Note: Actual data needs to be in the file for real usage

# Parse the data from the file
position_err = parse_file(dummy_filename, "position_err")
angle_err = parse_file(dummy_filename, "angle_err")
desired_angular_accel = parse_file(dummy_filename, "[GEOMETRIC CONTROLLER]: desired angular accel:")
curr_angle = parse_file(dummy_filename, "curr_angle:")
desired_angle = parse_file(dummy_filename, "desired_angle:")
angular_rate_error = parse_file(dummy_filename, "angular_rate_error:")
PID_values = parse_file(dummy_filename, "[PID CONTROLLER]: desired angular accel: ")
GEO_values = parse_file(dummy_filename, "[GEO CONTROLLER]: desired angular accel: ")




val = min(len(position_err), len(angle_err), len(desired_angular_accel), len(desired_angle), len(curr_angle), len(GEO_values), len(angular_rate_error), len(PID_values))



# Plotting
time = np.arange(0, val)

time = time[:val]
position_err = position_err[:val]
angle_err = angle_err[:val]
desired_angular_accel = desired_angular_accel[:val]
curr_angle = curr_angle[:val]
desired_angle = desired_angle[:val]
angular_rate_error = angular_rate_error[:val]
PID_values = PID_values[:val]
GEO_values = GEO_values[:val]

test_name = create_directory_with_timestamp(".")
if not os.path.exists(test_name):
    os.makedirs(test_name)

plot_data(time, position_err, 'Position Error', 'Position Error', f'{test_name}/out_pos_err.png')
plot_data(time, angle_err, 'Angle Error', 'Angle Error', f'{test_name}/out_ang_err.png')
plot_data(time, desired_angular_accel, 'Desired Angular Acceleration', 'Angular Acceleration', f'{test_name}/out_ang_accel.png')
plot_data(time, curr_angle, 'Current Angle', 'Current Angle', f'{test_name}/out_ang.png')
plot_data(time, desired_angle, 'desired_angle', 'desired_angle', f'{test_name}/out_des_ang.png')
plot_data(time, angular_rate_error, 'angular_rate_error', 'angular_rate_error', f'{test_name}/angular_rate_error')
plot_data(time, PID_values, 'PID_values', 'PID_values', f'{test_name}/PID_values')
plot_data(time, GEO_values, 'GEO_values', 'GEO_values', f'{test_name}/GEO_values')
plt.show()