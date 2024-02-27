import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# Path to the directory where CSV files are stored
directory_path = '/home/nopparuj/FRA532EXAM_WS/odom_record'

# Finding the latest generated CSV file
list_of_files = glob.glob(os.path.join(directory_path, '*.csv'))
latest_file = max(list_of_files, key=os.path.getctime)

# Read the latest CSV file into a DataFrame
data = pd.read_csv(latest_file)

use_absolute = True

# Extracting x and y columns as NumPy arrays
odom_x = data['odom_x'].to_numpy()
odom_y = data['odom_y'].to_numpy()
integrate_x = data['integrated_x'].to_numpy()
integrate_y = data['integrated_y'].to_numpy()
ekf_x = data['ekf_x'].to_numpy()
ekf_y = data['ekf_y'].to_numpy()
try:
    absolute_x = data['x'].to_numpy()
    absolute_y = data['y'].to_numpy()
except:
    use_absolute = False

# Plotting x vs y
plt.figure(figsize=(8, 6))
plt.plot(odom_x, odom_y, marker='.', linestyle='-', color='b', label='Odom')
plt.plot(integrate_x, integrate_y, marker='.', linestyle='-', color='r', label='Integrated')
plt.plot(ekf_x, ekf_y, marker='.', linestyle='-', color='g', label='EKF')
if use_absolute:
    plt.plot(absolute_x, absolute_y, marker='.', linestyle='-', color='y', label='Absolute')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Plot of x vs y from the latest file')
plt.grid(True)
plt.legend()
plt.show()