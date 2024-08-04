import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

def search_nearest_index(x, y, bound_map_x, bound_map_y, index=0, limit=10):
    min_dist = 1000000
    min_index = index
    bound_map_x = bound_map_x + bound_map_x
    bound_map_y = bound_map_y + bound_map_y
    for i in range(index - limit, index + limit):
        dist = np.sqrt((x - bound_map_x[i])**2 + (y - bound_map_y[i])**2)
        if dist < min_dist:
            min_dist = dist
            min_index = i
    return min_index

def calc_width(x, y, bound_x, bound_y):
    return np.sqrt((x - bound_x)**2 + (y - bound_y)**2)

def calculate_center_line(outer_track_df, inner_track_df, limit=10, skip=5):
    outer_track_x = list(outer_track_df['x'])
    outer_track_y = list(outer_track_df['y'])
    inner_track_x = list(inner_track_df['x'])
    inner_track_y = list(inner_track_df['y'])
    center_x = []
    center_y = []
    w_tr_right_m = []
    w_tr_left_m = []
    nearest_index = 0
    for i in range(0, len(outer_track_x), skip):
        nearest_index = search_nearest_index(outer_track_x[i], outer_track_y[i], inner_track_x, inner_track_y, nearest_index, limit)

        if nearest_index > len(inner_track_x) - 1:
            nearest_index -= len(inner_track_x)
        center_x.append((outer_track_x[i] + inner_track_x[nearest_index]) / 2)
        center_y.append((outer_track_y[i] + inner_track_y[nearest_index]) / 2)
        width = calc_width(outer_track_x[i], outer_track_y[i], inner_track_x[nearest_index], inner_track_y[nearest_index])
        w_tr_right_m.append((width / 2) - 0.3)
        w_tr_left_m.append((width / 2) - 0.3)
    
    return pd.DataFrame({
        'x_m': center_x,
        'y_m': center_y,
        'w_tr_right_m': w_tr_right_m,
        'w_tr_left_m': w_tr_left_m
    })

def smooth_center_line(center_df, smoothing_factor=0.1):
    tck, u = splprep([center_df['x_m'], center_df['y_m']], s=smoothing_factor)
    unew = np.linspace(0, 1.0, len(center_df['x_m']))
    out = splev(unew, tck)
    return pd.DataFrame({
        'x_m': out[0],
        'y_m': out[1],
        'w_tr_right_m': center_df['w_tr_right_m'],
        'w_tr_left_m': center_df['w_tr_left_m']
    })

# Load the provided CSV files
outer_track_path = 'mademap/left_lane_bound.csv'
inner_track_path = 'mademap/right_lane_bound.csv'
center_track_path = 'mademap/center_lane_line.csv'

# Define column names
column_names = ['x', 'y', 'z']

# Re-load the CSV files with column names
outer_track_df = pd.read_csv(outer_track_path, names=column_names, header=0)
inner_track_df = pd.read_csv(inner_track_path, names=column_names, header=0)
center_track_df = pd.read_csv(center_track_path, names=column_names, header=0)

# Calculate the center line
center_df = calculate_center_line(outer_track_df, inner_track_df, limit=40, skip=3)

# Smooth the center line
smoothed_center_df = smooth_center_line(center_df, smoothing_factor=1.0)

# Define the output path
output_corrected_path = 'ai_challenge_smoothed.csv'

# Define the custom header string
custom_header = '# x_m, y_m, w_tr_right_m, w_tr_left_m\n'

# Open the file in write mode and write the custom header
with open(output_corrected_path, 'w') as f:
    f.write(custom_header)
    smoothed_center_df.to_csv(f, index=False, header=False)



# Plot the original and smoothed center lines
plt.figure(figsize=(10, 6))
plt.plot(outer_track_df['x'], outer_track_df['y'], 'black', label='Left Lane Bound')
plt.plot(inner_track_df['x'], inner_track_df['y'], 'black', label='Right Lane Bound')
#plt.plot(center_track_df['x'], center_track_df['y'], 'c--', label='Original Center Lane Line')
plt.plot(center_df['x_m'], center_df['y_m'], 'coral', label='Calculated Center Line')
plt.plot(smoothed_center_df['x_m'], smoothed_center_df['y_m'], 'c', label='Smoothed Center Line')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Track Visualization with Smoothed Center Line')
plt.legend()
plt.grid(True)
plt.savefig('track_smoothed.png')
plt.show()
