import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
waypoints_path = './maps/rad_perfect_smoothed.csv'


# CSVファイルの読み込み
waypoints_df = pd.read_csv(waypoints_path, comment='#', header=None, names=['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m', 'curvature', 'heading_angle_rad'])

# Extract x and y coordinates
x = waypoints_df['x_m'].values
y = waypoints_df['y_m'].values
curvature = waypoints_df['curvature'].values
heading_angle = np.radians(waypoints_df['heading_angle_rad'].values)

# PPPMを用いた新しい経路生成

# 新しい経路X,Y
X_ppm = [x[0]]
Y_ppm = [y[0]]

velocity = 1.0  # 速度 [m/s]
looking_time = 2.0  # ルックアヘッド時間 [s]
lookahead_distance =  int(velocity * looking_time) # ルックアヘッド距離
gain = 2.0  # PPMのゲイン


for i in range(1, len(x)):
    # PPMの計算

    # waypointの間隔
    length = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
    
    X_ppm.append(x[i] + length * np.sin(heading_angle[i]) * gain * (curvature[min(i -1 + lookahead_distance, len(curvature) - 1)] - curvature[i-1]) * velocity)
    Y_ppm.append(y[i] - length * np.cos(heading_angle[i]) * gain * (curvature[min(i -1 + lookahead_distance, len(curvature) - 1)] - curvature[i-1]) * velocity)


    # min(current_idx + lookahead_distance, len(curvature) - 1) は、配列のインデックスが範囲外に出ないようにするためのもの


# 曲線の1次および2次時間微分
dx = np.gradient(X_ppm)
dy = np.gradient(Y_ppm)
ddx = np.gradient(dx)
ddy = np.gradient(dy)
curvature2 = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)



# Define column names
column_names = ['x', 'y', 'z']
# Load the provided CSV files
outer_track_path = './maps/left_lane_bound.csv'
inner_track_path = './maps/right_lane_bound.csv'
center_track_path = './maps/center_lane_line.csv'
# Re-load the CSV files with column names
outer_track_df = pd.read_csv(outer_track_path, names=column_names, header=0)
inner_track_df = pd.read_csv(inner_track_path, names=column_names, header=0)
center_track_df = pd.read_csv(center_track_path, names=column_names, header=0)



# Plot the original and PPM paths
plt.figure(figsize=(12, 6))
plt.plot(x, y, label='Original Path', color='blue')
plt.plot(X_ppm, Y_ppm, label='PPM Path', color='red')
plt.plot(outer_track_df['x'], outer_track_df['y'], 'black') #label='Left Lane Bound'
plt.plot(inner_track_df['x'], inner_track_df['y'], 'black') #label='Right Lane Bound'
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Original and PPPM Paths')
plt.legend()
plt.grid(True)
plt.savefig('pppm_path.png')
plt.show()


# Plot curvature
plt.figure(figsize=(12, 6))
plt.subplot(1, 1, 1)
plt.plot(curvature, label='Curvature_Original')
plt.plot(curvature2, label='Curvature_PPM')
plt.xlabel('Waypoint Index')
plt.ylabel('Curvature (1/m)')
plt.title('Curvature along the Path')
plt.grid(True)
plt.legend()
plt.savefig('pppm_curvature.png')
plt.show()



