import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
waypoints_path = './maps/ai_challenge_smoothed.csv'

# CSVファイルの読み込み
waypoints_df = pd.read_csv(waypoints_path, comment='#', header=None, names=['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])

# Extract x and y coordinates
x = waypoints_df['x_m'].values
y = waypoints_df['y_m'].values
d = waypoints_df['w_tr_right_m'].values # 中心経路と右レーンの幅(右レーンの幅は左レーンの幅と同じと仮定)

# 曲線の1次および2次時間微分
dx = np.gradient(x)
dy = np.gradient(y)
ddx = np.gradient(dx)
ddy = np.gradient(dy)


# Calculate curvature
curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)




# theta^2 = 2d * np.abs(curvature)
# theta = np.sqrt(2 * d * np.abs(curvature))



# Calculate heading angle
# heading_angle = np.arctan2(dy, dx)



# 各ウェイポイント間の方位角を計算
heading_angles = np.arctan2(dy, dx)

# # 方位角の変化量を計算
# heading_angle_changes = np.diff(heading_angles)

# # 角度の変化がπを超えた場合の補正
# heading_angle_changes = (heading_angle_changes + np.pi) % (2 * np.pi) - np.pi

# # 最後の値を追加して同じ長さにする
# heading_angle_changes = np.append(heading_angle_changes, heading_angle_changes[-1])

# # ラジアンから度数に変換
# heading_angles_deg = np.degrees(heading_angle_changes)


# ラジアンから度数に変換
heading_angles_deg = np.degrees(heading_angles)



# 曲率と方位角の情報をデータフレームに追加
waypoints_df['curvature'] = curvature
waypoints_df['heading_angle_rad'] = heading_angles

# 新しいCSVファイルとして保存（適切なヘッダーを付加）
new_waypoints_path = './maps/rad_perfect_smoothed.csv'


# Define the custom header string
custom_header = '# x_m, y_m, w_tr_right_m, w_tr_left_m, curvature, heading_angle_rad\n'

# Open the file in write mode and write the custom header
with open(new_waypoints_path, 'w') as f:
    f.write(custom_header)
    waypoints_df.to_csv(f, index=False, header=False)




# # Plot dx, dy, ddx, ddy

# plt.figure(figsize=(12, 6))
# plt.subplot(2, 2, 1)
# plt.plot(dx, label='dx')
# plt.xlabel('Waypoint Index')
# plt.ylabel('dx')
# plt.title('dx along the Path')
# plt.grid(True)
# plt.legend()

# plt.subplot(2, 2, 2)
# plt.plot(dy, label='dy')
# plt.xlabel('Waypoint Index')
# plt.ylabel('dy')
# plt.title('dy along the Path')
# plt.grid(True)
# plt.legend()

# plt.subplot(2, 2, 3)
# plt.plot(ddx, label='ddx')
# plt.xlabel('Waypoint Index')
# plt.ylabel('ddx')
# plt.title('ddx along the Path')
# plt.grid(True)
# plt.legend()

# plt.subplot(2, 2, 4)
# plt.plot(ddy, label='ddy')
# plt.xlabel('Waypoint Index')
# plt.ylabel('ddy')
# plt.title('ddy along the Path')
# plt.grid(True)
# plt.legend()    

# plt.tight_layout()
# plt.savefig('./230802/rad_dx_dy_ddx_ddy.png')
# plt.show()


# dx が0の位置に対応する z の値を決定
z = np.where(dx == 0, np.pi / 2 * np.sign(dy), dy / dx)

# dxが0でdyが負の部分を修正
z = np.where((dx == 0) & (dy < 0), -np.pi / 2, z)


# if(dx == 0):
#     if(dy >= 0):
#         z = np.pi / 2
#     else:
#         z = -np.pi / 2
# else:
#     z = dy / dx


# plot dy/dx, arctan2(dy, dx)
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(z, label='dy/dx')
plt.xlabel('Waypoint Index')
plt.ylabel('dy/dx')
plt.title('z along the Path')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(heading_angles, label='heading_angles')
plt.xlabel('Waypoint Index')
plt.ylabel('heading_angles')
plt.title('arctan2(dy,dx) along the Path')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig('./230802/rad_z_heading_angles.png')
plt.show()




# # Plot curvature
# plt.figure(figsize=(12, 6))
# plt.subplot(2, 1, 1)
# plt.plot(curvature, label='Curvature')
# plt.xlabel('Waypoint Index')
# plt.ylabel('Curvature (1/m)')
# plt.title('Curvature along the Path')
# plt.grid(True)
# plt.legend()

# # Plot heading angle
# plt.subplot(2, 1, 2)
# plt.plot(heading_angles_deg, label='Heading Angle')
# plt.xlabel('Waypoint Index')
# plt.ylabel('Heading Angle (degrees)')
# plt.title('Heading Angle along the Path')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.savefig('rad_track_information.png')
# plt.show()


