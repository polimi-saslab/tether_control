import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

csv_file = 'tether_force_and_attitude.csv'

time, fx, fy, fz, roll, pitch, yaw = [], [], [], [], [], [], []
with open(csv_file, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row['timestamp']))
        fx.append(float(row['fx']))
        fy.append(float(row['fy']))
        fz.append(float(row['fz']))
        roll.append(float(row['roll']))
        pitch.append(float(row['pitch']))
        yaw.append(float(row['yaw']))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Tether force subplot
ax1.plot(time, fx, label='Force X (E)', color='r')
ax1.plot(time, fy, label='Force Y (N)', color='g')
ax1.plot(time, fz, label='Force Z (U)', color='b')
ax1.set_ylabel('Force [N]')
ax1.set_title('Tether Force (ENU)')
ax1.grid(True)
ax1.legend()

# Attitude subplot
ax2.plot(time, roll, label='Roll (rad)', color='r')
ax2.plot(time, pitch, label='Pitch (rad)', color='g')
ax2.plot(time, yaw, label='Yaw (rad)', color='b')
ax2.set_ylabel('Angle [rad]')
ax2.set_xlabel('Time [s]')
ax2.set_title('Vehicle Attitude (RPY)')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()
