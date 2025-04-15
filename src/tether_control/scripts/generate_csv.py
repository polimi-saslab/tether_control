import rosbag2_py
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R


csv_file = 'tether_force_and_attitude.csv'
date = 

# Open reader
storage_options = rosbag2_py.StorageOptions(
    uri='/home/yannis/tether_control_ws/install/tether_control/share/tether_control' + 'topic_results_{date}',
    storage_id='sqlite3'
)
converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Build topic type map manually
topic_type_map = {}
for topic_info in reader.get_all_topics_and_types():
    topic_type_map[topic_info.name] = topic_info.type

# Map ROS types to Python message classes
type_map = {
    'px4_msgs/msg/VehicleAttitude': VehicleAttitude,
    'geometry_msgs/msg/WrenchStamped': WrenchStamped,
    'std_msgs/msg/Float32MultiArray': Float32MultiArray,
}

data = {
    '/fmu/out/vehicle_attitude': [],
    '/drone/tether_force': [],
    '/metrics/model' : [],
}

# Extract both topics into separate lists
forces = []
attitudes = []

while reader.has_next():
    topic, msg_bytes, timestamp = reader.read_next()
    timestamp = timestamp * 1e-9  # ns → s

    ros_type_str = topic_type_map[topic]
    if ros_type_str not in type_map:
        continue

    msg = deserialize_message(msg_bytes, type_map[ros_type_str])

    if topic == '/drone/tether_force':
        forces.append((timestamp, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z))
    elif topic == '/fmu/out/vehicle_attitude':
        q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]  # (x, y, z, w)
        r = R.from_quat(q)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        attitudes.append((timestamp, roll, pitch, yaw))

# --- Interpolate attitude to match force timestamps ---
att_times, att_r, att_p, att_y = zip(*attitudes)
force_times = [t[0] for t in forces]

att_r_interp = np.interp(force_times, att_times, att_r)
att_p_interp = np.interp(force_times, att_times, att_p)
att_y_interp = np.interp(force_times, att_times, att_y)

# --- Filter by time window ---
start_time = 1744623891.582138  # <-- set your start time in seconds
end_time = 1744623935.0523984    # <-- set your end time in seconds

filtered_rows = []
for i, t in enumerate(force_times):
    if start_time <= t <= end_time:
        fx, fy, fz = forces[i][1], forces[i][2], forces[i][3]
        filtered_rows.append([
            t, fx, fy, fz,
            att_r_interp[i],
            att_p_interp[i],
            att_y_interp[i]
        ])

# --- Save to CSV ---
with open(csv_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'fx', 'fy', 'fz', 'roll', 'pitch', 'yaw'])
    writer.writerows(filtered_rows)

print(f"Saved filtered data to: {csv_file}")
