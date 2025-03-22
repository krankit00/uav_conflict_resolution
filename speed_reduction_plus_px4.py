import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import rospy
from pymavlink import mavutil
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

# PX4 setup
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Adjust for your PX4 setup
connection.wait_heartbeat()
print("Heartbeat received from PX4")

# ROS setup
rospy.init_node('uav_conflict_resolution', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
dummy_uav_position = np.array([0, 0])  # Placeholder for dummy UAV position

def dummy_uav_callback(msg):
    global dummy_uav_position
    dummy_uav_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    
rospy.Subscriber("/dummy_uav/pose", PoseWithCovarianceStamped, dummy_uav_callback)

# UAV parameters
num_uavs = 2  # Adjusted for PX4 integration
speed = 2  # Default speed
min_separation = 5  # Minimum separation distance
area_size = 50  # Size of the area

# Generate plus sign movement waypoints
def generate_plus_sign(area_size):
    step = area_size // 2
    start_points = np.array([
        [step, 0], [0, step]  # One UAV moving top to bottom, one left to right
    ])
    end_points = np.array([
        [step, area_size], [area_size, step]  # Top to bottom and left to right destinations
    ])
    return start_points, end_points

start_points, end_points = generate_plus_sign(area_size)
positions = np.copy(start_points).astype(float)
trajectories = [ [start_points[i].copy()] for i in range(num_uavs) ]

# PX4 commands for UAV 1
def set_mode(mode):
    mode_id = connection.mode_mapping()[mode]
    connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Mode set to {mode}")

def arm_vehicle():
    connection.arducopter_arm()
    print("UAV armed")

def takeoff(altitude=10):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print("Takeoff command sent")

# Conflict resolution using speed adjustments
def resolve_conflicts(positions, speeds):
    global dummy_uav_position
    dist = np.linalg.norm(positions[0] - dummy_uav_position)
    if dist < min_separation:
        print("Conflict detected! Slowing down UAV 1")
        speeds[0] *= 0.7  # Reduce speed of PX4 UAV
    return speeds

# PX4 UAV takes off and switches to offboard mode
arm_vehicle()
time.sleep(1)
takeoff()
time.sleep(5)
set_mode("OFFBOARD")

# Animation function
def update(frame):
    global positions
    speeds = np.full(num_uavs, speed)
    speeds = resolve_conflicts(positions, speeds)
    
    for i in range(num_uavs):
        direction = end_points[i] - positions[i]
        if np.linalg.norm(direction) > 1:
            positions[i] += (direction / np.linalg.norm(direction)) * speeds[i]
            trajectories[i].append(positions[i].copy())
    
    # Publish speed for UAV 1
    cmd = Twist()
    cmd.linear.x = speeds[0]
    cmd_vel_pub.publish(cmd)
    
    ax.clear()
    ax.set_xlim(0, area_size)
    ax.set_ylim(0, area_size)
    
    # Plot trajectories
    for i in range(num_uavs):
        traj = np.array(trajectories[i])
        ax.plot(traj[:, 0], traj[:, 1], linestyle='dotted', color='gray')
    
    ax.scatter(start_points[:, 0], start_points[:, 1], color='blue', label='Start')
    ax.scatter(end_points[:, 0], end_points[:, 1], color='red', label='End')
    ax.scatter(positions[:, 0], positions[:, 1], color='green', label='UAVs')
    ax.legend()
    ax.set_title("UAV Conflict Resolution Simulation - PX4 & ROS Integration")

fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, frames=100, interval=200)

# Save animation as GIF
ani.save("uav_conflict_resolution.gif", writer="pillow", fps=10)

plt.show()

rospy.spin()
