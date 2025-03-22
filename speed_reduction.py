import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# UAV parameters
num_uavs = 5
speed = 2  # Default speed
min_separation = 5  # Minimum separation distance
area_size = 50  # Size of the area

def generate_waypoints(num_uavs, area_size):
    """Generate random start and end points for UAVs."""
    start_points = np.random.randint(0, area_size, (num_uavs, 2))
    end_points = np.random.randint(0, area_size, (num_uavs, 2))
    return start_points, end_points

start_points, end_points = generate_waypoints(num_uavs, area_size)
positions = np.copy(start_points).astype(float)
trajectories = [ [start_points[i].copy()] for i in range(num_uavs) ]

# Conflict resolution: Adjust speed dynamically
def resolve_conflicts(positions, speeds):
    for i in range(num_uavs):
        for j in range(i + 1, num_uavs):
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist < min_separation:
                speeds[i] *= 0.9  # Reduce speed
                speeds[j] *= 1.1  # Increase speed
    return speeds

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
    ax.set_title("UAV Conflict Resolution Simulation")

fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, frames=100, interval=200)
plt.show()