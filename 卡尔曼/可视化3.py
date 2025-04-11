import pandas as pd
import matplotlib.pyplot as plt

true_states = pd.read_csv("/home/lin/Desktop/卡尔曼波/data/true_states.csv")
measurements = pd.read_csv("/home/lin/Desktop/卡尔曼波/data/measurements.csv")
kalman_output = pd.read_csv("/home/lin/Desktop/卡尔曼波/results/kalman_output.csv")

min_len = min(len(true_states), len(measurements), len(kalman_output))
true_states = true_states.iloc[:min_len]
measurements = measurements.iloc[:min_len]
kalman_output = kalman_output.iloc[:min_len]

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

ax.plot(true_states["x"], true_states["y"], true_states["z"], label="True Path", color='green')
ax.plot(measurements["x"], measurements["y"], measurements["z"], label="Measured", linestyle='dotted', color='red')
ax.plot(kalman_output["x"], kalman_output["y"], kalman_output["z"], label="Kalman Filtered", color='blue')

ax.set_title("3D Drone Trajectory Tracking using Kalman Filter")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.tight_layout()
plt.show()
