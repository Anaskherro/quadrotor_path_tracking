import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def closest_point_on_path(x_t, y_t, path_x, path_y):
    """
    Calculate the closest point on the reference path to the current point (x_t, y_t).
    """
    distances = np.sqrt((path_x - x_t)**2 + (path_y - y_t)**2)
    min_index = np.argmin(distances)
    return path_x[min_index], path_y[min_index]

# Load recorded path
recorded_path = np.loadtxt("record_x500.txt", delimiter=',')
recorded_x = recorded_path[:, 0]
recorded_y = recorded_path[:, 1]

# Load reference path
reference_path = np.loadtxt("path2.txt", delimiter=',')
reference_x = reference_path[:, 0]
reference_y = reference_path[:, 1]

# Create a common time base for interpolation
recorded_time = np.linspace(0, 1, len(recorded_x))
reference_time = np.linspace(0, 1, len(reference_x))
common_time = np.linspace(0, 1, max(len(recorded_x), len(reference_x)))

# Interpolate both paths to the common time base
interp_recorded_x = interp1d(recorded_time, recorded_x, kind='linear')
interp_recorded_y = interp1d(recorded_time, recorded_y, kind='linear')
interp_reference_x = interp1d(reference_time, reference_x, kind='linear')
interp_reference_y = interp1d(reference_time, reference_y, kind='linear')

common_recorded_x = interp_recorded_x(common_time)
common_recorded_y = interp_recorded_y(common_time)
common_reference_x = interp_reference_x(common_time)
common_reference_y = interp_reference_y(common_time)
# Calculate Path Tracking Error
errors = []
for x_t, y_t in zip(common_recorded_x, common_recorded_y):
    x_p, y_p = closest_point_on_path(x_t, y_t, common_reference_y, common_reference_x)
    error = np.sqrt((x_t - x_p)**2 + (y_t - y_p)**2)
    errors.append(error)

path_tracking_error = np.mean(errors)
print("Path Tracking Error:", path_tracking_error)

# Plot recorded path and reference path
plt.figure(figsize=(8, 6))
plt.plot(common_recorded_x, common_recorded_y, label="Trajectoire actuelle")
plt.plot(common_reference_y, common_reference_x, label="Trajectoire de référence")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Suivi de trajectoire par MPC - "+"Erreur : " + str(path_tracking_error))
plt.legend()
plt.grid(True)


plt.show()

