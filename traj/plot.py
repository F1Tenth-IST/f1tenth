import pandas as pd
import sys
import numpy as np
import yaml

import matplotlib.pyplot as plt

if len(sys.argv) != 2:
    print("Usage: python plot.py <path_to_csv>")
    csv_path = "traj/centerline_test_map.csv"
else:
    csv_path = sys.argv[1]

# Load the data
data = pd.read_csv(csv_path, header=None)
data.columns = ['s', 'x', 'y', 'kappa', 'n_l', 'n_r']

# Remove the first line of data
data = data.iloc[1:].reset_index(drop=True)

# Extract data
x = pd.to_numeric(data['x'], errors='coerce')
y = pd.to_numeric(data['y'], errors='coerce')
n_l = pd.to_numeric(data['n_l'], errors='coerce')
n_r = pd.to_numeric(data['n_r'], errors='coerce')
s = pd.to_numeric(data['s'], errors='coerce')
kappa = pd.to_numeric(data['kappa'], errors='coerce')

print(data.head())

# Compute psi (heading angle) using gradients
dx = np.gradient(x)
dy = np.gradient(y)
psi = np.arctan2(dy, dx)
    
# Compute boundaries using psi (heading angle)
left_boundary_x = x + n_l * (-np.sin(psi))
left_boundary_y = y + n_l * np.cos(psi)
right_boundary_x = x - n_r * (-np.sin(psi))
right_boundary_y = y - n_r * np.cos(psi)

# Plot the track
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Centerline', color='blue')
plt.plot(left_boundary_x, left_boundary_y, label='Left Boundary', color='green')
plt.plot(right_boundary_x, right_boundary_y, label='Right Boundary', color='red')

# Add direction arrows
for i in range(0, len(x) - 1, 10):  # Evitar acessar índice fora do intervalo
    plt.arrow(x[i], y[i], 0.1 * (x[i+1] - x[i]), 0.1 * (y[i+1] - y[i]),
              head_width=0.05, head_length=0.1, fc='blue', ec='blue')

# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Track Map with Centerline and Boundaries')
plt.legend()
plt.axis('equal')
plt.grid()

# Show the plot
plt.show()

plt.figure(figsize=(10, 4))
# Plot lines connecting centerline to boundaries
for i in range(len(x)):
    plt.plot([x[i], left_boundary_x[i]], [y[i], left_boundary_y[i]], color='green', linestyle='--', linewidth=0.5)
    plt.plot([x[i], right_boundary_x[i]], [y[i], right_boundary_y[i]], color='red', linestyle='--', linewidth=0.5)
    
    
import matplotlib.image as mpimg

# Load the .pgm map
map_path = "maps/test_map/map_output.pgm"  # Update with your actual path
map_image = mpimg.imread(map_path)

# Load the .yaml file to get map metadata
yaml_path = "maps/test_map/map_output.yaml"  # Update with your actual path
with open(yaml_path, 'r') as yaml_file:
    map_metadata = yaml.safe_load(yaml_file)

# Extract map metadata
resolution = map_metadata['resolution']  # meters per pixel
origin = map_metadata['origin']  # [x, y, theta]
width = map_image.shape[1]  # pixels
height = map_image.shape[0]  # pixels

# Define the map's extent
xmin = origin[0]
xmax = xmin + width * resolution
ymin = origin[1]
ymax = ymin + height * resolution
map_extent = [xmin, xmax, ymin, ymax]

# Flip the map vertically
map_image = np.flipud(map_image)

# Overlay the map on top of the scatter plot
plt.imshow(map_image, extent=map_extent, origin='lower', cmap='gray', alpha=0.5)

# Overlay the track data
plt.scatter(x, y, label='Centerline', color='blue', s=10)
plt.scatter(left_boundary_x, left_boundary_y, label='Left Boundary', color='green', s=10)
plt.scatter(right_boundary_x, right_boundary_y, label='Right Boundary', color='red', s=10)
    
plt.colorbar(label='s (m)')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Track Map Colored by s')
plt.axis('equal')
plt.show()


plt.figure(figsize=(10, 4))
plt.plot(s, n_l, label='Left Boundary Distance', color='green')
plt.plot(s, n_r, label='Right Boundary Distance', color='red')
plt.xlabel('s (m)')
plt.ylabel('Distance (m)')
plt.title('Boundary Distances vs. s')
plt.legend()
plt.grid()
plt.show()