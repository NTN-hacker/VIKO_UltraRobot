import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def curve_surface(theta, r):
    # x = theta
    # y = r * np.sin(theta)
    # z = np.zeros_like(x)

    theta_grid, r_grid = np.meshgrid(theta, r)
    x = theta_grid
    y = r_grid * np.sin(theta_grid)
    z = np.sin(theta_grid) * r_grid
    return np.stack([x, y, z], axis=-1)

# Generate points lying on the curve surface
theta_sample_num = 100
theta = np.linspace(0.0, 2 * np.pi, theta_sample_num)
r = np.linspace(0, 1, 10)
point_cloud = curve_surface(theta, r)

# Visualize the point cloud
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(point_cloud[:,0], point_cloud[:,1], point_cloud[:,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud on Curve Surface')
plt.show()