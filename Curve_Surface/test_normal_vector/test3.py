import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def curve_surface(theta, r):
    x = np.outer(theta, np.ones_like(r))
    y = np.zeros_like(x)
    z = np.outer(r, np.sin(theta))
    return x, y, z


# Generate curves lying on the surface
theta_sample_num = 100
r_sample_num = 10
theta = np.linspace(0.0, 2 * np.pi, theta_sample_num)
r = np.linspace(0.1, 1.0, r_sample_num)
x, y, z = curve_surface(theta, r)



fig = plt.figure()
ax = Axes3D(fig)
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
# Visualize the surface
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x, y, z, rstride=5, cstride=5, cmap='viridis')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Surface from Multiple Curves')
# plt.show()