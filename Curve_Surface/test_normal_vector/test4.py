import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def curve_surface(theta, r):
    theta_grid, r_grid = np.meshgrid(theta, r)
    x = theta_grid
    y = np.linspace(-1, 1, len(r_grid))[:, None]  # Tạo một trục y dày hơn
    z = np.sin(theta_grid) * r_grid
    return x, y, z

# Định nghĩa các giá trị cho theta và r
theta = np.linspace(0, 2*np.pi, 100)
r = np.linspace(0, 1, 10)

# Tạo mặt cong từ các điểm được xác định bởi theta và r
x, y, z = curve_surface(theta, r)

# Hiển thị mặt cong
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x, y, z, rstride=5, cstride=5, cmap='viridis')

# ax.scatter(x.ravel(), y.ravel(), z.ravel(), color='b')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Curve Surface')
plt.show()

