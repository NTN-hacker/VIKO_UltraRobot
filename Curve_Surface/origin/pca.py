import numpy as np
import matplotlib.pyplot as plt

#1. Generate random plane data

plate_X = np.arange(-1.0,1.0,0.1);
plate_Y = np.arange(-1.0,1.0,0.1);
plate_X, plate_Y = np.meshgrid(plate_X, plate_Y);
plate_Z = np.zeros_like(plate_X);
np_plate = np.stack([plate_X,plate_Y,plate_Z],axis=-1);
data = (np_plate + np.random.uniform(-0.1,0.1,np_plate.shape)).reshape((-1,3));


#Visualization of generated data
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[:,0],data[:,1],data[:,2]);
ax.set_xlim(-1.5,1.5);
ax.set_ylim(-1.5,1.5);
ax.set_zlim(-1.5,1.5);
plt.show()


#2. Calculating data averages
data_mean = data.mean(axis=0);
n = data.shape[0];
print("data:",data)
print("data mean:",data_mean)
print("data num:",n)
centroid_data = data - data_mean;

# centroid Data visualization
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[:,0],data[:,1],data[:,2],marker = 'o');
ax.scatter(centroid_data[:,0],centroid_data[:,1],centroid_data[:,2],marker = '^');
ax.set_xlim(-2.0,2.0);
ax.set_ylim(-2.0,2.0);
ax.set_zlim(-2.0,2.0);
plt.show()

#3. PCA 
X = centroid_data;
#Cov_X = (1.0/n)*np.dot(X.T,X);
Cov_X = (1.0/n)*np.dot(X.T,X);
U,S,V_T = np.linalg.svd(Cov_X);

print("Cov_X:",Cov_X)
print("U :",U)
print("S :",S)
print("V_T :",V_T)
print(np.dot(S.T,S))


"""
#4. Create a Plane
plate_X = np.arange(-1.0,1.0,0.1);
plate_Y = np.arange(-1.0,1.0,0.1);
plate_X, plate_Y = np.meshgrid(plate_X, plate_Y);
plate_Z = np.zeros_like(plate_X);
np_plate = np.stack([plate_X,plate_Y,plate_Z],axis=-1);

#PCA Planar visualization obtained through
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
np_plate = np.dot(V_T,np_plate.reshape((-1,3)).T).reshape(3,20,20).T;
ax.scatter(X[:,0],X[:,1],X[:,2],marker = '^');
ax.plot_surface(np_plate[:,:,0], np_plate[:,:,1], np_plate[:,:,2], alpha=0.2)
ax.set_xlim(-2.0,2.0);
ax.set_ylim(-2.0,2.0);
ax.set_zlim(-2.0,2.0);
plt.show()


#5.45 degree inclined plane
np_plate = np.stack([plate_X,plate_Y,plate_Z],axis=-1);
data = (np_plate + np.random.uniform(-0.1,0.1,np_plate.shape)).reshape((-1,3));

#Rotate 45 degrees around the plane x-axis
R_x = [ [1.0,0.0,0.0],
        [0.0,np.cos(np.deg2rad(45)),-np.sin(np.deg2rad(45))],
        [0.0,np.sin(np.deg2rad(45)),np.cos(np.deg2rad(45))],]
data = np.dot(R_x,data.T).T;

#PCA
data_mean = data.mean(axis=0);
X = data - data_mean;
Cov_X = (1.0/n)*np.dot(X.T,X);
U,S,V_T = np.linalg.svd(Cov_X);
np_plate = np.stack([plate_X,plate_Y,plate_Z],axis=-1);
np_plate = np.dot(V_T.T,np_plate.reshape((-1,3)).T).reshape(3,20,20).T;
fig = plt.figure()

ax = fig.add_subplot(projection='3d')
ax.scatter(X[:,0],X[:,1],X[:,2],marker = '^');
ax.plot_surface(np_plate[:,:,0], np_plate[:,:,1], np_plate[:,:,2], alpha=0.2)
ax.set_xlim(-2.0,2.0);
ax.set_ylim(-2.0,2.0);
ax.set_zlim(-2.0,2.0);
plt.show()


print("Cov_X:",Cov_X)
print("U :",U)
print("S :",S)
print("V_T :",V_T)
print(np.dot(S.T,S))

"""