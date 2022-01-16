#%%
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

# find angle with horizontal axis from start to finish using atan2
angle = np.deg2rad(30)

# find the start point of path
start_vector = np.array([-10,-10])

# create random noise
x = np.linspace(0, 10, 10)
y =  np.zeros(len(x))
z = np.zeros(len(x))

# add noise
noise = np.random.uniform(low=-4, high=4, size=len(x))
y = y + noise
# first and last value of y should be 0
y[0], y[-1] = 0, 0

# create the signal
xy = np.vstack((x, y, z)).T

# find the quadrant
if angle > np.pi/2:
    angle = np.pi - angle
    quadrant = 2
elif angle < -np.pi/2:
    angle = np.pi + angle
    quadrant = 3
elif angle < 0:
    angle = -angle
    quadrant = 4
else:
    quadrant = 1

# create a rotation matrix
r = R.from_euler('xyz', [0, 0, angle], degrees=False)

# rotate the points
xy_rot = r.apply(xy)

# reflect the points
if quadrant == 2:
    # reflect vector about y-axis
    xy_ref = np.vstack((-xy_rot[:,0], xy_rot[:,1])).T

elif quadrant == 3:
    # reflect vector about x-axis and y-axis
    xy_ref = np.vstack((-xy_rot[:,0], -xy_rot[:,1])).T

elif quadrant == 4:

    # reflect vector about x-axis
    xy_ref = np.vstack((xy_rot[:,0], -xy_rot[:,1])).T

else:
    xy_ref = xy_rot[:,:2]

# translate the points with start vector
xy_trans = xy_ref + start_vector

# into a plot

# create the axis
x_axis= np.linspace(-15, 15, 10)
y_axis =  np.linspace(-15, 15, 10)

plt.plot(x_axis,0*y_axis, 'k--')
plt.plot(0*x_axis,y_axis, 'k--')

plt.plot(xy[:,0], xy[:,1], '-')
plt.plot(xy_trans[:,0], xy_trans[:,1], '--')
plt.xlim([-20, 20])
plt.ylim([-20, 20])

plt.show()
# %%
