import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

# find angle with horizontal axis from start to finish using atan2
angle = np.deg2rad(179)

# create random noise
x = np.random.uniform(0, 1, 100)
x = np.linspace(0, 10, 10)
y =  np.zeros(len(x))
z = np.zeros(len(x))
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
print(np.rad2deg(angle))
print(quadrant)
# rotate the points
xy_rot = r.apply(xy)

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


# into a plot

# create the axis
x_axis= np.linspace(-15, 15, 10)
y_axis =  np.linspace(-15, 15, 10)

plt.plot(x_axis,0*y_axis, 'k--')
plt.plot(0*x_axis,y_axis, 'k--')

plt.plot(xy[:,0], xy[:,1], '-')
plt.plot(xy_ref[:,0], xy_ref[:,1], '--')
plt.xlim([-15, 15])
plt.ylim([-15, 15])

plt.show()