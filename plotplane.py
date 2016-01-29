#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import math
import time
from itertools import product, combinations
# for the arrow
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

## arrow function
class Arrow3D(FancyArrowPatch):
  def __init__(self, xs, ys, zs, *args, **kwargs):
    FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
    self._verts3d = xs, ys, zs

  def draw(self, renderer):
    xs3d, ys3d, zs3d = self._verts3d
    xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
    self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
    FancyArrowPatch.draw(self, renderer)

## initialize serial port
ser = serial.Serial('/dev/ttyUSB0', 38400)
time.sleep(100E-3)

## set plot to update
plt.ion()
## initialize plot
plt3d = plt.figure().gca(projection='3d')
## set axis limits
plt3d.set_xlim(-1.25, 1.25)
plt3d.set_ylim(-1.25, 1.25)
plt3d.set_zlim(-1.25, 1.25)

## angles
alpha   = 0
beta    = 0
gamma   = 0
## sin/cos
sin_alpha   = 0
cos_alpha   = 1
sin_beta    = 0
cos_beta    = 1
sin_gamma   = 0
cos_gamma   = 1
## rotation matrices
rot_x = [[1, 0, 0], [0, cos_alpha, -sin_alpha], [0, sin_alpha, cos_alpha]]
rot_y = [[cos_beta, 0, sin_beta], [0, 1, 0], [-sin_beta, 0, cos_beta]]
rot_z = [[cos_gamma, -sin_gamma, 0], [sin_gamma, cos_gamma, 0], [0, 0, 1]]

## start point
s = [0, 0, 0]
## end points, one for each axis
ex = [1, 0, 0]
ey = [0, 1, 0]
ez = [0, 0, 1]
## rotate end points
ex_rot = np.dot(np.dot(np.dot(ex, rot_z), rot_y), rot_z)
ey_rot = np.dot(np.dot(np.dot(ey, rot_z), rot_y), rot_z)
ez_rot = np.dot(np.dot(np.dot(ez, rot_z), rot_y), rot_z)
## create arrows
ax = Arrow3D(*zip(s, ex_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
ay = Arrow3D(*zip(s, ey_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
az = Arrow3D(*zip(s, ez_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
## add arrows
plt3d.add_artist(ax)
plt3d.add_artist(ay)
plt3d.add_artist(az)
## label arrows
tx = plt3d.text(*ex_rot, 'x', size=20, zorder=1, color="r")
ty = plt3d.text(*ey_rot, 'y', size=20, zorder=1, color="g")
tz = plt3d.text(*ez_rot, 'z', size=20, zorder=1, color="b")

## show plot
plt.show()
plt.xlabel('NORTH')
plt.ylabel('WEST')

## maximum value, used for scaling
m = 32768
## update surface
while True:
  # bytesToRead = ser.inWaiting()
  # print('bytesToRead: %d' % (bytesToRead))

  ## wait for serial data
  while (ser.inWaiting() < 20):
    plt.pause(1E-6)

  ## read serial data
  data = ser.readline().rstrip()
  data = data.decode('utf-8')
  ## split serial data
  serial_gx, serial_gy, serial_gz, serial_heading = data.split(',')
  ## convert serial readings to numbers
  gx = int(serial_gx)
  gy = int(serial_gy)
  gz = int(serial_gz)
  gamma = int(serial_heading)
  ## calculate gmax for calibration
  # print('gmax: %.2f' % (math.sqrt(gx*gx + gy*gy + gz*gz)))
  gmax = 0.54
  ## convert angles to [-1; 1] and scale with gmax
  gx = gx / m / gmax
  gy = gy / m / gmax
  gz = gz / m / gmax
  gamma = gamma / 180 * math.pi
  # print('gx: %2.2f gy: %2.2f gz: %2.2f' % (gx, gy, gz))

  ## skip when exceeding boundaries
  if (math.sqrt(gx*gx + gy*gy + gz*gz)) <= 1:
    ## update alpha, beta angles
    cos_beta = math.sqrt(gy*gy + gz*gz)
    alpha = math.asin(-gy / cos_beta)
    # beta = math.acos(cos_beta)
    beta = math.asin(gx)
    sin_alpha = math.sin(alpha)
    cos_alpha = math.cos(alpha)
    sin_beta = math.sin(beta)
    sin_gamma = math.sin(gamma)
    cos_gamma = math.cos(gamma)
    ## update rotation matrix
    rot_x = [[1, 0, 0], [0, cos_alpha, -sin_alpha], [0, sin_alpha, cos_alpha]]
    rot_y = [[cos_beta, 0, sin_beta], [0, 1, 0], [-sin_beta, 0, cos_beta]]
    rot_z = [[cos_gamma, -sin_gamma, 0], [sin_gamma, cos_gamma, 0], [0, 0, 1]]
    ## rotate end points
    ex_rot = np.dot(rot_z, np.dot(rot_y, np.dot(rot_x, ex)))
    ey_rot = np.dot(rot_z, np.dot(rot_y, np.dot(rot_x, ey)))
    ez_rot = np.dot(rot_z, np.dot(rot_y, np.dot(rot_x, ez)))
    ## remove arrows
    ax.remove()
    ay.remove()
    az.remove()
    ## remove old arrow labels
    tx.remove()
    ty.remove()
    tz.remove()
    ## update arrows
    ax = Arrow3D(*zip(s, ex_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
    ay = Arrow3D(*zip(s, ey_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
    az = Arrow3D(*zip(s, ez_rot), mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
    ## add arrows
    plt3d.add_artist(ax)
    plt3d.add_artist(ay)
    plt3d.add_artist(az)
    ## add new arrow labels
    tx = plt3d.text(*ex_rot, 'x', size=20, zorder=1, color="r")
    ty = plt3d.text(*ey_rot, 'y', size=20, zorder=1, color="g")
    tz = plt3d.text(*ez_rot, 'z', size=20, zorder=1, color="b")

    print('alpha: %3.2f beta: %3.2f gamma: %3.2f'
          % (alpha * 180 / math.pi, beta * 180 / math.pi, gamma * 180 / math.pi))

  else:
    print('skip')

  plt.draw()
