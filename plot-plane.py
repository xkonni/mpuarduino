#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import math

# initialize serial port
ser = serial.Serial('/dev/ttyUSB0', 38400)

# reference point
point  = np.array([0, 0, 0])
# create gx,gy,gz
rho, phi = np.meshgrid(range(0, 20), range(0, 360, 18))
phi = phi + 9
xx = rho/9 * np.cos(phi/180*math.pi)
yy = rho/9 * np.sin(phi/180*math.pi)
zz = [0] * 20

# set plot to update
plt.ion()
# initialize plot
plt3d = plt.figure().gca(projection='3d')
surf = plt3d.plot_surface(xx, yy, zz[0], color='red')
plt3d.set_zlim(-1, 1)

m = 32768
# update surface
while True:
  surf.remove()
  data = ser.readline().rstrip()
  if (len(data) < 5):
    print('bad things happened, try again')
    exit(1)

  data = data.decode('utf-8')
  # split serial data
  serial_gx, serial_gy, serial_gz, serial_heading = data.split(',')
  # convert serial readings to numbers
  gx = int(serial_gx)
  gy = int(serial_gy)
  gz = int(serial_gz)
  heading = int(serial_heading)
  # calculate gmax for calibration
  # print('gmax: %.2f' % (math.sqrt(gx*gx + gy*gy + gz*gz)))
  gmax = 0.54
  # convert angles to [-1; 1] and scale with gmax
  gx = gx / m / gmax
  gy = gy / m / gmax
  gz = gz / m / gmax

  # skip when exceeding boundaries
  if (math.sqrt(gx*gx + gy*gy + gz*gz)) <= 1:
    rho, phi = np.meshgrid(range(0, 20), range(0, 360, 18))
    phi = phi - heading + 9
    xx = rho/9 * np.cos(phi/180*math.pi)
    yy = rho/9 * np.sin(phi/180*math.pi)

    d = np.sum(point*[gx, gy, gz])
    print('%.2f %.2f %.2f %3d' % (gx, gy, gz, heading))
    # calculate corresponding gz
    # dont divide by zero
    if (gz == 0): gz = 0.01
    zz = (gx*xx + gy*yy + d)*1./gz
  # else:
  #   print('skip')

  surf = plt3d.plot_surface(xx, yy, zz, color='blue')
  plt.xlabel('north')
  plt.ylabel('west')
  plt.draw()
  plt.pause(100E-6)
