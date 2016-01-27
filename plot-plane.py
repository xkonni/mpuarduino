#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial

# initialize serial port
ser = serial.Serial('/dev/ttyUSB0', 38400)

# reference point
point  = np.array([0, 0, 1])
# create x,y,z
xx, yy = np.meshgrid(range(50), range(50))
zz = [0] * 50

# set plot to update
plt.ion()
# initialize plot
plt3d = plt.figure().gca(projection='3d')
surf = plt3d.plot_surface(xx, yy, zz[0], color='red')
plt3d.set_zlim(-100, 100)

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
  sx, sy, sz = data.split(',')
  # convert to number
  x = int(sx)
  y = int(sy)
  z = int(sz)
  # convert to [-1; 1]
  x = round(x/m, 2)
  y = round(y/m, 2)
  z = round(z/m, 2)

  d = np.sum(point*[x, y, z])
  print('%.2f %.2f %.2f' % (x, y, z))
  # calculate corresponding z
  # dont divide by zero
  if (z == 0): z = 0.01
  zz = (x*xx + y*yy + d)*1./z
  surf =  plt3d.plot_surface(xx, yy, zz, color='blue')
  plt.draw()
  plt.pause(100E-6)
