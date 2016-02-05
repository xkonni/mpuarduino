#!/usr/bin/python
import numpy as np
import serial
import math
import time

def getHeading(ax, ay, az, mx, my, mz):
  ax = ax / ( 1 << 15)
  ay = ay / ( 1 << 15)
  az = az / ( 1 << 15)
  # pitch = asin(-realA->x)
  pitch = -math.acos(-ax) + math.pi/2
  sin_pitch = math.sin(pitch)
  cos_pitch = math.cos(pitch)
  # roll = asin(ay/cos_pitch)
  roll = -math.acos(ay/cos_pitch) + math.pi/2
  cos_roll = math.cos(roll)
  sin_roll = math.sin(roll)

  xh = mx * cos_pitch + mz * sin_pitch
  yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch

  heading = 180 * math.atan2(yh, xh)/math.pi
  if (heading < 0): heading += 360
  return heading

## initialize serial port
ser = serial.Serial('/dev/ttyUSB0', 38400)
time.sleep(100E-3)

## maximum value, used for scaling
m = 32768
bmax = 0
## update surface
while True:
  ## wait for serial data
  while (ser.inWaiting() < 40):
    time.sleep(1E-6)

  ## read serial data
  data = ser.readline().rstrip()
  data = data.decode('utf-8')
  ## split serial data
  serial_gx, serial_gy, serial_gz, serial_mx, serial_my, serial_mz = data.split(',')
  ## convert serial readings to numbers
  gx = int(serial_gx)
  gy = int(serial_gy)
  gz = int(serial_gz)
  mx = int(serial_mx)
  my = int(serial_my)
  mz = int(serial_mz)

  ## calculate gmax for calibration
  # print('gmax: %.2f' % (math.sqrt(gx*gx + gy*gy + gz*gz)))
  gmax = 0.54
  ## convert angles to [-1; 1] and scale with gmax
  gx = gx / m / gmax
  gy = gy / m / gmax
  gz = gz / m / gmax
  bmax = math.sqrt(gx*gx + gy*gy + gz*gz)
  # print('bmax: %f' % bmax)
  ## 
  if (bmax <= 1.02) and (bmax >= 0.96):
    print('g: %2.2f/%2.2f/%2.2f |\tm: %4d/%4d/%4d |\t h: %3d' % (gx, gy, gz, mx, my, mz, getHeading(gx, gy, gz, mx, my, mz)))
  # else:
  #   print('skip')

  # ## skip when exceeding boundaries
  # bmax = math.sqrt(gx*gx + gy*gy + gz*gz)
  # print('bmax: %f' % bmax)
  # if (bmax <= 1) and (bmax >= 0.98):
  #   print('alpha: %3.2f beta: %3.2f gamma: %3.2f bmax: %f'
  #         % (alpha * 180 / math.pi, beta * 180 / math.pi, gamma * 180 / math.pi, bmax))
  #
  # else:
  #   print('skip bmax: %f' % bmax)
