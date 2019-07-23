import MPU9250
import time

mpu9250 = MPU9250.MPU9250()

while True:
  accel = mpu9250.readAccel()
  gypr = mpu9250.readGyro()
  magnet=mpu9250.readMagnet()
  print("accel  X: " + str(accel['x']) +  " accel  Y: " + str(accel['y']) + " accel  Z: " + str(accel['z']))
  print("gypr   X: " + str(gypr['x']) +   " gypr   Y: " + str(gypr['y']) +  " gypr   Z: " + str(gypr['z']))
  print("magnet X: " + str(magnet['x']) + " magnet Y: " + str(magnet['y'])+ " magnet Z: " + str(magnet['z']))
  time.sleep(0.1)