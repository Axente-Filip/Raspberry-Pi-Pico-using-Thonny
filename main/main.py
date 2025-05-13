from ReadMPU9250 import MPU
from ReadDH11 import DH
import _thread
import time

mpu = MPU()
dh = DH()

while True:
    
    #temp, humidity = dh.readdh()  
    pitch, roll, yaw = mpu.get_reading()
    #print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
    #print(f"Temperature: {temp}°C, Humidity: {humidity}%")