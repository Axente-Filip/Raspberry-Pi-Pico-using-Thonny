from ReadMPU9250 import MPU
from ReadDH11 import DH
import _thread
import time
from MotorControl import Motor
from machine import SPI, Pin, I2C, UART

LED = Pin(25, Pin.OUT)
LED.value(1)
                                                                    
motor = Motor()
mpu = MPU()
dh = DH()

while True:
    
    #temp, humidity = dh.readdh()  
    #pitch, roll, yaw = mpu.get_reading()
    #print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
    #print(f"Temperature: {temp}°C, Humidity: {humidity}%")
    speed, roll_target, roll, pitch, yaw= motor.work()
    print(f"pitch: {pitch},roll: {roll},yaw: {yaw}")	