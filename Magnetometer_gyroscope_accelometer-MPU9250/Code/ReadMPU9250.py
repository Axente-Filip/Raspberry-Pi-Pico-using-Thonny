"""	YOU CAN PASTE THE CODE IN YOU'RE CODE (Youre main.py) BY USING THE NEXT CODE( For pytch, yaw and roll ):
                                                                    from ReadMPU9250 import MPU
                                                                    
                                                                    mpu = MPU()
                                                                    pitch, roll, yaw = mpu.get_reading()
                                                                    print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")														"""

""" FIAX """

from machine import I2C, Pin
from math import sqrt, atan2, pi
from mpu9250 import MPU9250
from time import time, sleep

#class READING_MPU:
# Initialize LED for program indication
LED = Pin(25, Pin.OUT)
LED.value(1)
LED.value(0)

# Configure I2C for MPU9250
I2C_ID = 0
SDA = Pin(0)      #	Pin on Raspberry Pi Pico for SDA MPU9250
SCL = Pin(1)      #	Pin on Raspberry Pi Pico for SCL MPU9250
i2c = I2C(id=I2C_ID, scl=SCL, sda=SDA)

# Initialize MPU9250
print("Scanning I2C bus...")
print(i2c.scan())
m = MPU9250(i2c)		# MPU9250 is "m" in entire code
print("MPU9250 Initialized!")

class MPU:
    def __init__ (self):
        # Time management for sensor readings
        self.previous_time = time()
        self.dt = 0.01

        """ MAGNETOMETER SETTING """
        self.declination_angle = 0.853933  # Magnetic declination for Timisoara = declanation betwee original NORTH and our NORTH ( Timisoara in my case )
        self.filtered_magx = 0
        self.filtered_magy = 0
        
        """For the rest"""
        self.pitch, self.roll, self.yaw = 0.0, 0.0, 0.0
        self.Q_angle, self.Q_bias, self.R_measure = 0.01, 0.003, 0.03
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        
        """Magnetometer Calibration"""
        # Fix Hard and Soft Iron distorsion
        print("Calibrating magnetometer...")
        # You need to rotate magnetometer in this time (in my case 100 step, you can have max 256 steps), for better accurity and better calibration, it calculate max and mic value that can get the magnetometer 
        # Run calib for n steps, set: True if you want to calib/False if you have the value already after calib (you gave the value on terminal after calib just copy->paste on def calibrate at else )
        # I sugest to run at first time for 256, rotate with the Yaw axe for better result; after you can put it false and you re value in library (explained up)
        m.ak8963.calibrate(False,count=1)
        LED.value(0)  						# Turn LED during calibration
        print("Calibration complete!")

    # Low-pass filter 
    """ Magnetometer is sensible to high ferquency, vibration and inteference from other device/ component-> attenuating high frequency signal; Earth freq = 0.1 Hz """
    def low_pass_filter(self, preveu_value, new_value):
        return 0.99 * preveu_value +  0.01 * new_value
        # 0.85+ 0.15 need to be only =1, so we can chose 0.9 and 0.1 = 1, the value of new_value incrase=> the truth about preveu_value will incrase => distorsion increase at output (it will be shaky )
        # if we incrase the vlaue of " * new_value", it will be slower (rotate slow), but the shaky will be eliminated much better 

    """ KALMAN SETTING """
    # Kalman filter implementation
    def kalman_filter(self, angle, gyro_rate, accel_angle):
        rate = gyro_rate - self.bias
        angle += self.dt * rate

        self.P[0][0] += self.dt * (self.dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= self.dt * self.P[1][1]
        self.P[1][0] -= self.dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * self.dt

        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = accel_angle - angle
        angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return angle

    # Main sensor reading and calculations
    def get_reading(self):
        # Time management
        current_time = time()
        self.dt = current_time - self.previous_time
        if self.dt <= 0 or self.dt > 1:  # Ensure valid dt
            self.dt = 0.02
        self.previous_time = current_time

        # Read sensor data gyro, accelometer and magnetometer
        gx, gy, gz = m.gyro
        acc_x, acc_y, acc_z = m.acceleration
        mag_x, mag_y,  mag_z = m.magnetic

        # Kalman filtering for pitch and roll
        self.pitch = self.kalman_filter(self.pitch, gx, atan2(-acc_x, sqrt(acc_y ** 2 + acc_z ** 2)) * 180 / pi)
        self.roll = self.kalman_filter(self.roll, gy, atan2(acc_y, acc_z) * 180 / pi)

        """ MAGNETOMETER """
        # Calculate yaw from magnetometer
        self.filtered_magx = self.low_pass_filter(self.filtered_magx, mag_x)
        self.filtered_magy = self.low_pass_filter(self.filtered_magy, mag_y)
        
        # Normalize yaw to 0-360 degrees
        self.yaw = (atan2(self.filtered_magx, self.filtered_magy) * (180 / pi)) + self.declination_angle
        if self.yaw < 0:
            self.yaw += 360

        return round(self.pitch, 2), round(self.roll, 2), round(self.yaw, 2)  
    
    def get_pitch_roll_yaw(self):
        return round(self.pitch, 2), round(self.roll, 2), round(self.yaw, 2)
    
mpu = MPU()

"""
class READING():
    
    #READ THE DATA
    while True:
        LED.on()
        pitch, roll, yaw = mpu.get_reading()
        print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
"""
