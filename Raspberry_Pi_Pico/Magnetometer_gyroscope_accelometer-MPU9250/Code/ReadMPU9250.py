"""	YOU CAN PASTE THE CODE IN YOU'RE CODE (Youre main.py) BY USING THE NEXT CODE( For pytch, yaw and roll ):
                                                                    from ReadMPU9250 import MPU
                                                                    
                                                                    mpu = MPU()
                                                                    pitch, roll, yaw = mpu.get_reading()
                                                                    print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")														"""

""" FIAX """
import time
from machine import I2C, Pin
from math import sqrt, atan2, pi
from mpu9250 import MPU9250
from time import time, sleep
from mpu6500 import ACCEL_FS_SEL_8G
from mpu6500 import GYRO_FS_SEL_1000DPS

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
m = MPU9250(i2c, accel_fs=ACCEL_FS_SEL_8G, gyro_fs=GYRO_FS_SEL_1000DPS)		# MPU9250 is "m" in entire code
print("MPU9250 Initialized!")

acc_x_filtered = 0
acc_y_filtered = 0
prev_acc_x = 0
prev_acc_y = 0

class MPU:
    def __init__ (self):
        # Time management for sensor readings
        self.alpha = 0.98  # Weight for gyro influence
        self.prev_acc_x = 0
        self.prev_acc_y = 0
        self.previous_time = time()
        self.dt = 0.01
        self.gx = 0
        self.gy = 0
        self.gz  = 0
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        """ MAGNETOMETER SETTING """
        self.declination_angle = 0.853933  # Magnetic declination for Timisoara = declanation betwee original NORTH and our NORTH ( Timisoara in my case )
        self.filtered_magx = 0
        self.filtered_magy = 0
        self.Roll = 0
        self.Pitch = 0
        
        """For the rest"""
        self.pitch, self.roll, self.yaw = 0.0, 0.0, 0.0
        self.Q_angle = 0.0001  # Allow faster adaptation
        self.Q_bias = 0.0001  # Reduce gyro bias drift
        self.R_measure = 10  # Handle measurement noise
        
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        
        """Magnetometer Calibration"""
        # Fix Hard and Soft Iron distorsion
        print("Calibrating magnetometer...")
        # You need to rotate magnetometer in this time (in my case 100 step, you can have max 256 steps), for better accurity and better calibration, it calculate max and mic value that can get the magnetometer 
        # Run calib for n steps, set: True if you want to calib/False if you have the value already after calib (you gave the value on terminal after calib just copy->paste on def calibrate at else )
        # I sugest to run at first time for 256, rotate with the Yaw axe for better result; after you can put it false and you re value in library (explained up)
        m.ak8963.calibrate(False,count=1024)
        m.mpu6500.calibrate(False,count = 1024)
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
        self.dt = max(1, min(current_time - self.previous_time, 0.001))  # Keeps updates smooth
        self.previous_time = current_time

        # Read sensor data gyro, accelometer and magnetometer
        self.gx, self.gy, self.gz = m.gyro
        self.acc_x, self.acc_y, self.acc_z = m.acceleration
        mag_x, mag_y,  mag_z = m.magnetic
        
        self.acc_x_filtered = self.acc_x * 0.95 + self.prev_acc_x * 0.05
        self.acc_y_filtered = self.acc_y * 0.95 + self.prev_acc_y * 0.05
        
        # Store previous values for next iteration
        self.prev_acc_x = self.acc_x_filtered
        self.prev_acc_y = self.acc_y_filtered

        #print(f"gx: {self.gx}, gy: {self.gy}, gz: {self.gz}, acc_x: {self.acc_x}, acc_y: {self.acc_y}, acc_z: {self.acc_z}, mag_x: {mag_x}, mag_y: {mag_y}, mag_z: {mag_z}")
        
         # Normal Kalman filtering

        # Apply complementary filter for roll and pitc
        """ 
        self.roll = self.kalman_filter(self.roll, self.gx, atan2(self.acc_x_filtered*-1, sqrt(self.acc_y_filtered ** 2 + self.acc_z ** 2)) * 180 / pi)
        self.pitch = self.kalman_filter(self.roll, self.gy, atan2(self.acc_y_filtered, self.acc_z) * 180/ pi)
        """ 
        # Compute roll and pitch from accelerometer
        pitch_acc  = atan2(self.acc_x_filtered*-1, sqrt(self.acc_y_filtered ** 2 + self.acc_z ** 2)) * 180 / pi
        roll_acc = atan2(self.acc_y_filtered, self.acc_z) * 180 / pi

        # Integrate roll and pitch from gyroscope
        pitch_gyro = self.roll + self.gx * self.dt
        roll_gyro = self.pitch + self.gy * self.dt

        # Complementary filter to blend both sources
        self.roll = (0.99 * roll_gyro + 0.01 * roll_acc)
        self.pitch = (0.99 * pitch_gyro + 0.01 * pitch_acc)
        
        """
        self.roll = self.kalman_filter(self.roll, self.gx, roll_acc)
        self.pitch = self.kalman_filter(self.pitch, self.gy, pitch_acc)
        """
        """      
        self.roll = atan2(self.acc_x*-1, sqrt(self.acc_y ** 2 + self.acc_z ** 2)) * 180 / pi
        self.pitch = atan2(self.acc_y, self.acc_z) * 180/ pi
        """
        """
        self.roll = self.kalman_filter(self.roll, self.gx, atan2(self.acc_x*-1, sqrt(self.acc_y ** 2 + self.acc_z ** 2)) * 180 / pi)
        self.pitch = self.kalman_filter(self.roll, self.gy, atan2(self.acc_y, self.acc_z) * 180/ pi)
        """
        """"        roll_threshold = max(40, abs(self.roll) * 0.8)  # Adaptive threshold
        gyro_threshold = 80  # Keep gyro sensitivity reasonable
        if abs(self.gx) > gyro_threshold or abs(self.gy) > gyro_threshold or abs(self.roll) > roll_threshold:
            print("Falling detected!")"""

        """ MAGNETOMETER """
        # Calculate yaw from magnetometer
        self.filtered_magx = self.low_pass_filter(self.filtered_magx, mag_x)
        self.filtered_magy = self.low_pass_filter(self.filtered_magy, mag_y)
        
        # Normalize yaw to 0-360 degrees
        self.yaw = (atan2(self.filtered_magx, self.filtered_magy) * (180 / pi)) + self.declination_angle
        if self.yaw < 0:
            self.yaw += 360

        return round(self.pitch, 2), round(self.roll, 2), round(self.yaw, 2)
    
mpu = MPU()
""" 
class READING():
    #READ THE DATA
    while True:
        pitch, roll, yaw = mpu.get_reading()
        print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")	
""" 