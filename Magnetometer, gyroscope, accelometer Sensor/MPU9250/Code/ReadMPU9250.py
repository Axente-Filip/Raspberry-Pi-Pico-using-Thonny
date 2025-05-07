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

# Time management for sensor readings
previous_time = time()
dt = 0.01

""" MAGNETOMETER SETTING """
declination_angle = 0.853933  # Magnetic declination for Timisoara = declanation betwee originat NORTH and our NORTH ( Timisoara in my case )
filtered_magx = 0
filtered_magy = 0
# Magnetometer Calibration
# Fix Hard and Soft Iron distorsion
print("Calibrating magnetometer...")
# You need to rotate magnetometer in this time (in my case 100 step, you can have max 256 steps), for better accurity and better calibration, it calculate max and mic value that can get the magnetometer 
m.ak8963.calibrate(count=100)		# Run calib for 100 step instead of max 256, we need to move the magnetometer for calib
LED.value(0)  						# Turn LED during calibration
print("Calibration complete!")

# Low-pass filter 
""" Magnetometer is sensible to high ferquency, vibration and inteference from other device/ component-> attenuating high frequency signal; Earth freq = 0.1 Hz """
def low_pass_filter(preveu_value, new_value):
    return 0.99 * preveu_value +  0.01 * new_value
    # 0.85+ 0.15 need to be only =1, so we can chose 0.9 and 0.1 = 1, the value of new_value incrase=> the truth about preveu_value will incrase => distorsion increase at output (it will be shaky )
    # if we incrase the vlaue of " * new_value", it will be slower (rotate slow), but the shaky will be eliminated much better 

""" KALMAN SETTING """
# Kalman filter parameters
pitch, roll, yaw = 0.0, 0.0, 0.0
Q_angle, Q_bias, R_measure = 0.01, 0.003, 0.03
bias = 0.0
P = [[0.0, 0.0], [0.0, 0.0]]

# Initialize filtered values for low-pass filter
filtered_x_value = 0.0  # Initialize filtered x-value
filtered_y_value = 0.0  # Initialize filtered y-value
    
# Main loop for real-time sensor Read
#while True:
#    LED.on()
#    pitch, roll, yaw = get_reading()
#    print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
# Kalman filter implementation
def kalman_filter(angle, gyro_rate, accel_angle):
    global P, Q_angle, Q_bias, R_measure, dt, bias
    rate = gyro_rate - bias
    angle += dt * rate

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt

    S = P[0][0] + R_measure
    K = [P[0][0] / S, P[1][0] / S]

    y = accel_angle - angle
    angle += K[0] * y
    bias += K[1] * y

    P00_temp = P[0][0]
    P01_temp = P[0][1]
    P[0][0] -= K[0] * P00_temp
    P[0][1] -= K[0] * P01_temp
    P[1][0] -= K[1] * P00_temp
    P[1][1] -= K[1] * P01_temp

    return angle

# Main sensor reading and calculations
def get_reading():
    global previous_time, dt, pitch, roll, previous_yaw, filtered_magx, filtered_magy, declination_angle
    
    # Time management
    current_time = time()
    dt = current_time - previous_time
    if dt <= 0 or dt > 1:  # Ensure valid dt
        dt = 0.02
    previous_time = current_time

    # Read sensor data gyro, accelometer and magnetometer
    gx, gy, gz = m.gyro
    acc_x, acc_y, acc_z = m.acceleration
    mag_x, mag_y,  mag_z = m.magnetic

    # Kalman filtering for pitch and roll
    pitch = kalman_filter(pitch, gx, atan2(-acc_x, sqrt(acc_y ** 2 + acc_z ** 2)) * 180 / pi)
    roll = kalman_filter(roll, gy, atan2(acc_y, acc_z) * 180 / pi)

    """ MAGNETOMETER """
    # Calculate yaw from magnetometer
    filtered_magx = low_pass_filter (filtered_magx, mag_x)
    filtered_magy = low_pass_filter (filtered_magy, mag_y)
    
    yaw = ( atan2(filtered_magx, filtered_magy) * (180 / pi) ) + declination_angle

    # Normalize yaw to 0-360 degrees
    if yaw < 0:
        yaw += 360

    return pitch, roll, yaw

def run1 ():
        while True:
            LED.on()
            pitch, roll, yaw = get_reading()
            print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")