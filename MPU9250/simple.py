from machine import I2C, Pin
from math import sqrt, atan2, pi
from mpu9250 import MPU9250
from time import time, sleep

# Initialize LED for program indication
LED = Pin(25, Pin.OUT)
LED.value(1)
LED.value(0)

# Configure I2C for MPU9250
I2C_ID = 0
SDA = Pin(0)
SCL = Pin(1)
i2c = I2C(id=I2C_ID, scl=SCL, sda=SDA)

# Initialize MPU9250
print("Scanning I2C bus...")
print(i2c.scan())
m = MPU9250(i2c)
print("MPU9250 Initialized!")

# Magnetometer Calibration
print("Calibrating magnetometer...")
m.ak8963.calibrate(count=10)
LED.value(0)  # Turn LED off after calibration
print("Calibration complete!")

# Kalman filter parameters
pitch, roll, yaw = 0.0, 0.0, 0.0
Q_angle, Q_bias, R_measure = 0.01, 0.003, 0.03
bias = 0.0
P = [[0.0, 0.0], [0.0, 0.0]]

# Time management for sensor readings
previous_time = time()
dt = 0.02

# Smoothing lists for magnetometer
mag_x_smooth = [0.0] * 5
mag_y_smooth = [0.0] * 5
previous_yaw = 0.0

# Initialize filtered values for low-pass filter
filtered_x_value = 0.0  # Initialize filtered x-value
filtered_y_value = 0.0  # Initialize filtered y-value

# Function for smoothing data
def smooth_data(new_value, data_list):
    data_list.pop(0)  # Remove oldest value
    data_list.append(new_value)  # Add new value
    return sum(data_list) / len(data_list)  # Compute average

# Low-pass filter function
def low_pass_filter(raw_value: float, remembered_value: float) -> float:
    alpha = 0.8
    return (alpha * remembered_value) + (1.0 - alpha) * raw_value

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

# Degrees to heading function (fixed missing definition)
def degrees_to_heading(degrees):
    if 0 <= degrees < 90:
        return "North-East"
    elif 90 <= degrees < 180:
        return "South-East"
    elif 180 <= degrees < 270:
        return "South-West"
    else:
        return "North-West"

# Main sensor reading and calculations
def get_reading():
    global previous_time, dt, pitch, roll, previous_yaw, filtered_x_value, filtered_y_value

    # Time management
    current_time = time()
    dt = current_time - previous_time
    if dt <= 0 or dt > 1:  # Ensure valid dt
        dt = 0.02
    previous_time = current_time

    # Read sensor data
    gx, gy, gz = m.gyro
    acc_x, acc_y, acc_z = m.acceleration
    mag_x, mag_y, mag_z = m.magnetic

    # Kalman filtering for pitch and roll
    pitch = kalman_filter(pitch, gx, atan2(-acc_x, sqrt(acc_y ** 2 + acc_z ** 2)) * 180 / pi)
    roll = kalman_filter(roll, gy, atan2(acc_y, acc_z) * 180 / pi)

    # Smooth magnetometer data
    mag_x = smooth_data(mag_x, mag_x_smooth)
    mag_y = smooth_data(mag_y, mag_y_smooth)
    # Calculate yaw from magnetometer
    yaw = atan2(mag_y, mag_x)
    declination_angle = 0.853933  # Magnetic declination for Timisoara
    yaw += declination_angle

    # Normalize yaw to 0-360 degrees
    if yaw < 0:
        yaw += 2 * pi
    if yaw > 2 * pi:
        yaw -= 2 * pi
    yaw = yaw * 180.0 / pi

    # Apply dampening for yaw
    yaw = 0.9 * previous_yaw + 0.1 * yaw
    previous_yaw = yaw


    # YAW=AZ BUT OTHER OPERATION
    # Low-pass filter for magnetometer values
    filtered_x_value = low_pass_filter(mag_x, filtered_x_value)
    filtered_y_value = low_pass_filter(mag_y, filtered_y_value)

    # Calculate azimuth (az)
    az = 90 - atan2(filtered_y_value, filtered_x_value) * 180 / pi

    # Ensure azimuth is always positive and between 0–360 degrees
    if az < 0:
        az += 360

    heading = degrees_to_heading(az)

    return pitch, roll, yaw, az, heading

# Main loop for real-time sensor output
while True:
    LED.on()
    pitch, roll, yaw, az, heading = get_reading()
    print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")