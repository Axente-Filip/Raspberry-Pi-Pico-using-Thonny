"""	YOU CAN PASTE THE CODE IN YOU'RE CODE (Youre main.py) BY USING THE NEXT CODE( For pytch, yaw and roll ):
                                                                    from MotorControl import Motor
                                                                    
                                                                    motor = Motor()
                                                                    speed, roll_target, roll, pitch, yaw= motor.work()
                                                                    print(f"pitch: {pitch},roll: {roll},yaw: {yaw}")														"""

""" FIAX """
import time
from time import sleep
from machine import Pin, PWM
from ReadMPU9250 import MPU
mpu = MPU()

# Define motor pins
Mot1forward = PWM(Pin(12))
Mot1backward = PWM(Pin(13))
Mot2forward = PWM(Pin(7))
Mot2backward = PWM(Pin(10))

# Set a higher PWM frequency to reduce noise
Mot1forward.freq(25000)
Mot1backward.freq(25000)
Mot2forward.freq(25000)
Mot2backward.freq(25000)

# Enable motor driver
EnableMotors = Pin(6, Pin.OUT)
class Motor():
    def __init__(self):
        # PID Parameters (Proportional-only control for smooth movement), start with kp , kd  and last ki
        # use you re value, try again and again 
        self.kp = 60  			# Proportional Gain => High value -> increase brutality of motor when he try to balance
        self.kd = 1			# => Future -> Recorect the robot after was corrected ()
        self.ki = 100			# => Past -> Correct long term error
        # I *100 because I use duty for control the motor witch is max=65535, so 40*100=4000 min
        # set this vealue to reaction faster
        self.kp = self.kp * 100
        self.kd = self.kd * 100
        self.ki = self.ki * 100
        
        # rest for work()
        self.error = 0
        self.roll_target = 12.5 #15 to lean forward -> 15.5 to lean back
        self.prev_error = 0
        self.PID = 0
        self.Times = 0
        self.p = 0
        self.i = 0
        self.d = 0
        self.speed = 0

        self.integral_max = 20000  # Prevent the integral term from growing too large
        self.integral_min = -20000  # Lower limit for the integral term
        
        # Set for time
        self.millis = 0
        self.timePrev = 0
        self.Times = 0
        self.elapsedTime = 0
    # Function to set motor speed using duty cycle smoothly
    def set_motor_speed(self,motor_forward, motor_backward, duty):
        duty = int(max(min(duty, 65535), -65535)) # Limit duty cycle range
        if duty > 0:
            motor_forward.duty_u16(duty)
            motor_backward.duty_u16(0)
        elif duty < 0:
            motor_forward.duty_u16(0)
            motor_backward.duty_u16(-duty)  # Convert negative duty for backward motion
        else:
            motor_forward.duty_u16(0)
            motor_backward.duty_u16(0)  # Stop the motor smoothly
    
    def work(self):
        EnableMotors.value(1)  # Ensure motors are enabled
        # Prepare the time
        self.millis = time.ticks_ms()
        self.timePrev = self.Times
        self.Times = self.millis
        self.elapsedTime = max((self.Times - self.timePrev) / 1000, 0.001)  # Ensure at least 1ms

        # Read from MPU the roll
        roll, pitch, yaw = mpu.get_reading()
        
        # I move constant the balance point = 0 because at long term the robot lean back or forward
        # so when he wna t to lean back, I move the balance point forward than the normal , so he try
        # to redresing faster (like penduls)
        #if roll < 15:
        #    self.roll_target = 14.8
        #if roll > 15:
        #    self.roll_target = 15
        
        # Proportional-only control (no integration or derivative)
        self.error = self.roll_target - roll
        self.p = self.kp * self.error
        self.i += self.ki * self.error * self.elapsedTime
        self.i = max(min(self.i, self.integral_max), self.integral_min) 
        self.d = self.kd*((self.error - self.prev_error) / self.elapsedTime  )
        self.PID = self.p + self.i + self.d
        self.prev_error =self. error
        self.speed = self.PID
        
        # I set this because I use 7.4V battery, witch is low, so my motor start to function(spin) at duty=20k
        if self.speed > 1:
            self.speed += 20000
        if self.speed < -1:
            self.speed -= 20000

        # Set motor speeds based on direct angle-based control
        self.set_motor_speed(Mot1forward, Mot1backward, self.speed)
        self.set_motor_speed(Mot2forward, Mot2backward, self.speed)
        
        # return value for print in you're main code
        return self.speed, self.roll_target, roll, pitch, yaw
        
motor = Motor()

"""
class READING():
    while True:
        speed, roll_target, roll, pitch, yaw= motor.work()
        print(f"pitch: {pitch},roll: {roll},yaw: {yaw}")
"""