from time import sleep
from machine import Pin, PWM

# Define motor pins
Mot1forward = PWM(Pin(12))
Mot1backward = PWM(Pin(13))
Mot2forward = PWM(Pin(11))
Mot2backward = PWM(Pin(10))

# Set a higher PWM frequency to reduce noise
Mot1forward.freq(20000)
Mot1backward.freq(20000)
Mot2forward.freq(20000)
Mot2backward.freq(20000)

# Enable motor driver
EnableMotors = Pin(9, Pin.OUT)

# Function to set motor speed in percentage
def set_motor_speed(motor, percent):
    percent = max(min(percent, 100), 0)  # Ensure it's within valid range
    duty = int(percent * 65535 / 100)  # Convert percent to duty cycle
    
    if motor == 1:
        Mot1forward.duty_u16(duty)
        Mot1backward.duty_u16(0)  # Ensure no reverse signal
    if motor == 2:
        Mot1forward.duty_u16(0)
        Mot1backward.duty_u16(duty)
    if motor == 3:
        Mot2forward.duty_u16(duty)
        Mot2backward.duty_u16(0)  # Ensure no reverse signal
    if motor == 4:
        Mot2forward.duty_u16(0)
        Mot2backward.duty_u16(duty)

while True:
    EnableMotors.high()  # Enable motors
    
    # Set speeds dynamically
    set_motor_speed(1, 70)  # Motor 1 at 25% speed
    set_motor_speed(3, 0)  # Motor 2 at 75% speed
  
    print("Moving motors...")
