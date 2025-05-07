from machine import Pin
from dht import DHT11  # Import DHT11 class
from time import sleep

from sys import stdout


sensor = DHT11(Pin(4))  # Set up DHT11 sensor on GPIO4

def run2 ():
    global temp, humidity  # Ensure they are accessible across the script
    while True:
        sensor.measure()  # Take a measurement
        temp = sensor.temperature  # Access the property without ()
        humidity = sensor.humidity  # Access the property without ()
        print(f"Temperature: {temp}°C, Humidity: {humidity}%")
        
        stdout.write(f"{temp},{humidity}\n")
        stdout.flush()  # Ensure the data is sent
        
        sleep(5)