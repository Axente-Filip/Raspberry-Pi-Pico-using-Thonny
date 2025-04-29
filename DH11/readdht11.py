from machine import Pin
from dht import DHT11  # Import DHT11 class
from time import sleep

sensor = DHT11(Pin(4))  # Set up DHT11 sensor on GPIO4

while True:
    sensor.measure()  # Take a measurement
    temp = sensor.temperature  # Access the property without ()
    humidity = sensor.humidity  # Access the property without ()
    print(f"Temperature: {temp}°C, Humidity: {humidity}%")
    sleep(1)