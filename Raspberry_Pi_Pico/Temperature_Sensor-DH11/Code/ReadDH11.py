"""YOU CAN PASTE THE CODE IN YOU'RE CODE (Youre main.py) BY USING THE NEXT CODE( For temperature and humidity ):
    from ReadDH11 import DH
    
    dh = DH()
    temp, humidity = dh.readdh()  
    print(f"Temperature: {temp}°C, Humidity: {humidity}%") """

""" FIAX  """

from machine import Pin
from dht import DHT11
from time import sleep

# Define the pin of signal DH -> Raspberry Pi Pico 
sensor = DHT11(Pin(4))  

class DH:
    def __init__(self):
        self.temp = 0.0
        self.humidity = 0.0

    def readdh(self):
        sleep(5)  
        try:
            sensor.measure() 
            self.temp= sensor.temperature
            self.humidity = sensor.humidity
        except OSError:
            print("Sensor read error!")  
            self.temp, self.humidity = -1, -1  

        return self.temp, self.humidity  

dh = DH()

"""""
while True:
    temp, humidity = dh.readdh()  
    print(f"Temperature: {temp}°C, Humidity: {humidity}%")
"""""