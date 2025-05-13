import network
import socket
import ReadNrf24
from time import sleep
from machine import Pin, I2C


# Connect to network
ssid = 'ASUS etaj 2' # The network you-re connected name
password = '123456789'

I2C_ID = 0
SDA = Pin(0)      #	Pin on Raspberry Pi Pico for SDA MPU9250
SCL = Pin(1)      #	Pin on Raspberry Pi Pico for SCL MPU9250
i2c = I2C(id=I2C_ID, scl=SCL, sda=SDA)

def connect ():
    wlan  = network.WLAN (network.STA_IF)
    wlan.active (True)
    wlan.connect (ssid, password)
    while wlan.isconnected() == False:
        print('Connecting ... ')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    return ip

def open_socket(ip):
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(addres)
    connection.listen(1)
    return connection

def webpage(reading):
    html = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <title>Raspberry Pi Pico FIAX </title>
        <meta http-equiv="refresh" content="10">
        </head>
        <body>
        <p>{reading}<\p>
        </body>
        </html>
        """
    return str(html)

def server (connection):
    # Starting the web servers
    
    while True:
        reading = 'FIAX'
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str (request)
        html = webpage (reading)
        client.send (html)
        client.close()
        
try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)
except KeyboardIntrerrupt:
    machine.reset()
    