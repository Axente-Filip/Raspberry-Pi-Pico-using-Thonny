from nrf24l01 import NRF24L01
from machine import SPI, Pin, I2C, UART
import utime
import time
import struct
from ReadMPU9250 import MPU
#from MotorControl import Motor
#motor = Motor()
mpu = MPU()

# Turn on the LED from Raspberry Pi Pico => the device is functional/ON
LED = Pin(25, Pin.OUT)
LED.value(1)

# CODE FOR NRF24
"""
GND   -> Pin 38                              				|    VCC         -> Pin 36
CE     -> GPIO14   # Chip Enable              			|    CS           -> GPIO15  # Chip Select Not
SCK   -> GPIO18   # Clock                     			|    MOSI       -> GPIO19  # Master Output -> Slave Input
MISO -> GPIO16   # Master Input -> Slave Output 	|    Interrupt   -> 
"""

"""To reduce the noise: Wrap two cables between them
GND -> SCK
MISO -> MOSI
"""

cs = Pin(15, mode=Pin.OUT, value=1)  # GPIO15
ce = Pin(14, mode=Pin.OUT, value=0)  # GPIO14
spi = SPI(0, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
Payload_size = 24  # Set max/avg byte send and receive in a message; max we can transmit is 32 bytes

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Choose your role
role = "send"
# role = "receive"

if role == "send":
    send_pipe = b"\xe1\xf0\xf0\xf0\xf0"
    receive_pipe = b"\xd2\xf0\xf0\xf0\xf0"
else:
    send_pipe = b"\xd2\xf0\xf0\xf0\xf0"
    receive_pipe = b"\xe1\xf0\xf0\xf0\xf0"

def setup():
    print("Initializing NRF24L01 Module...")
    nrf = NRF24L01(spi, cs, ce, payload_size = Payload_size)  # Ensure this comes BEFORE flushing
    nrf.open_tx_pipe(send_pipe)
    nrf.open_rx_pipe(1, receive_pipe)
    nrf.set_channel(100)
    # Power dB and speed bps
    POWER_3 = 0x06  # 0 dBm
    SPEED_1M = const(0x00)  
    nrf.set_power_speed(POWER_3, SPEED_1M)	# High power, and 1Mbps transmision; we can chose between 250Kbps, 1Mbps, and  2Mbps; higher we chose, the distante is schord , iinterference high and speed transmison high
    # Auto Acknowledgment
    nrf.set_auto_ack(False) # Auto Acknowledgment -> If is True, it verify if the receiver get an information you sended; turn off if you dont have receiver yet *
    
    nrf.start_listening()
    return nrf

def send(nrf, msg):
    print("sending message.", msg)
    nrf.stop_listening()
    for n in range(len(msg)):
        try:
            encoded_string = msg[n].encode()
            byte_array = bytearray(encoded_string)
            #buf = struct.pack("s", byte_array) # Packing string
            buf = struct.pack("fff", pitch, roll, yaw)  # Packing three float values
            nrf.send(buf)
            # Check live what is transmited, you can pu this print in comment if you don t want to see it
            print(role, "message", msg[n], "sent")
        except OSError:
            # If you dont have receiver, and wana test the module : nrf.set_auto_ack(False) oi if you don t wana get confirm from receiver
            print(role, "Sorry message not sent")
    nrf.send("\n")
    nrf.start_listening()

"""!!! THE MAIN CODE !!!"""
nrf = setup()
nrf.start_listening()
msg_string = ""

while True:
    msg = ""
    if role == "send":
        # Use get_pitch_roll_yaw() instead of get_reading() to don t conflict I2C with SPI
        pitch, roll, yaw = mpu.get_pitch_roll_yaw()
        #pitch, roll, yaw = motor.work()
        send(nrf, f"{pitch}; {roll}; {yaw}")
        #send(nrf, "Hello World")
    else:
        # Check for Messages
        if nrf.any():
            package = nrf.recv()
            message = struct.unpack("s", package)
            msg = message[0].decode()

            # Check for the new line character
            if (msg == "\n") and (len(msg_string) <= 32):
                print("full message", msg_string, msg)
                msg_string = ""
            else:
                if len(msg_string) <= 32:
                    msg_string = msg_string + msg
                else:
                    msg_string = ""