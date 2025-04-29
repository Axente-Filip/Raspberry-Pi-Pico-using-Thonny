import _thread
from simple import *
from readdht11 import *
import time

# Function to run simple.py
def run_simple():
    while True:
        print("Running simple.py...")
        time.sleep(1)  # Simulate the work done in simple.py

# Function to run readdht11.py
def run_readdht11():
    while True:
        print("Running readdht11.py...")
        time.sleep(1)  # Simulate the work done in readdht11.py

# Start the first script in a new thread
_thread.start_new_thread(run_simple, ())

# Run the second script on the main thread
run_readdht11()