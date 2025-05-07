from ReadDH11 import *
from ReadMPU9250 import *
import _thread
import time

chore1 = _thread.start_new_thread (run2, () )
#chore2 = threding.Thread(target = run2)

run1()