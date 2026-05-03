#import sys
#import threading
import sys
import time as t
#import select
import signal
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from scripts import Seq5, Seq6, utils
from scripts import Seq1, Seq2, Seq3, Seq4, Seq5, Seq6, Seq7, Seq_fin, Seq7_1
from scripts import tmp


signal.signal(signal.SIGINT, utils.shutdown_handler)
signal.signal(signal.SIGTERM, utils.shutdown_handler)

def navigate():
  try:
    #tmp.run()
    #Seq1.run()
    #Seq2.run()
    #Seq3.run()
    #Seq4.run()
    #Seq5.run()
    #Seq6.run()
    Seq7_1.run()
    #tmp.run()
    
  except KeyboardInterrupt:
    print("\n% CTRL+C pressed")

  finally:
    utils.stop_robot()
    service.stop = True
    service.terminate()
    print("Shutdown complete")
    sys.exit(0)

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      # set title of process, so that it is not just called Python
      setproctitle("mqtt-client")
      print("% Starting")
      # where is the MQTT data server:
      service.setup('localhost') # localhost
      #service.setup('10.197.217.81') # Juniper
      #service.setup('10.197.217.80') # Newton
      # service.setup('bode.local') # Bode
      if service.connected:
        navigate()
      service.terminate()
    print("% Main Terminated")