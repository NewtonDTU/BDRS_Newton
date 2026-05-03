# ROUNDABOUT to 90 SECS

#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from scripts import utils

'''
def stop_robot():
  edge.lineControl(0, mode="center")
  for _ in range(10):
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)
'''



def run():
  try:
    if service.connected:
        # follow line for a chosen distance in meters
        utils.follow_line_for_distance(2.18, 0.5, "center")

        utils.stop_robot()

  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()
    