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
        utils.follow_line_for_distance(0.32, 0.2, mode="center")
        utils.drive_distance(0.2, -0.2)


  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()
    