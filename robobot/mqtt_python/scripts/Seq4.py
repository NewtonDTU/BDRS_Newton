# 90 SECS

import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
from uservice import service
from scripts import utils


def run():
  try:
    utils.driveTurn(np.pi/4, -0.5)
    utils.drive_distance(0.70, 0.30)
    utils.driveTurn(np.pi/4, -0.5)
    utils.drive_distance(0.85, 0.40)
    utils.driveTurn(np.pi, -0.7)
    utils.drive_distance(0.90, 0.40)
    utils.drive_until_line(0.4)

  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")


if __name__ == "__main__":
    run()