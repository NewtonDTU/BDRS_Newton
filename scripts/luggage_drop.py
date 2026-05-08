
from uservice import service
import numpy as np

from scripts import utils

def run():
  try:
    service.send("robobot/cmd/T0","servo 1 -900 0") #-900 is max up, 3000 down, 200 middle
    utils.driveTurn(np.pi/2, 0.5)
    utils.drive_distance(1.05, 0.3)

    #utils.follow_line_for_distance(0.8, 0.2, "right")


  except KeyboardInterrupt:
    print("% CTRL+C pressed")

  finally:
    print("% STOP ROBOT")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/ti","rc 0 0")
    service.send("robobot/cmd/T0","servo 1 3000 0")


if __name__ == "__main__":
    run()