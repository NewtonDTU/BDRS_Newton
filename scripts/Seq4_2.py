#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
import numpy as np

from scripts import utils




def run():
    try:
        if service.connected:
            utils.driveTurn(np.pi/2, -0.5)
            utils.drive_distance(0.2, 0.2)
            utils.drive_until_line(0.2)
            utils.follow_line_until_intersection(
            velocity=0.2,
            mode="center",
            active_threshold=450,
            min_active_sensors=6,
            min_sensor_span=6,
            hold_time=0.03,
            use_dynamic_threshold=False,
            )
            

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()