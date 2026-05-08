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
        print("% Seq4_1 STARTED - about to turn and detect ramp", flush=True)
        if service.connected:
            utils.driveTurn(np.pi/12, 0.5)
            utils.follow_line_until_tilt_back_normal(
                seconds=25,
                velocity=0.4,
                mode="left",
                ref_position=0.0,
                tilt_up_limit=0.12,
                tilt_down_limit=0.09,
                up_hold_time=0.15,
                down_hold_time=0.15,
                min_ramp_time_before_down_check=1.0,
                end_settle_time=0.05,
            )
            

    except KeyboardInterrupt:
        print("% CTRL+C pressed")

    finally:
        print("% STOP ROBOT")
        service.send("robobot/cmd/ti","rc 0 0")
        service.send("robobot/cmd/ti","rc 0 0")

if __name__ == "__main__":
    run()