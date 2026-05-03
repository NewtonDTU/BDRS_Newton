#!/usr/bin/env python3

import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service


def stop_robot():
    edge.lineControl(0, True)
    for _ in range(10):
        service.send("robobot/cmd/ti", "rc 0 0")
        t.sleep(0.05)

def rotate_ccw_67_5():
    target_angle = 1.75
    #target_angle = 3.0 * 3.141592653589793 / 8.0  # 67.5 deg

    edge.lineControl(0, True)
    t.sleep(0.1)

    pose.tripBreset()
    state = 0

    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", "rc 0.0 0.5")
            state = 1

        elif state == 1:
            if pose.tripBh >= target_angle or pose.tripBtimePassed() > 5:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2

        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                service.send("robobot/cmd/ti", "rc 0 0")
                break

        t.sleep(0.02)


def follow_line_until_tilt_back_normal(
    seconds=25,
    velocity=0.20,
    follow_left=False,
    tilt_up_limit=0.12,
    tilt_down_limit=0.05,
    up_hold_time=0.20,
    down_hold_time=0.20
):
    edge.lineControl(velocity, follow_left)

    start = t.time()
    baseline_tilt = pose.pose[3]

    on_ramp = False
    tilt_up_start = None
    tilt_down_start = None

    while not service.stop and (t.time() - start < seconds):
        tilt_delta = abs(pose.pose[3] - baseline_tilt)

        if not on_ramp:
            if tilt_delta > tilt_up_limit:
                if tilt_up_start is None:
                    tilt_up_start = t.time()
                elif (t.time() - tilt_up_start) >= up_hold_time:
                    print(True)
                    on_ramp = True
            else:
                tilt_up_start = None
        else:
            if tilt_delta < tilt_down_limit:
                if tilt_down_start is None:
                    tilt_down_start = t.time()
                elif (t.time() - tilt_down_start) >= down_hold_time:
                    break
            else:
                tilt_down_start = None

        t.sleep(0.05)

    stop_robot()


if __name__ == "__main__":
    setproctitle("sequence-3")

    service.setup("localhost")

    if service.connected:
        edge.setup()
        edge.lineKp = 0.3
        edge.lineTauz = 1.0
        edge.lineTaup = 0.35

        follow_line_until_tilt_back_normal(
            seconds=25,
            velocity=0.20,
            follow_left=False,
            tilt_up_limit=0.12,
            tilt_down_limit=0.10,
            up_hold_time=0.15,
            down_hold_time=0.03
        )

        rotate_ccw_67_5()

        stop_robot()

        

    service.terminate()