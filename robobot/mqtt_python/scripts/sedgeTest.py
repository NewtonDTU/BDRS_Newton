#!/usr/bin/env python3
import time as t
from setproctitle import setproctitle

from sedge import edge
from spose import pose
from uservice import service
from scripts import utils


def stop_robot():
    edge.lineControl(0, mode="center", refPosition=0.0)
    for _ in range(10):
        service.send("robobot/cmd/ti", "rc 0 0")
        t.sleep(0.05)



if __name__ == "__main__":
    setproctitle("line-follow-test")

    service.setup("localhost")

    if service.connected:
        pose.setup()
        edge.setup()

        # --- tuning ---
        #edge.linePosAlpha = 0.90
        #edge.turnAlpha = 0.80 #5
        #edge.lineKp = 0.2 #70
        #edge.lineTauZ = 0.9 #95 good 
        #edge.maxTurnRate = 6
        
        

        #edge.minContrast = 90
        #edge.edgeDetectThreshold = 120
        #edge.cornerGain = 2.2
        #edge.lostTurn = 0.85

        edge.PIDrecalculate()

        # --- choose ONE of these tests ---

        # normal center follow
        utils.follow_line_for_distance(distance=50.0, velocity=0.20, mode="center")

        # left-edge follow
        # utils.follow_line_for_distance(distance=2.0, velocity=0.22, mode="left")

        # right-edge follow
        # utils.follow_line_for_distance(distance=2.0, velocity=0.22, mode="right")

        stop_robot()

    service.terminate()