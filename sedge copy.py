#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */


#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#***************************************************************************/

from datetime import datetime
import time as t
from ulog import flog


class SEdge:
    # raw AD values
        # line mode: "left", "center", "right"
    followMode = "center"

    # filtered positions
    posLeftFilt = 0.0
    posRightFilt = 0.0
    linePosFilt = 0.0

    # remember last seen direction when line is lost
    lastError = 0.0

    # thresholds / shaping
    minContrast = 90
    edgeDetectThreshold = 120
    cornerGain = 2.2
    lostTurn = 0.85
    edge = [0, 0, 0, 0, 0, 0, 0, 0]
    edgeUpdCnt = 0
    edgeTime = datetime.now()
    edgeInterval = 0

    # white calibration values
    edge_n_w = [0, 0, 0, 0, 0, 0, 0, 0]
    edge_n_wUpdCnt = 0
    edge_n_wTime = datetime.now()

    # normalized values (0..1000)
    edge_n = [0, 0, 0, 0, 0, 0, 0, 0]
    edge_nUpdCnt = 0
    edge_nTime = datetime.now()
    edge_nInterval = 0
    edgeIntervalSetup = 0.1

    # thresholds
    lineValidThreshold = 700
    crossingThreshold = 850

    # line state
    posLeft = 0.0
    posRight = 0.0
    linePos = 0.0
    linePosRaw = 0.0
    followLeft = True
    refPosition = 0.0
    lineValid = False
    lineValidCnt = 0
    crossingLine = False
    crossingLineCnt = 0
    average = 0
    high = 0
    low = 0
    lostLineCnt = 0

    # smoothing / response
    linePosAlpha = 0.90     # high = fast response
    turnAlpha = 0.80       # 1.0 = no output smoothing

    # controller
    lineCtrl = False
    lineKp = 0.2
    lineTauZ = 0.9
    lineTauP = 0.04

    tauP2pT = 1.0
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0

    lineE1 = 0.0
    lineY1 = 0.0
    lineY = 0.0
    u = 0.0

    maxTurnRate = 6

    topicCmdT0 = ""
    sendCalibRequest = False
    velocity = 0.0

    def setup(self):
        from uservice import service

        sendBlack = False
        loops = 0

        print("% Edge (sedge.py):: turns on line sensor")
        self.topicCmdT0 = "robobot/cmd/T0"
        service.send(self.topicCmdT0, "lip 1")
        service.send(self.topicCmdT0, "sub livn 10")

        while not service.stop:
            t.sleep(0.02)

            if service.args.white:
                if not sendBlack:
                    sendBlack = service.send(self.topicCmdT0, "litb 0 0 0 0 0 0 0 0")
                elif self.edgeUpdCnt < 3:
                    service.send(self.topicCmdT0, "livi")
                elif not self.sendCalibRequest:
                    service.send(self.topicCmdT0, "liwi")
                    t.sleep(0.02)
                    service.send(self.topicCmdT0, "licw 100")
                    print("# Edge (sedge.py):: sending calibration request")
                    t.sleep(0.25)
                    service.send(self.topicCmdT0, "eew")
                    self.sendCalibRequest = True
                    service.send(self.topicCmdT0, "liwi")
                    t.sleep(0.02)
                else:
                    t.sleep(0.25)
                    service.args.white = False
                    print("% Edge (sedge.py):: calibration should be fine, terminates.")
                    service.stop = True
            elif self.edge_n_wUpdCnt == 0:
                service.send(self.topicCmdT0, "liwi")
            elif self.edge_nUpdCnt == 0:
                pass
            else:
                print(f"% Edge (sedge.py):: got data stream; after {loops} loops")
                break

            loops += 1
            if loops > 30:
                print(f"% Edge (sedge.py):: got no data after {loops}")
                break

    def decode(self, topic, msg):
        used = True

        if topic == "T0/liv":
            gg = msg.split(" ")
            if len(gg) >= 9:
                t0 = self.edgeTime
                self.edgeTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    value = int(gg[i + 1])
                    # Filter out invalid sensor values (> 1000 are clearly spurious)
                    self.edge[i] = value if value <= 1000 else 0

                t1 = self.edgeTime
                if self.edgeUpdCnt == 2:
                    self.edgeInterval = (t1 - t0).total_seconds() * 1000
                elif self.edgeUpdCnt > 2:
                    self.edgeInterval = (
                        self.edgeInterval * 99 + (t1 - t0).total_seconds() * 1000
                    ) / 100
                self.edgeUpdCnt += 1

        elif topic == "T0/livn":
            gg = msg.split(" ")
            if len(gg) >= 9:
                t0 = self.edge_nTime
                self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    value = int(gg[i + 1])
                    # Filter out invalid sensor values (> 1000 are clearly spurious)
                    self.edge_n[i] = value if value <= 1000 else 0

                t1 = self.edge_nTime
                if self.edge_nUpdCnt == 2:
                    self.edge_nInterval = (t1 - t0).total_seconds() * 1000
                    self.PIDrecalculate()
                elif self.edge_nUpdCnt > 2:
                    self.edge_nInterval = (
                        self.edge_nInterval * 99 + (t1 - t0).total_seconds() * 1000
                    ) / 100

                self.edge_nUpdCnt += 1

                self.LineDetect()

                if self.lineCtrl:
                    self.followLine()

                if self.edge_nUpdCnt % 10 == 0:
                    flog.write()

        elif topic == "T0/liw":
            gg = msg.split(" ")
            if len(gg) >= 9:
                self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    value = int(gg[i + 1])
                    # Filter out invalid sensor values (> 1000 are clearly spurious)
                    self.edge_n_w[i] = value if value <= 1000 else 0
                self.edge_n_wUpdCnt += 1
        else:
            used = False

        return used

    
    def LineDetect(self):
        values = self.edge_n[:]

        self.high = max(values)
        self.low = min(values)
        self.average = sum(values) / 8.0
        contrast = self.high - self.low

        # strong white area -> possible crossing / wide line
        self.crossingLine = self.detectCrossPath()
        # reject weak detections
        self.lineValid = contrast >= self.minContrast

        # sensor positions:
        # positive = line more to left side of robot
        # negative = line more to right side of robot
        sensor_pos = [3.5, 2.5, 1.5, 0.5, -0.5, -1.5, -2.5, -3.5]

        if not self.lineValid:
            self.lineValidCnt = max(0, self.lineValidCnt - 1)
            self.lostLineCnt += 1

            if self.crossingLine:
                self.crossingLineCnt = min(20, self.crossingLineCnt + 1)
            else:
                self.crossingLineCnt = max(0, self.crossingLineCnt - 1)

            print(
                f"% LineDetect: sensors={values}, contrast={contrast:.1f}, "
                f"valid=False"
            )
            return

        # Make brightness relative to floor
        bright = [max(0.0, v - self.low) for v in values]

        # Dynamic threshold to define the white "blob"
        blob_threshold = max(self.edgeDetectThreshold, 0.45 * contrast)

        active = [b >= blob_threshold for b in bright]

        # fallback if threshold is too hard
        if not any(active):
            active = [b >= 0.30 * contrast for b in bright]

        idx = [i for i, a in enumerate(active) if a]

        if len(idx) == 0:
            self.lineValid = False
            self.lineValidCnt = max(0, self.lineValidCnt - 1)
            self.lostLineCnt += 1
            print(
                f"% LineDetect: sensors={values}, contrast={contrast:.1f}, no blob"
            )
            return

        # leftmost and rightmost detected sensor indices on the white line
        i_left = idx[0]
        i_right = idx[-1]

        # interpolate left edge
        if i_left > 0:
            a = bright[i_left - 1]
            b = bright[i_left]
            frac_left = 0.0 if abs(b - a) < 1e-6 else (blob_threshold - a) / (b - a)
            frac_left = max(0.0, min(1.0, frac_left))
            pos_left_raw = sensor_pos[i_left - 1] + frac_left * (sensor_pos[i_left] - sensor_pos[i_left - 1])
        else:
            pos_left_raw = sensor_pos[i_left]

        # interpolate right edge
        if i_right < 7:
            a = bright[i_right]
            b = bright[i_right + 1]
            frac_right = 0.0 if abs(a - b) < 1e-6 else (a - blob_threshold) / (a - b)
            frac_right = max(0.0, min(1.0, frac_right))
            pos_right_raw = sensor_pos[i_right] + frac_right * (sensor_pos[i_right + 1] - sensor_pos[i_right])
        else:
            pos_right_raw = sensor_pos[i_right]

        # since sensor_pos goes from left to right as 3.5 -> -3.5,
        # the numerically larger value is the physical left side
        self.posLeft = max(pos_left_raw, pos_right_raw)
        self.posRight = min(pos_left_raw, pos_right_raw)
        self.linePosRaw = 0.5 * (self.posLeft + self.posRight)

        # adaptive smoothing:
        # straight sections -> more smoothing
        # corners / fast changes -> less smoothing
        delta = abs(self.linePosRaw - self.linePosFilt)
        alpha_pos = self.linePosAlpha + 0.35 * min(1.0, delta / 1.5)
        alpha_pos = max(0.12, min(0.65, alpha_pos))

        edge_sep = abs(self.posLeft - self.posRight)
        alpha_edge = max(0.15, min(0.55, alpha_pos + 0.05))

        self.posLeftFilt = (1.0 - alpha_edge) * self.posLeftFilt + alpha_edge * self.posLeft
        self.posRightFilt = (1.0 - alpha_edge) * self.posRightFilt + alpha_edge * self.posRight
        self.linePosFilt = (1.0 - alpha_pos) * self.linePosFilt + alpha_pos * self.linePosRaw

        self.posLeft = self.posLeftFilt
        self.posRight = self.posRightFilt
        self.linePos = self.linePosFilt

        self.lineValidCnt = min(20, self.lineValidCnt + 1)
        self.lostLineCnt = 0

        if self.crossingLine:
            self.crossingLineCnt = min(20, self.crossingLineCnt + 1)
        else:
            self.crossingLineCnt = max(0, self.crossingLineCnt - 1)

        print(
            f"% LineDetect: sensors={values}, "
            f"L={self.posLeft:.2f}, C={self.linePos:.2f}, R={self.posRight:.2f}, "
            f"contrast={contrast:.1f}, valid={self.lineValid}, crossing={self.crossingLine}"
        )

    def lineControl(self, velocity, mode="center", refPosition=0.0):
        self.velocity = velocity
        self.followMode = mode  # "left", "center", "right"
        self.refPosition = refPosition
        self.lineCtrl = velocity > 0.001

        if self.lineCtrl:
            self.lineE1 = 0.0
            self.lineY1 = 0.0
            self.lineY = 0.0
            self.lastError = 0.0

    def followLine(self):
        from uservice import service

        # If line is lost, keep turning a little in last known direction
        if self.lineValidCnt < 1:
            search_turn = self.lostTurn if self.lastError >= 0 else -self.lostTurn
            par = f"rc {self.velocity:.3f} {search_turn:.3f} {t.time()}"
            service.send("robobot/cmd/ti", par)
            print(f"% followLine: LOST LINE, search_turn={search_turn:.2f}")
            return

        # choose tracking target
        if self.followMode == "left":
            target_pos = self.posLeft
        elif self.followMode == "right":
            target_pos = self.posRight
        else:
            target_pos = self.linePos

        e = target_pos - self.refPosition

        # react harder in corners
        # big error = probably sharp turn or branch entry
        gain = 1.0 + (self.cornerGain - 1.0) * min(1.0, abs(e) / 2.0)
        self.u = self.lineKp * gain * e

        # lead/lag controller from your existing code
        y_new = (
            self.u * self.tauZ2pT
            - self.lineE1 * self.tauZ2mT
            + self.lineY1 * self.tauP2mT
        ) / self.tauP2pT

        # output smoothing:
        # small changes are smoothed a lot
        # big corrections go through faster
        dy = abs(y_new - self.lineY1)
        alpha_turn = self.turnAlpha + 0.45 * min(1.0, dy / self.maxTurnRate)
        alpha_turn = max(0.15, min(0.85, alpha_turn))

        self.lineY = (1.0 - alpha_turn) * self.lineY1 + alpha_turn * y_new

        if self.lineY > self.maxTurnRate:
            self.lineY = self.maxTurnRate
        elif self.lineY < -self.maxTurnRate:
            self.lineY = -self.maxTurnRate

        self.lineE1 = self.u
        self.lineY1 = self.lineY
        self.lastError = e

        par = f"rc {self.velocity:.3f} {self.lineY:.3f} {t.time()}"
        service.send("robobot/cmd/ti", par)

        print(
            f"% followLine: mode={self.followMode}, "
            f"L={self.posLeft:.2f}, C={self.linePos:.2f}, R={self.posRight:.2f}, "
            f"e={e:.2f}, y={self.lineY:.2f}"
        )

    def PIDrecalculate(self):
        Tsec = max(0.001, self.edge_nInterval / 1000.0)
        self.tauP2pT = self.lineTauP * 2.0 + Tsec
        self.tauP2mT = self.lineTauP * 2.0 - Tsec
        self.tauZ2pT = self.lineTauZ * 2.0 + Tsec
        self.tauZ2mT = self.lineTauZ * 2.0 - Tsec

        print(
            f"%% Lead: tauZ={self.lineTauZ:.3f}, "
            f"tauP={self.lineTauP:.3f}, T={self.edge_nInterval:.3f} ms"
        )

    def detectCrossPath(self):
        values = self.edge_n[:]

        # how many sensors clearly see white
        white_threshold = 780
        white_count = sum(1 for v in values if v >= white_threshold)

        # wide white area across the sensor bar
        wide_white = white_count >= 6

        # also require the average to be high
        avg_white = (sum(values) / 8.0) >= self.crossingThreshold

        # optional: require center sensors to also be white
        center_white = values[3] >= white_threshold and values[4] >= white_threshold

        # final decision
        cross_now = wide_white and avg_white and center_white

        if cross_now:
            self.crossingLineCnt = min(20, self.crossingLineCnt + 1)
        else:
            self.crossingLineCnt = max(0, self.crossingLineCnt - 1)

        # require it for a few frames to avoid false triggers
        return self.crossingLineCnt >= 3
    
    def terminate(self):
        from uservice import service
        print("% Edge (sedge.py):: turn off line sensor")
        service.send(self.topicCmdT0, "lip 0")
        print("% Edge (sedge.py):: terminated")


edge = SEdge()