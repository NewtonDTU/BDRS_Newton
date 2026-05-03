#import sys
#import threading
import time as t
#import select
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
from sedge import edge
from scripts.arucoDet import detectArucoMarkers
import cv2

ID_SIZE_DICTIONARY = {
    10: 0.1,
    11: 0.1,
    12: 0.1,
    13: 0.1,
    14: 0.1,
    15: 0.1,
    16: 0.1,
    17: 0.1,
    5: 0.035,
    20: 0.035,
    53: 0.035,
    25: 0.1
}

def shutdown_handler(sig, frame):
    global stop_program
    print("\nCtrl+C pressed! Stopping...")
    stop_program = True

def stop_robot(mode="center", settle_time=0.0):
  edge.lineControl(0, mode)
  service.send("robobot/cmd/ti", "rc 0 0")
  t.sleep(0.05)
  service.send("robobot/cmd/ti", "rc 0 0")
  if settle_time > 0:
    t.sleep(settle_time)

def sleep_robot(time=0.05):
  t.sleep(time)

def drive_until_line(speed=0.2, distance=0.0, recovery=False):
  print("Driving forward until line is detected...")
  target_distance = abs(distance)
  use_distance_limit = target_distance > 0.0
  line_found = False
  direction = "center"

  if use_distance_limit:
    pose.tripBreset()

  # Drive forward until line is found, or until distance limit (if provided).
  while edge.lineValidCnt <= 4 and not service.stop:
    service.send("robobot/cmd/ti", f"rc {speed:.3f} 0")
    t.sleep(0.05)

    if use_distance_limit and abs(pose.tripB) >= target_distance:
      print(f"Distance limit reached ({target_distance:.3f} m) without line detection")
      break

  if edge.lineValidCnt > 4:
    line_found = True
    print("Line detected")

  # stop
  service.send("robobot/cmd/ti", "rc 0 0")

  if line_found or service.stop:
    return (line_found, direction)

  if recovery and use_distance_limit:
    print("Recovery start: turn right 90 deg and search 0.2 m")
    driveTurn(np.pi / 2, -0.5)
    line_found, _ = drive_until_line(speed=0.2, distance=0.4, recovery=False)
    if line_found or service.stop:
      return (line_found, "left")

    print("Recovery step 2: turn 180 deg and search 0.4 m")
    driveTurn(np.pi, 0.5)
    line_found, _ = drive_until_line(speed=0.2, distance=0.8, recovery=False)
    return (line_found, "right")

  return (False, direction)

def drive_distance(distance=0.5, speed=0.2):
  state = 0
  target_distance = abs(distance)
  pose.tripBreset()
  print("% Driving distance -------------------------")

  while not (service.stop):

    if state == 0:
      # start driving forward
      service.send("robobot/cmd/ti",f"rc {speed} 0.0")
      state = 1

    elif state == 1:
      # stop when absolute traveled distance reaches target
      if abs(pose.tripB) >= target_distance or pose.tripBtimePassed() > 10:
        service.send("robobot/cmd/ti","rc 0.0 0.0")
        state = 2

    elif state == 2:
      # wait until the robot is completely stopped
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        service.send("robobot/cmd/ti","rc 0 0")
        state = 99

    else:
      print(f"# drive_to_wall reached {pose.tripB:.3f} m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0 0")
      t.sleep(0.2)
      service.send("robobot/cmd/ti","rc 0 0")
      break

    print(f"# drive_to_wall state {state}, distance {pose.tripB:.3f} m")
    t.sleep(0.05)

def driveTurn(angle=np.pi/4, angular_speed=0.5):
  state = 0
  pose.tripBreset()
  print(f"% Driving a {np.degrees(angle)} deg turn -------------------------")

  while not (service.stop):

    if state == 0:
      # initiate the turn
      service.send("robobot/cmd/ti",f"rc 0.0 {angular_speed}")
      state = 1

    elif state == 1:
      # check if the turn is completed
      if abs(pose.tripBh) > angle or pose.tripBtimePassed() > 10:
        service.send("robobot/cmd/ti","rc 0.0 0.0")
        state = 2

    elif state == 2:
      # wait until the robot is completely stopped
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        service.send("robobot/cmd/ti","rc 0 0")
        state = 99

    else:
      print(f"# turn45 turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0")
      break

    print(f"# turn45 state {state}, angle {pose.tripBh:.3f} rad")
    t.sleep(0.05)

def driveCircle(rotation=2*np.pi, v=0.2, w=0.6):
  #v       # forward velocity (fixed as required)
  #w       # angular velocity -> controls circle radius
  state = 0
  pose.tripBreset()
  print("% Driving a circle -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green

  while not (service.stop):

    if state == 0:  # start moving
      service.send("robobot/cmd/ti", f"rc {v} {w}")
      state = 1

    elif state == 1:  # keep moving until one full rotation
      if abs(pose.tripBh) >= rotation or pose.tripBtimePassed() > 30:
        service.send("robobot/cmd/ti","rc 0.0 0.0")
        state = 2

    elif state == 2:  # wait until robot stops
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99

    else:
      print(f"# circle completed {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0")
      break

    print(f"# state {state}, heading {pose.tripBh:.3f} rad, time {pose.tripBtimePassed():.3f}")
    t.sleep(0.05)

  print("% Driving a circle ------------------------- end")

def follow_line_for_distance(distance=1.0, velocity=0.15, mode="center", ref_position=0.0):
    """
    Follow line for a given traveled distance.

    mode:
        "left"   = follow left edge of white line
        "center" = follow center of white line
        "right"  = follow right edge of white line
    """
    edge.setup()
    
    # Start line controller
    edge.lineControl(velocity, mode=mode, refPosition=ref_position)
    
    # Wait until line is detected
    while edge.lineValidCnt < 3 and not service.stop:
        t.sleep(0.02)

    print("% Line detected, starting to follow")
    
    # Reset distance counter and follow for the requested distance
    pose.tripAreset()

    while not service.stop and abs(pose.tripA) < distance:
        t.sleep(0.02)

    stop_robot()
    t.sleep(0.3)

def follow_until_line_ends(velocity=0.15, mode="center"):
  print("% Following line until it ends")

  edge.setup()

  # start line controller
  edge.lineControl(velocity, mode=mode)

  # wait until line is detected
  while edge.lineValidCnt < 3 and not service.stop:
    t.sleep(0.02)

  print("% Line detected")

  # follow until the line disappears
  while not service.stop:
    if edge.lostLineCnt > 8:
      print("% Line ended")
      break

    t.sleep(0.02)

def count_active_line_sensors(active_threshold=None, use_dynamic_threshold=True, dynamic_ratio=0.45):
  """
  Return how many of the 8 line sensors are currently active.

  Dynamic mode (default):
    threshold = low + dynamic_ratio * (high - low)

  Absolute mode:
    A sensor is active when edge.edge_n[i] >= active_threshold.
    If active_threshold is None, edge.lineValidThreshold is used.
  """
  values = edge.edge_n[:]
  low = min(values)
  high = max(values)
  contrast = high - low

  if use_dynamic_threshold:
    threshold = low + dynamic_ratio * contrast
  else:
    if active_threshold is None:
      active_threshold = edge.lineValidThreshold
    threshold = active_threshold

  active_mask = [v >= threshold for v in values]
  active_indices = [i for i, is_active in enumerate(active_mask) if is_active]
  active_count = sum(active_mask)
  return active_count, active_mask, active_indices, values, threshold, contrast

def follow_line_until_intersection(
  velocity=0.15,
  mode="center",
  ref_position=0.0,
  active_threshold=None,
  min_active_sensors=6,
  min_sensor_span=6,
  min_crossing_count=2,
  hold_time=0.02,
  use_dynamic_threshold=True,
  dynamic_ratio=0.35,
):
  """
  Follow the line and stop when an intersection is detected.

  Intersection condition:
  - at least min_active_sensors are active at the same time
  - OR crossingLine is stable for min_crossing_count cycles
  - condition is stable for hold_time seconds
  """
  edge.setup()
  edge.lineControl(velocity, mode=mode, refPosition=ref_position)

  # Wait until line is detected before searching for intersection
  while edge.lineValidCnt < 3 and not service.stop:
    t.sleep(0.02)

  print("% Line detected, searching for intersection")

  active_since = None
  max_active_seen = 0
  loop_counter = 0
  while not service.stop:
    active_count, active_mask, active_indices, values, used_threshold, contrast = count_active_line_sensors(
      active_threshold=active_threshold,
      use_dynamic_threshold=use_dynamic_threshold,
      dynamic_ratio=dynamic_ratio,
    )
    loop_counter += 1

    sensor_span = 0
    if active_indices:
      sensor_span = active_indices[-1] - active_indices[0] + 1

    if active_count > max_active_seen:
      max_active_seen = active_count

    crossing_ready = edge.crossingLine and edge.crossingLineCnt >= min_crossing_count
    active_ready = active_count >= min_active_sensors and sensor_span >= min_sensor_span

    if edge.lineValid and (active_ready or crossing_ready):
      if active_since is None:
        active_since = t.time()
      elif (t.time() - active_since) >= hold_time:
        print(
          f"% Intersection detected: active={active_count}/8, "
          f"span={sensor_span}/8, cross={edge.crossingLineCnt}, avg={edge.average:.1f}, "
          f"thr={used_threshold:.1f}, contrast={contrast:.1f}, "
          f"values={values}, mask={active_mask}"
        )
        break
    else:
      active_since = None

    if loop_counter % 50 == 0:
      print(
        f"% intersection-check: active={active_count}/8, span={sensor_span}/8, max={max_active_seen}/8, "
        f"cross={edge.crossingLineCnt}, avg={edge.average:.1f}, "
        f"thr={used_threshold:.1f}, contrast={contrast:.1f}, values={values}"
      )

    t.sleep(0.01)

  stop_robot(mode=mode, settle_time=0.2)

def follow_line_until_tilt_back_normal(
    seconds=25,
    velocity=0.20,
    mode="center",
    ref_position=0.0,
    tilt_up_limit=0.12,
    tilt_down_limit=0.10,
    up_hold_time=0.15,
    down_hold_time=0.03,
    min_ramp_time_before_down_check=0.9,
    end_settle_time=0.3,
):
    edge.setup()
    edge.lineControl(velocity, mode=mode, refPosition=ref_position)

    # Wait until line is detected
    while edge.lineValidCnt < 4 and not service.stop:
        t.sleep(0.02)

    print("% Detection line, starting to follow", flush=True)

    start = t.time()
    baseline_tilt = pose.pose[3]
    
    # Write debug info to file
    with open("/tmp/ramp_debug.log", "w") as f:
        f.write(f"START: baseline_tilt={baseline_tilt:.4f}, tilt_up_limit={tilt_up_limit}, velocity={velocity}\n")
        f.flush()

    on_ramp = False
    tilt_up_start = None 
    tilt_down_start = None
    ramp_start_time = None
    debug_counter = 0

    while not service.stop and (t.time() - start < seconds):
        tilt_delta = abs(pose.pose[3] - baseline_tilt)
        debug_counter += 1
        
        # Write debug every ~1 second (100 iterations at 0.01s sleep)
        if debug_counter % 100 == 0:
            with open("/tmp/ramp_debug.log", "a") as f:
                f.write(f"LOOP: tilt={pose.pose[3]:.4f}, delta={tilt_delta:.4f}, on_ramp={on_ramp}, elapsed={t.time()-start:.1f}s\n")
                f.flush()

        if not on_ramp:
            if tilt_delta > tilt_up_limit:
                if tilt_up_start is None:
                    tilt_up_start = t.time()
                    with open("/tmp/ramp_debug.log", "a") as f:
                        f.write(f"RISE: delta={tilt_delta:.4f} > limit={tilt_up_limit}\n")
                        f.flush()
                elif (t.time() - tilt_up_start) >= up_hold_time:
                    print("% Ramp detected", flush=True)
                    with open("/tmp/ramp_debug.log", "a") as f:
                        f.write(f"DETECTED: Ramp entry confirmed\n")
                        f.flush()
                    on_ramp = True
                    ramp_start_time = t.time()
            else:
                tilt_up_start = None
        else:
            ramp_time_ok = (
                ramp_start_time is not None
                and (t.time() - ramp_start_time) >= min_ramp_time_before_down_check
            )

            if ramp_time_ok and tilt_delta < tilt_down_limit:
                if tilt_down_start is None:
                    tilt_down_start = t.time()
                elif (t.time() - tilt_down_start) >= down_hold_time:
                    with open("/tmp/ramp_debug.log", "a") as f:
                        f.write(f"EXIT: Leaving ramp\n")
                        f.flush()
                    break
            else:
                tilt_down_start = None

        t.sleep(0.01)

    # Stop line control and robot
    edge.lineControl(0, mode)
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)
    service.send("robobot/cmd/ti", "rc 0 0")
    
    if end_settle_time > 0:
        t.sleep(end_settle_time)

def rotate(target_angle, turn_speed=0.5):
    pose.tripBreset()
    state = 0

    while not service.stop:
        if state == 0:
            service.send("robobot/cmd/ti", f"rc 0.0 {turn_speed:.3f}")
            state = 1

        elif state == 1:
            print(f"tripBh={pose.tripBh:.3f}, target={target_angle:.3f}, time={pose.tripBtimePassed():.2f}")

            if abs(pose.tripBh) >= abs(target_angle):
                print("% rotate: reached target angle")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2

            elif pose.tripBtimePassed() > 15:
                print("% rotate: timeout")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2

        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                service.send("robobot/cmd/ti", "rc 0 0")
                break

        t.sleep(0.02)

def follow_line_until_angle(
  target_rad=np.pi/4,
  line_speed=0.10,
  mode="right",
  timeout=10.0,
  sample_time=0.02,
  use_abs_angle=True,
  stop_on_reach=True,
):
  """
  Follow the line while monitoring heading and stop/return when target angle is reached.
  Angles are in radians.

  Returns:
    (reached_target, current_rad)
  """
  pose.tripBreset()
  print("% Heading reset to 0")
  print(f"% Start line following, target={target_rad:.3f} rad ({np.rad2deg(target_rad):.1f} deg)")

  edge.lineControl(velocity=line_speed, mode=mode)
  start_time = t.time()

  while not service.stop:
    current_rad = pose.tripBh
    check_rad = abs(current_rad) if use_abs_angle else current_rad
    target_check = abs(target_rad) if use_abs_angle else target_rad

    print(f"% turn={current_rad:.3f} rad ({np.rad2deg(current_rad):.1f} deg)")

    if check_rad >= target_check:
      print(f"% Target angle reached: {current_rad:.3f} rad ({np.rad2deg(current_rad):.1f} deg)")
      if stop_on_reach:
        stop_robot(mode=mode, settle_time=0.1)
      return (True, current_rad)

    if timeout > 0 and (t.time() - start_time) >= timeout:
      print(
        f"% Timeout before reaching {target_rad:.3f} rad "
        f"({np.rad2deg(target_rad):.1f} deg) "
        f"(current={current_rad:.3f} rad / {np.rad2deg(current_rad):.1f} deg)"
      )
      if stop_on_reach:
        stop_robot(mode=mode, settle_time=0.1)
      return (False, current_rad)

    t.sleep(sample_time)

  if stop_on_reach:
    stop_robot(mode=mode, settle_time=0.1)
  return (False, pose.tripBh)

def follow_and_turn(line_speed=0.10, turn_speed=0.8, mode="right"):
  pose.tripBreset()
  print("% tripBh reset to 0")
  print("% Start line following")

  edge.lineControl(velocity=line_speed, mode=mode)

  state = 0

  while not service.stop:
    deg = np.rad2deg(pose.tripBh)
    print(f"% tripBh = {deg:.1f} deg")

    if state == 0:
      if deg >= 45:
        print("% Reached 45 deg")
        edge.lineControl(velocity=0, mode=mode)
        service.send("robobot/cmd/ti", "rc 0 0")
        t.sleep(0.2)

        pose.tripBreset()
        service.send("robobot/cmd/ti", f"rc 0 {-turn_speed:.3f}")
        state = 1

    elif state == 1:
      if pose.tripBh <= -np.deg2rad(92.5):
        print("% Finished 90 deg turn")
        service.send("robobot/cmd/ti", "rc 0 0")
        break

      t.sleep(0.05)

def rvec_tvec_to_matrix(rvec, tvec):

    R, _ = cv2.Rodrigues(rvec)

    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = tvec.flatten()

    return T

def find_aruco_markers(frame, expected_marker_id=0):

    # ---- Load camera calibration ----
    data = np.load("calibration.npz")
    camera_matrix = data["cameraMatrix"]
    dist_coeffs = data["distCoeffs"]

    marker_size = ID_SIZE_DICTIONARY.get(expected_marker_id, 0)

    theta_deg = -25
    theta = np.deg2rad(theta_deg)


    T_cam_ee = np.array([
        [1,     0,              0,          0.00],
        [0,np.cos(theta), -np.sin(theta),   0.19],
        [0,np.sin(theta),  np.cos(theta),   0.04],
        [0,     0,              0,            1]
        ])

    # frame = cv2.undistort(frame, camera_matrix, dist_coeffs) # Do NOT undistort passed to detectMarkers!

    detections = detectArucoMarkers(frame, camera_matrix, dist_coeffs, marker_size)

    if detections is None:
        #print("Expected marker ID {} not found in detections.".format(expected_marker_id))
        return None, frame

    detection = next((d for d in detections if d[3] == expected_marker_id), None)

    if detection is None:
        #print("Expected marker ID {} not found in detections.".format(expected_marker_id))
        return None, frame

    rvec, tvec, distance, marker_id, img_points = detection

    pts = img_points.astype(int)

    # Draw marker bounding box
    cv2.polylines(frame, [pts], True, (0,255,0), 2)

    # Draw coordinate axes
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

    #distance in end-effector frame

    T_marker_cam = rvec_tvec_to_matrix(rvec, tvec)
    T_marker_ee = T_cam_ee @ T_marker_cam
    marker_pos_ee = T_marker_ee[:3,3]
    distance = np.linalg.norm(marker_pos_ee)

    cv2.putText(
        frame,
        f"ID:{marker_id} Dist:{distance:.2f}m "
        f"X:{marker_pos_ee[0]:.2f} Y:{marker_pos_ee[1]:.2f} Z:{marker_pos_ee[2]:.2f}",
        (int(pts[0][0]-50), int(pts[0][1]) - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0,255,0),
        1
    )

    print("X:{:.2f}m Y:{:.2f}m Z:{:.2f}m".format(marker_pos_ee[0], marker_pos_ee[1], marker_pos_ee[2]))

    return marker_pos_ee, frame

def detect_luggages():
  cam = cv2.VideoCapture('http://10.197.216.163:7123/stream.mjpg')
  distance_prev = None
  distance_20 = 500
  distance_53 = 500
  debounce_arriving = 0
  while True:
    ret, frame = cam.read()
    if not ret:
      continue
    
    marker_pos_ee20, _ = find_aruco_markers(frame, expected_marker_id=20)
    marker_pos_ee53, _ = find_aruco_markers(frame, expected_marker_id=53)
    distance = 0
    if marker_pos_ee20 is not None or marker_pos_ee53 is not None:
      # It doesn't matter which one we select
      if marker_pos_ee20 is not None:
        distance_20 = marker_pos_ee20[2]

      if marker_pos_ee53 is not None:
        distance_53 = marker_pos_ee53[2]

      # Take the closest one
      if distance_20 < distance_53:
        distance = distance_20
      else:
        distance = distance_53
      
      if distance_prev is not None and distance_prev > distance:
        debounce_arriving += 1
      
      if debounce_arriving > 3:
        if distance > 0 and distance < 0.5:
          print(f"Distance: {distance}")
          break

    if distance > 0:
      distance_prev = distance

  cam.release()

def detect_closest_luggage():
  cam = cv2.VideoCapture('http://10.197.216.163:7123/stream.mjpg')
  distance = 0
  detected_marker_id = 0
  while True:
    ret, frame = cam.read()
    if not ret:
        continue
    
    marker_pos_ee20, _ = find_aruco_markers(frame, expected_marker_id=20)
    marker_pos_ee53, _ = find_aruco_markers(frame, expected_marker_id=53)
    distance_20 = 500 # just for it not to choose upon not detecting
    distance_53 = 500
    if marker_pos_ee20 is not None or marker_pos_ee53 is not None:
      if marker_pos_ee20 is not None:
        distance_20 = marker_pos_ee20[2]
      if marker_pos_ee53 is not None:
        distance_53 = marker_pos_ee53[2]
      
      if distance_20 < distance_53:
        detected_marker_id = 20
        distance = distance_20
      else:
        detected_marker_id = 53
        distance = distance_53

      break
  
  cam.release()
  return detected_marker_id, distance

def go_to_marker_id(marker_id, distance_sp = 0.4):
  print (f"Going to marker ID {marker_id}...")
  start_find_aruco = True
  find_aruco_state = 99
  Kp_turn = 0.6
  find_debouncer = 0
  lost_debouncer = 0
  found_marker = False
  lost_marker = False
  debounce_threshold = 2
  cam = cv2.VideoCapture('http://10.197.216.163:7123/stream.mjpg')
  while True:
    ret, frame = cam.read()
    if not ret:
      continue

    # detect marker
    print("Detecting marker...")
    marker_pos_ee, frame = find_aruco_markers(frame, expected_marker_id=marker_id)
    if marker_pos_ee is not None:
      marker_pos_ee[0] = marker_pos_ee[0] + 0.05 #offset
      if find_debouncer > debounce_threshold:
        turn_angle = -np.arctan2(marker_pos_ee[0], marker_pos_ee[1])
        distance_to_marker = marker_pos_ee[2]
        found_marker = True
        lost_marker = False
      lost_debouncer = 0
      find_debouncer += 1
      lost_marker = False
    else:
      if lost_debouncer > debounce_threshold:
        lost_marker = True
        found_marker = False
        turn_angle = 0
        distance_to_marker = 0
      lost_debouncer += 1
      find_debouncer = 0
      found_marker = False
      
    if start_find_aruco:

      print (f"State: {find_aruco_state}")

      if find_aruco_state == 0:
        pose.tripBreset()
        print(f"Turning robot by {np.rad2deg(turn_angle):.2f} degrees to face marker...")
        find_aruco_state = 1
      
      elif find_aruco_state == 1:
        if lost_marker == True:
          find_aruco_state = 99
        angular_speed = Kp_turn * turn_angle
        angular_speed = np.clip(angular_speed, -0.5, 0.5)
        service.send("robobot/cmd/ti", "rc 0.0 " + str(round(angular_speed, 2)))
        if abs(turn_angle) < np.deg2rad(2) or pose.tripBtimePassed() > 20:
          print(f"Triggered by angle {np.rad2deg(turn_angle):.2f} degrees or timeout {pose.tripBtimePassed():.2f} seconds")
          service.send("robobot/cmd/ti",f"rc 0.0 0.0")
          find_aruco_state = 2
      
      elif find_aruco_state == 2:
        pose.tripBreset()
        print(f"Driving towards marker, distance: {distance_to_marker:.2f} m")
        find_aruco_state = 3

      elif find_aruco_state == 3:
          
        if distance_to_marker < distance_sp:
          find_aruco_state = 4
          marker_close_time = t.time()
        else:
          speed = 0.1
        service.send("robobot/cmd/ti", "rc " + str(round(speed, 2)) + " 0.0")

      elif find_aruco_state == 4:
        speed = 0.1
        service.send("robobot/cmd/ti", "rc " + str(round(speed, 2)) + " 0.0")
        service.send("robobot/cmd/ti","rc 0.0 0.0")
        find_aruco_state = 5

      elif find_aruco_state == 5:
        angular_speed = Kp_turn * turn_angle
        angular_speed = np.clip(angular_speed, -0.4, 0.4)
        service.send("robobot/cmd/ti", "rc 0.0 " + str(round(angular_speed, 2)))
        if abs(turn_angle) < np.deg2rad(2) or pose.tripBtimePassed() > 20:
          print(f"Triggered by angle {np.rad2deg(turn_angle):.2f} degrees or timeout {pose.tripBtimePassed():.2f} seconds")
          service.send("robobot/cmd/ti",f"rc 0.0 0.0")
          find_aruco_state = 6
         
      elif find_aruco_state == 6:
        pose.tripBreset()
        print(f"Driving towards marker, distance: {distance_to_marker:.2f} m")
        find_aruco_state = 7

      elif find_aruco_state == 7:
        speed = 0.1
        service.send("robobot/cmd/ti", "rc " + str(round(speed, 2)) + " 0.0")
        if distance_to_marker < 0.3 or pose.tripBtimePassed() > 1 or lost_marker == True:
          print(f"Triggered by distance {distance_to_marker:.2f} m or timeout {pose.tripBtimePassed():.2f} seconds")
          service.send("robobot/cmd/ti",f"rc 0.0 0.0")
          find_aruco_state = 8

      elif find_aruco_state == 8:
        print("Reached marker!")
        find_aruco_state = 99
        start_find_aruco = False
        break

      elif find_aruco_state == 99:
        service.send("robobot/cmd/ti",f"rc 0.0 0.2")
        if found_marker and service.connected:
          service.send("robobot/cmd/ti",f"rc 0.0 0.0")
          print(f"Marker position in EE frame:\n{marker_pos_ee}")
          find_aruco_state = 0

  cam.release()