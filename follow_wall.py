#!/usr/bin/env python
import numpy as np

def follow_wall(side,scan,wall_dist=0.5, w_max = 0.9, v_max=0.3):
    if scan is not None:
        # get distance ahead 
        distance_ahead_left = get_distance_in_sector(scan,
                                                     -np.pi,
                                                     -np.pi + np.pi/12) # 15 deg left
        distance_ahead_right = get_distance_in_sector(scan,
                                                      np.pi - np.pi/12,
                                                      np.pi) # 15 deg right
        distance_ahead = (distance_ahead_left + distance_ahead_right) / 2.0 # 30 deg front span

        # get distance to both sides
        if side == 'right':
            distance_to_right = get_distance_in_sector(scan,
                                                       np.pi/2,
                                                       np.pi/2 + np.pi/6)  # 30 deg span to the right
        elif side == 'left':
            distance_to_left = get_distance_in_sector(scan,
                                                      -np.pi/2 - np.pi/6,
                                                      -np.pi/2)  # 30 deg span to the left
        else:
            print('Invalid side')
            return 0,0

        #--------------------------------------------------------------
        # proportional gains for each control variable
        k_ang = 1.2
        k_dist = 0.75

        # get angle to align the robot to follow wall parallel
        error = find_wall_direction(side,scan)
        # get parallel distance to wall error -> desired - actual
        # sum both errors to control
        if side == 'right':
            error_dist = wall_dist - distance_to_right
            w = (error* k_ang) + (error_dist * k_dist)
        elif side == 'left':
            error_dist = wall_dist - distance_to_left
            w = ( (error* k_ang) + (error_dist * k_dist) ) *-1
        # max linear velocity to move fwd
        v = v_max
        
        # if there is something ahead (twice wall_dist) turn side
        # if there is no wall aside, turn around that corner
        if side == 'right':
            if distance_ahead <= wall_dist * 2:
                w = w_max
            elif np.isnan(error):
                w = -v/wall_dist
        elif side == 'left':
            if distance_ahead <= wall_dist * 2:
                w = -w_max
            elif np.isnan(error):
                w = v/wall_dist
        
        # debug print
        # if side == 'right':
        #     print('error: %.2f'%error,'distance to right: %.2f'%distance_to_right, 'distance ahead: %.2f'%distance_ahead)
        # elif side == 'left':
        #     print('error: %.2f'%error,'distance to left: %.2f'%distance_to_left, 'distance ahead: %.2f'%distance_ahead)

        # saturation
        z = np.clip(w, -w_max,w_max )
        x = v
        return z,x

def find_wall_direction(side,scan):
    """Assuming wall is on the right, finds the direction of the wall w.r.t
    the robot frame (x ahead, y to the left). The direction is returned
    as an angle, which is 0 if the wall is parallel to the heading of the
    robot and negative if the robot is heading away from the wall.
    """

    if side == 'right':
        # angle range span
        start_angle = np.pi/2 #- np.pi/12
        end_angle = np.pi/2 + np.pi/6
        # 30 deg sector
        alpha = abs(end_angle) - abs(start_angle)
        # get indexes
        start_index = range_index(scan,start_angle)
        end_index = range_index(scan,end_angle)
        # by trigonometry, obtain desired angle
        a = scan.ranges[start_index]
        b = scan.ranges[end_index]
        c = np.sqrt(a**2 +b**2 - (2 * (a*b) * np.cos(abs(alpha))))
        angle = np.arcsin((a - b * np.cos(alpha)) / c) # sin(angle) = d/c
    elif side == 'left':
        # angle range span
        start_angle = -np.pi/2 - np.pi/6
        end_angle = -np.pi/2
        # 30 deg sector
        alpha = abs(end_angle) - abs(start_angle)
        # get indexes
        start_index = range_index(scan,start_angle)
        end_index = range_index(scan,end_angle)
        # by trigonometry, obtain desired angle
        a = scan.ranges[end_index]
        b = scan.ranges[start_index]
        c = np.sqrt(a**2 +b**2 - (2 * (a*b) * np.cos(abs(alpha))))
        angle = np.arcsin((a - b * np.cos(alpha)) / c) # sin(angle) = d/c
    
    return angle


def get_distance_in_sector(scan, start_angle, end_angle) :
    """Returns the distance in m in the given sector by taking the average of the
    range scans.
    """

    # n total scans
    num_scans = len(scan.ranges)
    # get span indexes
    start_index = range_index(scan,start_angle)
    end_index = range_index(scan,end_angle)
    if end_index == num_scans-1:
        end_index = end_index + 1
    span = np.array(scan.ranges[start_index:end_index])
    n_infs = np.count_nonzero(span == np.inf)
    if n_infs == len(span):     # if all are inf
        return np.inf
    else:       # if not all are inf
        return np.mean(span[np.isfinite(span)])

def check_any_inf(scan,start_angle,end_angle):
    """Returns True if there is any inf in the ray span"""
    # n total scans
    num_scans = len(scan.ranges)
    # get span indexes
    start_index = range_index(scan,start_angle)
    end_index = range_index(scan,end_angle)
    if end_index == num_scans-1:
        end_index = end_index + 1
    span = np.array(scan.ranges[start_index:end_index])
    n_infs = np.count_nonzero(span == np.inf)
    if n_infs > 0:     # if there is at least 1 inf
        return True
    else:       # no inf
        return False

def range_index(scan, angle):
    """Returns the index into the scan ranges that correspond to the angle given (in rad).
    If the angle is out of range, then simply the first (0) or last index is returned, no
    exception is raised.
    """
    
    min_angle = scan.angle_min  # rad
    max_angle = scan.angle_max  # rad
    min_index = 0       # idx (int)
    max_index = (len(scan.ranges) - 1)  # idx (int)
	# remap angle to scan range
    index = int(np.round( ( (angle - min_angle) * (max_index - min_index) ) / ( (max_angle - min_angle) + min_index ) ) )
	# make sure to be inside limits
    return np.clip(index,min_index,max_index)
