import numpy as np
from sensor_msgs.msg import LaserScan

CLIP_RADIUS = 160

CUT_CONV_SIZE = 3
POINT_BEST_CONV_SIZE = 80

MAX_LIDAR_DIST = 30.0
SPEED_MAX = 9.0
SPEED_STEP = 3.5

STEERING_ANGLE_STEP = np.pi / 12


def _longest_gap(ranges: np.ndarray) -> tuple[int, int]:
    masked = np.ma.masked_where(ranges == 0, ranges)
    slices = np.ma.notmasked_contiguous(masked)

    longest_slice = max(slices, key=lambda s: s.stop - s.start)
    return longest_slice.start, longest_slice.stop


def _find_best_point(start: int, end: int, ranges: np.ndarray) -> int:
    find_conv = np.full(POINT_BEST_CONV_SIZE, 1 / POINT_BEST_CONV_SIZE)
    averaged_max_gap = np.convolve(ranges[start:end], find_conv, "same")
    return averaged_max_gap.argmax() + start


def follow(scan: LaserScan) -> tuple[float, float]:
    # Cut values behind the car (more than 90 deg from front)
    cutoff = max(int((scan.angle_max - (np.pi / 2)) / scan.angle_increment), 0)
    ranges_cut = scan.ranges[cutoff:-cutoff]

    # Calculate running mean over LIDAR scan
    conv_pre = np.full(CUT_CONV_SIZE, 1 / CUT_CONV_SIZE)
    ranges_pre = np.convolve(ranges_cut, conv_pre, "same")
    ranges_pre = np.clip(ranges_pre, 0, MAX_LIDAR_DIST)

    # Get closest point
    point_close = ranges_pre.argmin()

    # Remove points in proximity to closest point
    minidx = max(point_close - CLIP_RADIUS, 0)
    maxidx = min(point_close + CLIP_RADIUS, len(ranges_pre) - 1)
    ranges_pre[minidx:maxidx] = 0.0

    # Calculate best point to
    gap_start, gap_end = _longest_gap(ranges_pre)
    point_best = _find_best_point(gap_start, gap_end, ranges_pre)

    # Calculate steering angle
    lidar_angle = (point_best - len(ranges_pre) // 2) * scan.angle_increment
    steering_angle = lidar_angle / 2

    # Calculate speed
    speed = SPEED_MAX - (SPEED_MAX - SPEED_STEP) * steering_angle**2 / (
        STEERING_ANGLE_STEP**2
    )
    speed = max(speed, 1.0)

    return speed, steering_angle
