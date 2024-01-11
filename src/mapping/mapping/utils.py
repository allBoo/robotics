

def min_el_idx(els: list[float], from_idx: int, to_idx: int) -> tuple[float, int]:
    min_el = min_idx = None

    for idx in range(from_idx, to_idx + 1):
        el = els[idx]
        if min_el is None or el < min_el:
            min_el = el
            min_idx = idx

    return min_el, min_idx


def lidar_idx_to_angle(idx: int) -> int:
    """
    calculate relative angle from the center of the robot
    so that left angles are negative from 0 to 180 and right - positive from 0 to 180
    :param idx:
    :return:
    """
    if 0 <= idx <= 180:
        return -idx
    elif -180 <= idx <= 0:
        return -idx
    elif -360 <= idx <= -180:
        return 360--idx
    else:
        return 360 - idx
