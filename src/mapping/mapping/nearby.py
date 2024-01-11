import math
from dataclasses import dataclass, field

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher

from abc import ABC, abstractmethod
from typing import Callable

from .utils import min_el_idx, lidar_idx_to_angle


@dataclass
class Correction:
    value: float = None
    error: float = 0.0
    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    c: float = field(init=False)

    def __post_init__(self):
        self.c = self.p + self.i + self.d


@dataclass
class Response:
    distance: Correction
    angle: Correction


class EmptyResponse(Response):
    def __init__(self):
        super().__init__(Correction(), Correction())


class TargetMachine(ABC):
    STATE_IDLE = 0
    STATE_MOVE = 1
    STATE_DONE = 2

    COEFF_P = 0.3
    COEFF_D = 0.1
    COEFF_I = 25.0

    def __init__(self, target_distance: float, target_angle: float):
        self.target_distance = target_distance
        self.target_angle = target_angle

        self.distance = None
        self.angle = None

        self.state = self.STATE_IDLE

        self.angle_correction = Correction()
        self.distance_correction = Correction()

    @abstractmethod
    def update(self, lidar_ranges: list[float]):
        pass

    def done(self):
        self.angle_correction = Correction()
        self.distance_correction = Correction()
        self.state = self.STATE_DONE

    def is_done(self):
        return self.state == self.STATE_DONE

    def move(self):
        if self.is_done():
            return
        self.state = self.STATE_MOVE

        if self.angle is not None:
            self.angle_correction = self.get_correction(
                math.radians(self.angle),
                math.radians(self.target_angle),
                self.angle_correction
            )

        if self.distance is not None:
            self.distance_correction = self.get_correction(
                self.distance,
                self.target_distance,
                self.distance_correction
            )

    def _dt(self) -> float:
        return 1.0

    def get_correction(self, value: float, target_value: float, prev_correction: Correction) -> Correction:
        dt = self._dt()

        error = target_value - value

        p = self.COEFF_P * error
        i = prev_correction.i + (self.COEFF_P / self.COEFF_I) * error * dt
        d = -self.COEFF_P * self.COEFF_D * (value - (prev_correction.value or value)) / dt

        # reset integral when error changed its sign
        if (error >= 0) != (prev_correction.error >= 0):
            i = 0.0

        return Correction(value, error, p, i, d)


class MoveCloseTarget(TargetMachine):

    def __init__(self, target_distance: float):
        super().__init__(target_distance, 0.0)

    def __str__(self):
        return f'MoveCloseTarget {self.target_distance}'

    def update(self, lidar_ranges: list[float]):
        if self.is_done():
            return

        # step 1 - turn on the 90 degrees to the wall
        # 1 find the closer point around - its idx is a relative angle (in degrees) to the wall
        # 1.1 the goal is to turn so that the closer point had 0 idx
        if self.distance is None:
            if self.angle is None:
                # for the first time search the wall from the right side
                distance, lidar_idx = min_el_idx(lidar_ranges, 225, 359)
            else:
                # after original angle found - look forward
                distance, lidar_idx = min_el_idx(lidar_ranges, -45, 45)

            self.angle = lidar_idx_to_angle(lidar_idx)
            if -10 <= self.angle <= 10:
                # target angle reached, move forward
                self.distance = distance
        else:
            self.distance, lidar_idx = min_el_idx(lidar_ranges, -15, 15)
            self.angle = lidar_idx_to_angle(lidar_idx)

        # 2. Move forward or backward while distance to the wall is not reached the target
        # keep 0 angle to the wall
        if self.distance is not None and self.distance <= self.target_distance:
            # target distance reached
            self.done()
        else:
            # calculate corrections
            self.move()


class TurnTarget(TargetMachine):

    def __init__(self, target_angle: float, threshold: float = 1.0):
        super().__init__(0.0, target_angle)
        self.threshold = math.radians(threshold)

    def __str__(self):
        return f'TurnTarget {self.target_angle}'

    def update(self, lidar_ranges: list[float]):
        if self.is_done():
            return

        _, lidar_idx = min_el_idx(lidar_ranges, -105, 15)
        self.angle = lidar_idx_to_angle(lidar_idx)

        if self.angle is not None and abs(self.angle - self.target_angle) <= self.threshold:
            # target distance reached
            self.done()
        else:
            # calculate corrections
            self.move()


class MoveForwardTarget(TargetMachine):

    def __init__(self, target_distance: float):
        super().__init__(target_distance, 90.0)

    def __str__(self):
        return f'MoveForwardTarget {self.target_distance} {self.target_angle}'

    def update(self, lidar_ranges: list[float]):
        if self.is_done():
            return

        # 1. find the right angle
        distance, lidar_idx = min_el_idx(lidar_ranges, 225, 315)
        self.angle = lidar_idx_to_angle(lidar_idx)

        # 2. find the wall in front of view
        self.distance = lidar_ranges[0]
        if self.distance > self.target_distance * 2:
            self.distance = self.target_distance * 2

        # 2. Move forward while wall didn't reach and keep 90 angle to the right wall
        if self.distance <= self.target_distance:
            # target distance reached
            self.done()
        else:
            # calculate corrections
            self.move()


class NearbyController:

    STATE_IDLE = 0
    STATE_PREPARE = 1
    STATE_READY = 2
    STATE_MOVE = 3
    STATE_DONE = 4

    STATE_MAP = {
        STATE_IDLE: STATE_PREPARE,
        STATE_PREPARE: STATE_READY,
        STATE_READY: STATE_MOVE,
        STATE_MOVE: STATE_DONE
    }

    def __init__(
            self,
            target_distance: float,
            logger: RcutilsLogger,
            callback: Callable
    ):
        self.state = self.STATE_IDLE
        self.target_distance = target_distance
        self.logger = logger
        self.callback = callback

        self.lidar_ranges = []
        self.target = None

    def get_state_action_map(self):
        return {
            self.STATE_IDLE: self.on_idle,
            self.STATE_PREPARE: self.on_prepare,
            self.STATE_READY: self.on_ready,
            self.STATE_MOVE: self.on_move,
            self.STATE_DONE: self.on_idle,
        }

    def next_state(self):
        if self.state in self.STATE_MAP:
            return self.STATE_MAP[self.state]
        else:
            raise RuntimeError('Nearby Controller is ended')

    def go(self):
        self.state = self.next_state()

    def stop(self):
        self.state = self.STATE_IDLE
        self.logger.info('Stop Nearby Controller')

    def update(self, ranges: list[float]):
        if self.state == self.STATE_DONE:
            return

        self.lidar_ranges = ranges

        action = self.get_state_action_map()[self.state]
        action()

        if self.target:
            self.logger.info(f'State {self.target} / d={self.target.distance}, a={self.target.angle}, s={self.target.state}')

            if self.target.is_done():
                self.logger.info(f'Target {self.target} reached')
                self.state = self.next_state()

        self._send_response()

    def _send_response(self):
        if self.target:
            self.callback(Response(self.target.distance_correction, self.target.angle_correction))
        else:
            self.callback(EmptyResponse())

    def on_idle(self):
        pass

    def on_prepare(self):
        # the goal is to move closer to the nearest right wall
        if not isinstance(self.target, MoveCloseTarget):
            self.target = MoveCloseTarget(self.target_distance)

        self.target.update(self.lidar_ranges)

    def on_ready(self):
        # the goal is to turn left on 90 degrees to the wall
        if not isinstance(self.target, TurnTarget):
            self.target = TurnTarget(90.0)

        self.target.update(self.lidar_ranges)

    def on_move(self):
        # the goal is to move forward till the front wall reached, and keep the angle 90 degrees to the right wall
        if not isinstance(self.target, MoveForwardTarget):
            self.target = MoveForwardTarget(self.target_distance)

        self.target.update(self.lidar_ranges)
