from __future__ import annotations
import numpy as np


class Drone:
    def __init__(self, drone_id: int, start, end, speed: float = 15.0, priority: int | None = None):
        self.id = drone_id
        self.start = np.asarray(start, dtype=float)
        self.end = np.asarray(end, dtype=float)
        self.base_speed = float(speed)
        self.priority = priority if priority is not None else drone_id

        self.position = self.start.copy()
        self.current_speed = self.base_speed
        self.arrived = False
        self.toa = None

        path = self.end - self.start
        self.total_distance = float(np.linalg.norm(path))

        if self.total_distance > 0:
            self.direction = path / self.total_distance
        else:
            self.direction = np.zeros(2)

        self.position_history = [self.position.copy()]
        self.speed_history = [self.base_speed]
        self.time_history = [0.0]

    def distance_to_end(self) -> float:
        return float(np.linalg.norm(self.end - self.position))

    def position_at(self, t: float) -> np.ndarray:
        distance_covered = self.base_speed * t
        if distance_covered >= self.total_distance:
            return self.end.copy()
        return self.start + self.direction * distance_covered

    def step(self, dt: float, current_time: float) -> None:
        if self.arrived:
            return

        remaining_distance = self.distance_to_end()
        step_distance = self.current_speed * dt

        if step_distance >= remaining_distance:
            self.position = self.end.copy()
            self.arrived = True
            self.toa = current_time + dt
        else:
            self.position += self.direction * step_distance

        self.position_history.append(self.position.copy())
        self.speed_history.append(self.current_speed)
        self.time_history.append(current_time + dt)

    def decelerate(self, factor: float = 0.5) -> None:
        self.current_speed = max(self.base_speed * factor, 0.0)

    def resume_speed(self) -> None:
        self.current_speed = self.base_speed

    def __repr__(self) -> str:
        return (
            f"Drone(id={self.id}, "
            f"pos={np.round(self.position, 1)}, "
            f"speed={self.current_speed:.1f}, "
            f"arrived={self.arrived})"
        )