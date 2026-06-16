from __future__ import annotations
from dataclasses import dataclass
import numpy as np

MIN_SEPARATION = 20.0


@dataclass
class Conflict:
    drone_a: int
    drone_b: int
    time: float
    position_a: np.ndarray
    position_b: np.ndarray
    distance: float
    resolved: bool = False

    def __repr__(self) -> str:
        return (
            f"Conflict(drones=({self.drone_a}, {self.drone_b}), "
            f"t={self.time:.2f}s, "
            f"dist={self.distance:.1f}m, "
            f"resolved={self.resolved})"
        )


def predict_conflict(drones, min_separation: float = MIN_SEPARATION):
    predicted = []
    n_drones = len(drones)

    for i in range(n_drones):
        for j in range(i + 1, n_drones):
            drone_a = drones[i]
            drone_b = drones[j]

            velocity_a = drone_a.direction * drone_a.base_speed
            velocity_b = drone_b.direction * drone_b.base_speed

            relative_velocity = velocity_a - velocity_b
            relative_position = drone_a.start - drone_b.start

            a_coef = np.dot(relative_velocity, relative_velocity)
            b_coef = 2.0 * np.dot(relative_position, relative_velocity)

            flight_time_a = (drone_a.total_distance / drone_a.base_speed)
            flight_time_b = (drone_b.total_distance / drone_b.base_speed)

            max_time = min(flight_time_a, flight_time_b)

            if a_coef < 1e-10:
                t_closest = 0.0
            else:
                t_closest = -b_coef / (2.0 * a_coef)
                t_closest = float(np.clip(t_closest, 0.0, max_time))

            pos_a = drone_a.position_at(t_closest)
            pos_b = drone_b.position_at(t_closest)

            min_distance = float(np.linalg.norm(pos_a - pos_b))

            if min_distance < min_separation:
                predicted.append(
                    {
                        "drone_a": drone_a.id,
                        "drone_b": drone_b.id,
                        "t_closest": t_closest,
                        "pos_a": pos_a,
                        "pos_b": pos_b,
                        "min_dist": min_distance,
                    }
                )
    return predicted


def check_separation(drones, current_time: float, min_separation: float = MIN_SEPARATION):
    active_drones = [drone for drone in drones if not drone.arrived]
    conflicts = []

    for i in range(len(active_drones)):
        for j in range(i + 1, len(active_drones)):
            drone_a = active_drones[i]
            drone_b = active_drones[j]

            separation = float(np.linalg.norm(drone_a.position - drone_b.position))

            if separation < min_separation:
                conflicts.append(
                    Conflict(
                        drone_a=drone_a.id,
                        drone_b=drone_b.id,
                        time=current_time,
                        position_a=drone_a.position.copy(),
                        position_b=drone_b.position.copy(),
                        distance=separation,
                    )
                )
    return conflicts
