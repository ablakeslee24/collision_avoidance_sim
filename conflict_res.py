from __future__ import annotations
from typing import Dict, List
import numpy as np
from conflict_detect import MIN_SEPARATION, check_separation

SLOW_FACTOR = 0.3
RESUME_MARGIN = MIN_SEPARATION * 1.5


class Resolution:
    def __init__(self, slow_factor: float = SLOW_FACTOR, resume_margin: float = RESUME_MARGIN,
                 min_separation: float = MIN_SEPARATION):
        self.slow_factor = slow_factor
        self.resume_margin = resume_margin
        self.min_separation = min_separation

        self.yielding: Dict[int, int] = {}
        self.resolution_log: List[dict] = []

    def resolve(self, drones, current_time: float):
        drone_map = {d.id: d for d in drones}
        active_ids = {d.id for d in drones if not d.arrived}

        conflicts = check_separation(drones, current_time, self.min_separation)

        for conflict in conflicts:
            a = drone_map[conflict.drone_a]
            b = drone_map[conflict.drone_b]

            if a.priority <= b.priority:
                winner, yielder = a, b
            else:
                winner, yielder = b, a

            if yielder.id not in self.yielding:
                yielder.slow_down(self.slow_factor)
                self.yielding[yielder.id] = winner.id

                self.resolution_log.append(
                    {
                        "time": current_time,
                        "action": "slow",
                        "yielder": yielder.id,
                        "winner": winner.id,
                        "distance": conflict.distance
                    }
                )
        to_release: List[int] = []

        for yielder_id, winner_id in self.yielding.items():
            if yielder_id not in active_ids or winner_id not in active_ids:
                to_release.append(yielder_id)
                continue

            dist = float(np.linalg.norm(drone_map[yielder_id].position - drone_map[winner_id].position))

            if dist >= self.resume_margin:
                to_release.append(yielder_id)

        for yielder_id in to_release:
            yielder = drone_map.get(yielder_id)

            if yielder and not yielder.arrived:
                yielder.resume_speed()

                self.resolution_log.append(
                    {
                        "time": current_time,
                        "action": "resume",
                        "yielder": yielder_id,
                        "winner": self.yielding.get(yielder_id),
                    }
                )
            self.yielding.pop(yielder_id, None)
        return self.resolution_log

    def summary(self):
        slows = [e for e in self.resolution_log if e["action"] == "slow"]
        resumes = [e for e in self.resolution_log if e["action"] == "resume"]

        return {
            "total_slow_events": len(slows),
            "total_resume_events": len(resumes),
            "drones_still_yielding": list(self.yielding.keys()),
        }