from dataclasses import dataclass
from typing import List, Optional, Literal


@dataclass(init=True)
class Neighbor:
    id: int = 0
    rssi: int = 0
    distance: float = 0.0
    timestamp: int = 0
    exchange: int = 0
    dropped: bool = False
    row: int = 0


class NeighborHistory:
    def __init__(self, save_history: Literal["distance", "rssi", "distance & rssi"], history_depth: int = 10000):
        self._history_depth = history_depth
        self._depth = 0

        self._history_time: Optional[List[int]] = None
        self._history_distance: Optional[List[float]] = None
        self._history_rssi: Optional[List[int]] = None

        if save_history == "distance":
            self._history_time = []
            self._history_distance = []
        elif save_history == "rssi":
            self._history_time = []
            self._history_rssi = []
        elif save_history == "distance_v_rssi":
            self._history_distance = []
            self._history_rssi = []
        else:
            raise ValueError(f"Unknown value: {save_history}")

    @property
    def time(self):
        if self._history_time is not None:
            return self._history_time
        raise AttributeError("This object is not saving time data")

    @property
    def distance(self):
        if self._history_distance is not None:
            return self._history_distance
        raise AttributeError("This object is not saving distance data")

    @property
    def rssi(self):
        if self._history_rssi is not None:
            return self._history_rssi
        raise AttributeError("This object is not saving RSSI data")

    def _update_time(self, timestamp):
        if self._history_time is not None:
            self._history_time.append(timestamp)
            if self._depth > self._history_depth:
                del self._history_time[0]

    def _update_distance(self, range_):
        if self._history_distance is not None:
            self._history_distance.append(range_)
            if self._depth > self._history_depth:
                del self._history_distance[0]

    def _update_rssi(self, rssi):
        if self._history_rssi is not None:
            self._history_rssi.append(rssi)
            if self._depth > self._history_depth:
                del self._history_rssi[0]

    def update_history(self, updates: dict):
        self._depth += 1
        self._update_time(updates["TIMESTAMP"])
        self._update_distance(updates["RANGE"])
        self._update_rssi((updates["RSSI"]))
        if self._depth > self._history_depth:
            self._depth -= 1

    def clear(self):
        if self._history_time is not None:
            self._history_time.clear()
        if self._history_rssi is not None:
            self._history_rssi.clear()
        if self._history_distance is not None:
            self._history_distance.clear()
        self._depth = 0
