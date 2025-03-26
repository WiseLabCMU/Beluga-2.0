from typing import Dict, List, Union


class BelugaEntryError(Exception):
    def __init__(self, msg: str, header: bool = False):
        self.header = header
        super().__init__(msg)


class BelugaNeighbor:
    def __init__(self, neighbor: dict):
        self._id: int = neighbor["ID"]
        self._range: float = 0.0
        self._rssi: int = 0
        self._time: int = 0
        self._exchange_id: int = 0
        self._updated = False
        self.update(neighbor)

    def __iter__(self):
        return iter(
            [("RANGE", self._range), ("RSSI", self._rssi), ("TIMESTAMP", self._time), ("EXCHANGE", self._exchange_id)])

    @property
    def id(self) -> int:
        return self._id

    @property
    def range(self) -> float:
        return self._range

    @property
    def rssi(self) -> int:
        return self._rssi

    @property
    def time(self) -> int:
        return self._time

    @property
    def exchange(self) -> int:
        return self._exchange_id

    @property
    def updated(self) -> bool:
        return self._updated

    @updated.setter
    def updated(self, update: bool):
        self._updated = update

    def update(self, neighbor: dict):
        self._range = neighbor["RANGE"]
        self._rssi = neighbor["RSSI"]
        self._time = neighbor["TIMESTAMP"]
        if "EXCHANGE" in neighbor.keys():
            self._exchange_id = neighbor["EXCHANGE"]
        self._updated = True


class BelugaNeighborList:
    def __init__(self):
        self._list: Dict[int, BelugaNeighbor] = {}
        self._neighbors_update: bool = False
        self._range_update: bool = False
        return

    def update(self, updates: List[Dict[str, Union[int, float]]]):
        for entry in updates:
            if entry["ID"] not in self._list.keys():
                self._list[entry["ID"]] = BelugaNeighbor(entry)
                self._range_update = True
                self._neighbors_update = True
            else:
                self._list[entry["ID"]].update(entry)
                self._range_update = True

    def remove_neighbor(self, neighbor_id: int):
        if neighbor_id in self._list.keys():
            del self._list[neighbor_id]
            self._neighbors_update = True

    def get_updates(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            if x.updated:
                ret[x.id] = dict(x)
                x.updated = False
        self._range_update = False
        return ret

    def get_neighbors(self) -> Dict[int, Dict[str, Union[int, float]]]:
        ret = {}
        for x in self._list.values():
            ret[x.id] = dict(x)
        self._neighbors_update = False
        return ret

    def clear(self):
        if self._list:
            self._list.clear()
            self._neighbors_update = True
            self._range_update = False

    @property
    def neighbor_update(self) -> bool:
        return self._neighbors_update

    @property
    def range_update(self) -> bool:
        return self._range_update
