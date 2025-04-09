from PyQt5.QtWidgets import QTableWidget, QWidget, QTableWidgetItem
from typing import Optional, Dict
from dataclasses import dataclass


@dataclass(init=True)
class Neighbor:
    id: int = 0
    rssi: int = 0
    distance: float = 0.0
    timestamp: int = 0
    exchange: int = 0
    dropped: bool = False
    row: int = 0


class NeighborListTable(QTableWidget):
    ID_INDEX = 0
    RSSI_INDEX = 1
    RANGE_INDEX = 2
    TIMESTAMP_INDEX = 3
    EXCHANGE_INDEX = 4
    DROPPED_INDEX = 5

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._drop_neighbors = False
        self._neighbors: Dict[int, Neighbor] = {}


    def remove_dropped_neighbors(self, drop: bool):
        self._drop_neighbors = drop
        self.update_neighbor_list()

    @staticmethod
    def id_difference(existing: dict, update: dict) -> set:
        return  set(update.keys()) - set(existing.keys())

    def _update_neighbor(self, neighbor: Neighbor):
        self.setItem(neighbor.row, self.ID_INDEX, QTableWidgetItem(str(neighbor.id)))
        self.setItem(neighbor.row, self.RSSI_INDEX, QTableWidgetItem(str(neighbor.rssi)))
        self.setItem(neighbor.row, self.RANGE_INDEX, QTableWidgetItem(str(neighbor.distance)))
        self.setItem(neighbor.row, self.TIMESTAMP_INDEX, QTableWidgetItem(str(neighbor.timestamp)))
        self.setItem(neighbor.row, self.EXCHANGE_INDEX, QTableWidgetItem(str(neighbor.exchange)))
        if neighbor.dropped:
            dropped = "Yes"
        else:
            dropped = "No"
        self.setItem(neighbor.row, self.DROPPED_INDEX, QTableWidgetItem(dropped))

    def _prune_dropped_neighbors(self):
        rows = []
        id_rm = []

        # Find rows and neighbors to remove
        for id_ in self._neighbors:
            if self._neighbors[id_].dropped:
                rows.append(self._neighbors[id_].row)
                id_rm.append(id_)

        # Remove rows bottom up
        for row in sorted(rows, reverse=True):
            self.removeRow(row)

        # Remove neighbors
        for id_ in id_rm:
            del self._neighbors[id_]

        # Update rows
        for row in range(self.rowCount()):
            id_ = int(self.item(row, self.ID_INDEX).text())
            self._neighbors[id_].row = row

    def _update_neighbor_list(self, neighbor_list: dict):
        missing_ids = self.id_difference(self._neighbors, neighbor_list)
        dropped_ids = self.id_difference(neighbor_list, self._neighbors)

        if missing_ids:
            row = self.rowCount()
            for id_ in missing_ids:
                attr = neighbor_list[id_]
                self._neighbors[id_] = Neighbor(id=id_, rssi=attr["RSSI"], distance=attr["RANGE"],
                                                timestamp=attr["TIMESTAMP"], exchange=attr["EXCHANGE"], row=row)
                self.insertRow(row)
                self._update_neighbor(self._neighbors[id_])
                row += 1

        for id_ in dropped_ids:
            self._neighbors[id_].dropped = True
            self._update_neighbor(self._neighbors[id_])

    def _sort_list(self):
        neighbors = list(self._neighbors.values())
        neighbors.sort(key=lambda neighbor_: neighbor_.id + (65536 * int(neighbor_.dropped)))
        for row, neighbor in enumerate(neighbors):
            self._neighbors[neighbor.id].row = row
            self._update_neighbor(self._neighbors[neighbor.id])

    def update_neighbor_list(self, neighbor_list: Optional[dict] = None):
        if neighbor_list is not None:
            self._update_neighbor_list(neighbor_list)
        if self._drop_neighbors:
            self._prune_dropped_neighbors()
        self._sort_list()

    def update_neighbors(self, updates: dict):
        for id_ in updates:
            if id_ in self._neighbors:
                self._neighbors[id_].rssi = updates[id_]["RSSI"]
                self._neighbors[id_].distance = updates[id_]["RANGE"]
                self._neighbors[id_].timestamp = updates[id_]["TIMESTAMP"]
                self._neighbors[id_].exchange = updates[id_]["EXCHANGE"]
                self._neighbors[id_].dropped = False
                self._update_neighbor(self._neighbors[id_])

    def clear(self):
        self._neighbors.clear()
        for row in range(self.rowCount() - 1, -1, -1):
            self.removeRow(row)
