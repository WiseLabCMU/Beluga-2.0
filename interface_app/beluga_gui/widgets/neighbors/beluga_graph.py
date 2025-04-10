from pyqtgraph import PlotWidget, mkPen, ScatterPlotItem
from PyQt5.QtWidgets import QWidget
from typing import Optional, Any, Dict
from .neighbors import NeighborHistory


class BelugaGraph(PlotWidget):
    COLORS = ["b", "c", "d", "g", "m", "r", "w", "y"]
    MARKERS = ["o", "s", "t", "d", "+", "t1", "t2", "t3", "p", "h", "star", "x", "arrow_up", "arrow_right",
               "arrow_down", "arrow_left", "crosshair"]

    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None, **kwargs):
        super().__init__(parent, background, plot_item, **kwargs)
        self._neighbors: Dict[int, NeighborHistory] = {}
        self._drop_neighbors = False
        self._color_index = 0
        self._symbol_index = 0
        self.showGrid(x=True, y=True)
        self.addLegend()

    def _fetch_marker_and_color(self):
        color = self.COLORS[self._color_index]
        symbol = self.MARKERS[self._symbol_index]
        self._color_index += 1
        if self._color_index % len(self.COLORS) == 0:
            self._color_index = 0
            self._symbol_index += 1
            if self._symbol_index % len(self.MARKERS) == 0:
                self._symbol_index = 0
        return symbol, color

    @staticmethod
    def id_difference(existing: dict, update: dict) -> set:
        return set(update.keys()) - set(existing.keys())

    def _prune_dropped_neighbors(self):
        id_rm = []

        # Find ids
        for id_ in self._neighbors:
            if self._neighbors[id_].dropped:
                id_rm.append(id_)

        # Remove ids and remove graphs
        for id_ in id_rm:
            self.removeItem(self._neighbors[id_].plot)
            del self._neighbors[id_]

    def _update_neighbor_list(self, neighbor_list: dict):
        missing_ids = self.id_difference(self._neighbors, neighbor_list)
        dropped_ids = self.id_difference(neighbor_list, self._neighbors)
        marker, color = self._fetch_marker_and_color()
        for id_ in dropped_ids:
            self._neighbors[id_].dropped = True
        return missing_ids, marker, color

    def update_neighbor_list(self, neighbor_list: Optional[dict] = None):
        if neighbor_list is not None:
            self._update_neighbor_list(neighbor_list)
        if self._drop_neighbors:
            self._prune_dropped_neighbors()

    def update_neighbors(self, neighbor_list: dict):
        # Must be defined in subclass
        pass

    def clear_neighbors(self):
        for id_ in self._neighbors:
            self.removeItem(self._neighbors[id_].plot)
        self._neighbors.clear()

    def remove_dropped_neighbors(self, state):
        self._drop_neighbors = state
        self.update_neighbor_list()


class DistanceGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="Distance")
        self.plotItem.getAxis('left').setLabel(text='Distance', units='m')
        self.plotItem.getAxis("bottom").setLabel(text="Time", units="s")

    def _update_neighbor_list(self, neighbor_list: dict):
        missing_ids, marker, color = super()._update_neighbor_list(neighbor_list)
        for id_ in missing_ids:
            self._neighbors[id_] = NeighborHistory("distance", self.plot([], [], name=f"Node ID {id_}",
                                                                           pen=mkPen(color), symbol=marker,
                                                                           symbolSize=10,
                                                                           symbolBrush=color))

    def update_neighbors(self, neighbor_list: dict):
        for id_ in neighbor_list:
            if id_ in self._neighbors:
                self._neighbors[id_].update_history(neighbor_list[id_])
                self._neighbors[id_].plot.setData(self._neighbors[id_].time, self._neighbors[id_].distance)


class RssiGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="RSSI")
        self.plotItem.getAxis('left').setLabel(text='RSSI', units="dBm")
        self.plotItem.getAxis("bottom").setLabel(text="Time", units="s")

    def _update_neighbor_list(self, neighbor_list: dict):
        missing_ids, marker, color = super()._update_neighbor_list(neighbor_list)
        for id_ in missing_ids:
            self._neighbors[id_] = NeighborHistory("rssi", self.plot([], [], name=f"Node ID {id_}",
                                                                         pen=mkPen(color), symbol=marker,
                                                                         symbolSize=10,
                                                                         symbolBrush=color))

    def update_neighbors(self, neighbor_list: dict):
        for id_ in neighbor_list:
            if id_ in self._neighbors:
                self._neighbors[id_].update_history(neighbor_list[id_])
                self._neighbors[id_].plot.setData(self._neighbors[id_].time, self._neighbors[id_].rssi)

class DistanceVRssiGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="Distance vs RSSI")
        self.plotItem.getAxis('left').setLabel(text='RSSI', units="dBm")
        self.plotItem.getAxis("bottom").setLabel(text="Distance", units="m")

    def _update_neighbor_list(self, neighbor_list: dict):
        missing_ids, marker, color = super()._update_neighbor_list(neighbor_list)
        for id_ in missing_ids:
            self._neighbors[id_] = NeighborHistory("distance & rssi", ScatterPlotItem(name=f"Node ID {id_}",
                                                                     pen=mkPen(color), symbol=marker,
                                                                     symbolSize=10,
                                                                     symbolBrush=color))
            self.addItem(self._neighbors[id_].plot)

    def update_neighbors(self, neighbor_list: dict):
        for id_ in neighbor_list:
            if id_ in self._neighbors:
                self._neighbors[id_].update_history(neighbor_list[id_])
                self._neighbors[id_].plot.setData(self._neighbors[id_].distance, self._neighbors[id_].rssi)
