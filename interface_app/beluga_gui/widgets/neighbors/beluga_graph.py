from pyqtgraph import PlotWidget, PlotItem
from PyQt5.QtWidgets import QWidget
from typing import Optional, Any
from enum import Enum, auto
from .neighbors import NeighborHistory


class BelugaGraph(PlotWidget):
    class GraphType(Enum):
        DISTANCE = auto()
        RSSI = auto()
        DISTANCE_V_RSSI = auto()

    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None, **kwargs):
        super().__init__(parent, background, plot_item, **kwargs)

    def clear(self):
        pass

    def remove_dropped_neighbors(self):
        pass


class DistanceGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="Distance")
        self.plotItem.getAxis('left').setLabel(text='Distance', units='m')
        self.plotItem.getAxis("bottom").setLabel(text="Time", units="ms")


class RssiGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="RSSI")
        self.plotItem.getAxis('left').setLabel(text='RSSI', units="dBm")
        self.plotItem.getAxis("bottom").setLabel(text="Time", units="ms")


class DistanceVRssiGraph(BelugaGraph):
    def __init__(self, parent: Optional[QWidget] = None, background = 'default', plot_item: Any = None):
        super().__init__(parent, background, plot_item, title="Distance vs RSSI")
        self.plotItem.getAxis('left').setLabel(text='RSSI', units="dBm")
        self.plotItem.getAxis("bottom").setLabel(text="Distance", units="m")
