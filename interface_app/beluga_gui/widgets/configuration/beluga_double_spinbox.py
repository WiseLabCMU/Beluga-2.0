from PyQt5.QtWidgets import QDoubleSpinBox


class FineGainDoubleSpinBox(QDoubleSpinBox):
    def update_power(self, power: int):
        stage = self.property("power_stage")
        mask = 0x1F
        shift = stage * 8
        setting = (power >> shift) & mask
        self.setValue(setting / 2)

    def simple_power_update(self, mode: int):
        if mode == 0:
            power = 0x0E080222
        else:
            power = 0x1F1F1F1F
        self.update_power(power)
