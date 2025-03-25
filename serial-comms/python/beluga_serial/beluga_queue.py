from typing import Optional
import queue


class BelugaQueue(queue.Queue):
    def __init__(self, maxsize: int = 1, update_old_items: bool = True):
        self._update = update_old_items
        super().__init__(maxsize=maxsize)

    def put(self, obj, block: bool = True, timeout: Optional[float] = None) -> None:
        try:
            super().put(obj, block, timeout)
        except queue.Full:
            try:
                item = self.get_nowait()
            except queue.Empty:
                # Queue item got removed between the put and the get operations. Place new item
                pass
            else:
                if self._update:
                    # This will update the existing keys, add new key-value pairs if not present, and keep keys that
                    # are not present in the new obj unchanged
                    item.update(obj)
                    obj = item
                # else drop the old item...
            super().put(obj, block, timeout)
        return

    def clear(self):
        while not super().empty():
            try:
                self.get_nowait()
            except queue.Empty:
                pass
