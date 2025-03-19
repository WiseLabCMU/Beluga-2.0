import asyncio
from smp import error as smperror
from smp.os_management import OS_MGMT_RET_RC
from smpclient import SMPClient
from smpclient.generics import error, error_v1, error_v2, success
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smp.image_management import ImageState
from smpclient.requests.os_management import ResetWrite
from smpclient.transport.serial import SMPSerialTransport
from pathlib import Path
from typing import List, Union, Callable, Optional, Type
import time


class ImageManagerException(Exception):
    pass


async def _read_image_states(port: str) -> List[ImageState]:
    async with SMPClient(SMPSerialTransport(), port) as client:
        response = await client.request(ImageStatesRead(), timeout_s=2.0)
        if success(response):
            return response.images
        elif error(response):
            raise ImageManagerException(f"Received error: {response}")
        else:
            raise ImageManagerException(f"Unknown response: {response}")


def read_image_states(port: str) -> List[ImageState]:
    return asyncio.run(_read_image_states(port))


from abc import ABC, abstractmethod
class UploadStatusBase(ABC):
    def __init__(self, image_len: int):
        self._len = image_len

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    @abstractmethod
    def update(self, offset: int) -> None:
        pass


class UploadStatus(UploadStatusBase):
    def __init__(self, image_len: int):
        super().__init__(image_len)
        self._start_s: Optional[float] = None

    def __enter__(self):
        self._start_s = time.time()
        return super().__enter__()

    def update(self, offset: int):
        print(f"\rUploaded {offset:,} / {self._len:,} Bytes | "
              f"{offset / (time.time() - self._start_s) / 1000:.2f} KB/s           ",
              end="", flush=True)


async def _mark_test_image(client: SMPClient, port: str):
    images = await _read_image_states(port)
    await client.request(ImageStatesWrite(hash=images[1].hash))


async def __upload_image_stats(port: str, image: bytes, slot: int,
                               update_status: Type[UploadStatusBase]) -> None:
    with update_status(len(image)) as status:
        async with SMPClient(SMPSerialTransport(), port) as client:
            async for offset in client.upload(image, slot, first_timeout_s=2.5, use_sha=True):
                status.update(offset)
            await _mark_test_image(client, port)


async def __upload_image_no_stats(port: str, image: bytes, slot: int) -> None:
    async with SMPClient(SMPSerialTransport(), port) as client:
        async for _ in client.upload(image, slot, first_timeout_s=2.5, use_sha=True):
            pass
        await _mark_test_image(client, port)

async def __upload_image(port: str, build_dir: Union[Path, str], application: str, slot: int = 0,
                        update_status: Optional[Type[UploadStatusBase]] = None) -> None:
    image_bin = Path(build_dir) / application / "zephyr" / "zephyr.signed.bin"
    with open(image_bin, "rb") as f:
        image: bytes = f.read()
    if update_status is None:
        await __upload_image_no_stats(port, image, slot)
    else:
        await __upload_image_stats(port, image, slot, update_status)


async def _upload_image(port: str, build_dir: Union[Path, str], application: str, slot: int = 0,
                        update_status: Optional[Type[UploadStatusBase]] = None, retries: int = 3) -> None:
    if retries < 0:
        raise ValueError("Retries must be positive or 0")
    while True:
        try:
            await __upload_image(port, build_dir, application, slot, update_status)
        except Exception as e:
            retries -= 1
            if retries < 0:
                raise ImageManagerException(e)
            await asyncio.sleep(1)
        else:
            break


def upload_image(port: str, build_dir: Union[Path, str], application: str, slot: int = 0,
                 update_status: Optional[Type[UploadStatusBase]] = None, retries: int = 3):
    asyncio.run(_upload_image(port, build_dir, application, slot, update_status, retries))


async def _reset_mcu(port: str, force: bool):
    async with SMPClient(SMPSerialTransport(), port) as client:
        response = await client.request(ResetWrite(force=force), timeout_s=2.5)
        if error_v1(response):
            if response.rc != smperror.MGMT_ERR.EOK:
                raise ImageManagerException("Response is not OK")
        elif error_v2(response):
            if response.err.rc != OS_MGMT_RET_RC.OK:
                raise ImageManagerException("Response is not OK")


def reset_mcu(port: str, force: bool = False):
    asyncio.run(_reset_mcu(port, force))


async def _confirm_image(port: str, slot: int, force: bool) -> None:
    images: List[ImageState] = await _read_image_states(port)

    if slot < 0 or slot >= len(images):
        raise ImageManagerException("Invalid slot")

    image = images[slot]
    if not image.active or not image.bootable or image.confirmed:
        if not image.active and not force:
            raise ImageManagerException("Cannot confirm image that is not active")
        if not image.bootable:
            raise ImageManagerException("Cannot confirm image that is not bootable")
        if image.confirmed and not force:
            raise ImageManagerException("Cannot confirm image that has been confirmed already")
    async with SMPClient(SMPSerialTransport(), port) as client:
        await client.request(ImageStatesWrite(hash=image.hash, confirm=True), timeout_s=2.5)


def confirm_image(port: str, slot: int = 0, force: bool = False) -> None:
    asyncio.run(_confirm_image(port, slot, force))


async def _mark_slot_pending(port: str, slot: int):
    images: List[ImageState] = await _read_image_states(port)
    if slot >= len(images):
        raise ImageManagerException("Cannot mark image as pending since it doesn't exist")
    async with SMPClient(SMPSerialTransport(), port) as client:
        await client.request(ImageStatesWrite(hash=images[1].hash))


def mark_slot_pending(port: str, slot: int = 1):
    asyncio.run(_mark_slot_pending(port, slot))


if __name__ == "__main__":
    from find_port import find_mcumgr_ports
    targets = find_mcumgr_ports([])
    for target in targets:
        for port in targets[target]:
            images = read_image_states(port)
            print("Port:", port)
            for image in images:
                print("Slot:", image.slot)
                print("  Hash:", image.hash)
                print("  Version:", image.version)
                print("  Active:", image.active)
                print("  Bootable:", image.bootable)
                print("  Confirmed:", image.confirmed)
                print("  Pending:", image.pending)
                print("  Permanent:", image.permanent)
