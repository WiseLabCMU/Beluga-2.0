from enum import IntEnum
from typing import Optional, Union, Dict, List, Tuple
import json
import struct


class FrameType(IntEnum):
    """Enumerations for different frames types for Beluga"""
    RESPONSE = 0,
    UPDATES = 1,
    EVENT = 2,
    DROP = 3,
    START = 4,
    NO_TYPE = 5,

    def __str__(self):
        return f"{self.name} ({self.value})"


HEADER_BYTES = 1
PAYLOAD_LEN_BYTES = 2
TYPE_BYTES = 1
FOOTER_BYTES = 1

BYTE_FORMAT_MAP = {1: "B", 2: "H", 4: "I", 8: "Q"}

HEADER_FORMAT = f"={BYTE_FORMAT_MAP[HEADER_BYTES]}{BYTE_FORMAT_MAP[PAYLOAD_LEN_BYTES]}{BYTE_FORMAT_MAP[TYPE_BYTES]}"
FOOTER_FORMAT = f"={BYTE_FORMAT_MAP[FOOTER_BYTES]}"
PAYLOAD_FORMAT = f"={BYTE_FORMAT_MAP[PAYLOAD_LEN_BYTES]}"

FRAME_HEADER_OVERHEAD = HEADER_BYTES + PAYLOAD_LEN_BYTES + TYPE_BYTES
FRAME_FOOTER_OVERHEAD = FOOTER_BYTES

FRAME_OVERHEAD = FRAME_HEADER_OVERHEAD + FRAME_FOOTER_OVERHEAD

MAX_PAYLOAD_BYTES = (2 ** (8 * PAYLOAD_LEN_BYTES)) - 1

FRAME_MAX_SIZE = MAX_PAYLOAD_BYTES + FRAME_OVERHEAD

HEADER_OFFSET = 0
PAYLOAD_LEN_OFFSET = HEADER_OFFSET + HEADER_BYTES
TYPE_OFFSET = PAYLOAD_LEN_OFFSET + PAYLOAD_LEN_BYTES

HEADER_BYTE = ord("@")
FOOTER_BYTE = ord("*")


class BelugaFrame:
    def __init__(self, type_: FrameType, payload):
        self._type: Optional[FrameType] = type_
        self._payload: Optional[Union[str, int, List[Dict[str, Union[int, float]]]]] = payload

    def __str__(self):
        str_val = f"{self._type}\n"
        if isinstance(self._payload, list):
            str_val += f"Payload: {json.dumps(self._payload)}"
        else:
            str_val += f"Payload: {self._payload}"
        return str_val

    @classmethod
    def frame_present(cls, data: bytearray, error_no_footer: bool = True) -> Tuple[int, int, int]:
        """
        Checks if a frame is present in the data

        :param data: The data to check
        :param error_no_footer: Indicates whether to check if the footer is correct or present.
        :return: A tuple indicating the header index, the frame size, and the number of bytes left
        to read to create a complete frame. If the last element is zero or negative, indicates
        that a complete frame is present.
        """
        data_len = len(data)

        for i, byte in enumerate(data):
            if byte != HEADER_BYTE:
                continue
            header_index = i

            type_index = header_index + TYPE_OFFSET
            if type_index > data_len:
                continue

            payload_len_index = header_index + PAYLOAD_LEN_OFFSET
            payload_len = struct.unpack(PAYLOAD_FORMAT, data[payload_len_index:payload_len_index + PAYLOAD_LEN_BYTES])[0]

            footer_index = header_index + FRAME_HEADER_OVERHEAD + payload_len
            if footer_index >= data_len and error_no_footer:
                continue

            if error_no_footer:
                if data[footer_index] != FOOTER_BYTE:
                    continue

            frame_size = FRAME_OVERHEAD + payload_len
            return header_index, frame_size, footer_index - len(data[header_index:]) + 1
        return -1, -1, -1


    @classmethod
    def extract_frame(cls, data: bytearray, frame_start: int) -> 'BelugaFrame':
        type_ = FrameType(data[frame_start + TYPE_OFFSET])

        payload_len_index = frame_start + PAYLOAD_LEN_OFFSET
        payload_len = struct.unpack(PAYLOAD_FORMAT, data[payload_len_index:payload_len_index + PAYLOAD_LEN_BYTES])[0]
        payload_start = frame_start + FRAME_HEADER_OVERHEAD

        payload_ = data[payload_start:payload_start + payload_len]

        match type_:
            case FrameType.RESPONSE:
                payload = payload_.decode()
            case FrameType.UPDATES:
                payload = json.loads(payload_.decode())
            case FrameType.EVENT:
                payload = json.loads(payload_.decode())
            case FrameType.DROP:
                payload = int(payload_.decode())
            case FrameType.START:
                payload = payload_.decode()
            case _:
                raise ValueError("Invalid type")

        return BelugaFrame(type_, payload)

    @property
    def type(self):
        return self._type

    @property
    def payload(self):
        return self._payload
