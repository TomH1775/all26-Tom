# pylint: disable=E1101

from numpy.typing import NDArray
from cv2.typing import MatLike
from typing_extensions import override, Buffer
import cv2
import numpy as np
from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class YuyvDecoder(Decoder):
    """Adapts buffers encoded as YUYV."""

    def __init__(self, size: Size):
        self._size = size

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        img = img.reshape((self._size.height, self._size.width * 2))  # type:ignore
        img = img[:, ::2]
        # the stride above is just a noncontiguous view, so:
        return np.ascontiguousarray(img)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        img = img.reshape((self._size.height, self._size.width * 2))  # type:ignore
        img_bgr: MatLike = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
        return np.ascontiguousarray(img_bgr)
