# pylint: disable=E1101

from numpy.typing import NDArray
from cv2.typing import MatLike

from typing_extensions import override, Buffer
import cv2
import numpy as np
from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class MjpegDecoder(Decoder):
    """Adapts buffers encoded as JPEG."""

    def __init__(self, size: Size):
        self._size = size

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_GRAYSCALE)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_COLOR)
