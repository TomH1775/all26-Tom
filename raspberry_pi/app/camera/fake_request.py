# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from typing_extensions import Buffer, override
from cv2.typing import MatLike
from app.camera.request_protocol import Request
from app.decoder.mat_decoder import MatDecoder


class FakeRequest(Request):
    def __init__(self, img: MatLike, fps: float) -> None:
        """
        img: must be 3-channel cv2 BGR.
        """
        self.img = img
        self._fps = fps

    @override
    def decoder(self) -> MatDecoder:
        return MatDecoder()

    @override
    def fps(self) -> float:
        return self._fps

    @override
    def delay_us(self) -> int:
        return 500

    @override
    def buffer(self) -> AbstractContextManager[Buffer]:
        return nullcontext(self.img.copy().data)

    @override
    def release(self) -> None:
        pass
