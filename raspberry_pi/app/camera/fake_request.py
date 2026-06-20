# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from typing_extensions import Buffer, override
import cv2
from cv2.typing import MatLike
from app.camera.request_protocol import Request


class FakeRequest(Request):
    def __init__(self, img: MatLike, fps: float, yuv: bool) -> None:
        """
        img: should be cv2 RGB (really BGR).
        yuv: if true, transcode the input to YUV420.
        """
        self.img = img
        self._fps = fps
        self._yuv = yuv

    @override
    def fps(self) -> float:
        return self._fps

    @override
    def delay_us(self) -> int:
        return 500

    @override
    def buffer(self) -> AbstractContextManager[Buffer]:
        if self._yuv:
            return self.yuv()
        return self.rgb()

    def rgb(self) -> AbstractContextManager[Buffer]:
        return nullcontext(self.img.copy().data)

    def yuv(self) -> AbstractContextManager[Buffer]:
        return nullcontext(cv2.cvtColor(self.img, cv2.COLOR_RGB2YUV_I420).data)

    @override
    def release(self) -> None:
        pass
