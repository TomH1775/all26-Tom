# pylint: disable=C0103,E1101,R0903

import cv2
import numpy as np
from numpy.typing import NDArray
from typing_extensions import Buffer, override
from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.dashboard.display import Display
from app.network.network_protocol import Network


class NullDetector(Interpreter):
    """A detector that does nothing but publish its input."""

    def __init__(
        self,
        cam: Camera,
        display: Display,
        network: Network,
    ) -> None:
        self.display = display
        self._network = network
        # network output for camera FPS
        self._fps = network.get_double_sender("fps")

        print("\n*** Interpreter: NullDetector")
        size: Size = cam.get_size()
        self.width: int = size.width
        self.height: int = size.height
        self.y_len = self.width * self.height
        print(f"width {self.width} height {self.height} ylen {self.y_len}")

    @override
    def analyze(self, req: Request) -> None:
        self.analyzeMJPEG(req)
        # self.analyzeRGB(req)
        # self.analyzeYUV420(req)
        # self.analyzeYUYV(req)

    def analyzeMJPEG(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            # buffer here is jpeg encoded.
            img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
            img_bgr = cv2.imdecode(img, 0)  # grayscale
            fps = req.fps()
            self._fps.send(fps)
            delay_us = req.delay_us()
            self.display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
            self.display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            self.display.put(img_bgr)

    def analyzeRGB(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
            img_bgr: NDArray[np.uint8] = img.reshape((self.height, self.width, 3))
            fps = req.fps()
            self._fps.send(fps)
            delay_us = req.delay_us()
            self.display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
            self.display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            self.display.put(img_bgr)

    def analyzeYUV420(self, req: Request) -> None:
        # YUV420 puts all the luminance first
        # followed by the two chrominance images ("planes")
        # at reduced resolution.
        with req.buffer() as buffer:
            img: NDArray[np.uint8] = np.frombuffer(
                buffer, dtype=np.uint8, count=self.y_len
            )
            img = img.reshape((self.height, self.width))  # type:ignore
            fps = req.fps()
            self._fps.send(fps)
            delay_us = req.delay_us()
            self.display.text(img, f"FPS {fps:2.0f}", (10, 80))
            self.display.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self.display.put(img)

    def analyzeYUYV(self, req: Request) -> None:
        # YUYV puts each luminance and chrominance pixel
        # next to each other instead of in separate "planes"
        # so extract it with "stride"
        with req.buffer() as buffer:
            img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
            img = img.reshape((self.height, self.width * 2))  # type:ignore
            img = img[:, ::2]
            # the stride above is just a noncontiguous view, so:
            img = np.ascontiguousarray(img)

            fps = req.fps()
            self._fps.send(fps)
            delay_us = req.delay_us()
            self.display.text(img, f"FPS {fps:2.0f}", (10, 80))
            self.display.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self.display.put(img)
