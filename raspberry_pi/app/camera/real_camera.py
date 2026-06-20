# pylint: disable=E0401

from pprint import pprint
from typing import Any

import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2  # type: ignore
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request
from app.camera.config_factory import ConfigFactory
from app.camera.config_protocol import Config
from app.camera.distortion import Distortion
from app.camera.intrinsic import Intrinsic
from app.camera.model import Model
from app.camera.real_request import RealRequest
from app.camera.size import Size
from app.config.identity import Identity
from app.util.timer import Timer


class RealCamera(Camera):
    """This is a wrapper for Picamera2.

    It handles configuration of each camera according to the Pi identity.

    For more on the Picamera2 library, see the manual:

    https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

    and the source:

    https://github.com/raspberrypi/picamera2/
    """

    def __init__(self, identity: Identity) -> None:
        Picamera2.set_logging(Picamera2.INFO)  # type: ignore
        # debug logs with every frame (!)
        # Picamera2.set_logging(Picamera2.DEBUG)
        print("GLOBAL CAMERA INFO")
        pprint(Picamera2.global_camera_info())  # type: ignore
        print("+==================")
        self._cam: Picamera2 = Picamera2()  # type: ignore

        print("\n*** SENSOR MODES AVAILABLE")
        pprint(self._cam.sensor_modes)  # type:ignore

        print("\n*** CAMERA CONTROLS")
        pprint(self._cam.camera_controls)  # type:ignore

        print("\n*** RAW MODES")
        pprint(self._cam._raw_modes)  # type:ignore

        self._mtx: Intrinsic = Intrinsic(identity)
        self._dist: Distortion = Distortion(identity)

        model: Model = Model.get(self._cam.camera_properties)  # type: ignore
        self._size: Size = Size.from_model(model)

        print("\n\n*** CONFIG! ***\n\n")
        config: Config = ConfigFactory.get(identity)
        self._camera_config: dict[str, Any] = self._get_config(  # type: ignore
            config, self._size, identity, self._cam  # type: ignore
        )
        print("\n*** REQUESTED CONFIG")
        print(self._camera_config)
        # optimal alignment makes the ISP a little faster
        self._cam.align_configuration(self._camera_config, optimal=True)  # type:ignore
        print("\n*** ALIGNED CONFIG")
        print(self._camera_config)

        self._cam.configure(self._camera_config)  # type:ignore
        self._check_config(config)

        self._cam.start()  # type:ignore
        self._frame_time = Timer.time_ns()

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self._cam.capture_request()  # type:ignore
        total_time_ms = (capture_start - self._frame_time) / 1000000
        self._frame_time = capture_start
        fps = 1000 / total_time_ms
        return RealRequest(req, fps)  # type: ignore

    @override
    def stop(self) -> None:
        self._cam.stop()  # type: ignore
        print("Camera stop")

    @override
    def get_size(self) -> Size:
        return self._size

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        return self._mtx.get()

    @override
    def get_dist(self) -> NDArray[np.float32]:
        return self._dist.get()

    def _get_config(
        self,
        conf: Config,
        size: Size,
        identity: Identity,
        cam: Picamera2,  # type: ignore
    ) -> dict[str, Any]:
        """Produce a camera configuration dict for a single
        "main" stream.  Years ago, we attempted to use both "main"
        and "lores" so we could run both color and greyscale
        detection on the same frame without conversion, but we
        never really needed that, and it's not supported for
        UVC cameras, so it's gone.  There's just one "main"
        stream."""
        return cam.create_still_configuration(  # type:ignore
            buffer_count=conf.buffer_count(),
            queue=conf.queue(),
            sensor=conf.sensor(size),
            main=conf.main(size),
            raw=conf.raw(),
            transform=conf.transform(identity),  # type:ignore
            controls=conf.controls(),
        )

    def _check_config(self, config: Config):
        """Verify the requested config matches the camera config."""
        if not config.ok(self._camera_config, self._cam.camera_config):  # type:ignore
            pprint(self._camera_config)
            pprint(self._cam.camera_config)  # type:ignore
            raise ValueError("Config check fail")
