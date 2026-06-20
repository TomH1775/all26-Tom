# pylint: disable=E0401

from typing import Any
from typing_extensions import override

from app.camera.config_protocol import Config
from app.camera.size import Size


class ConfigGsMono(Config):
    """The Raspberry Pi Global Shutter camera, monochrome.
    It uses the YUV420 12-bit encoding option from the camera.
    """

    def __init__(self) -> None:
        print("\n*** Config: ConfigGsMono")

    @override
    def sensor(self, size: Size) -> dict[str, Any]:
        # for rpi camera
        return {
            "output_size": (size.sensor_width, size.sensor_height),
            "bit_depth": 10,
        }

    @override
    def main(self, size: Size) -> dict[str, Any]:
        """YUV420 encoding."""
        return {"format": "YUV420", "size": (size.width, size.height)}

    def controls(self) -> dict[str, Any]:
        return {
            # ANALOGUE GAIN
            # To minimize blur, set this as high as possible.
            # TODO: try much larger values, up to 250.
            # "AnalogueGain": 8,
            #
            # AUTO EXPOSURE
            # Must be true for outside or in bright sun.
            # "AeEnable": True,
            # "AeEnable": False,
            #
            # EXPOSURE TIME (microseconds)
            # Minimizes blur.  Requires pretty good light.
            # "ExposureTime": 500,
            # Works in less light, slightly more blur.
            # "ExposureTime": 2000,
            #
            # FRAME DURATION LIMITS
            # limit auto exposure: go as fast as possible but no slower than 30fps
            # without a duration limit, we slow down in the dark, which is fine
            # "FrameDurationLimits": (500, 33333),  # 41 fps
            #
            # NOISE REDUCTION MODE
            # noise reduction takes A LOT of time (100 ms per frame!), don't need it.
            # See libcamera.controls.draft.NoiseReductionModeEnum.Off,
            # "NoiseReductionMode": 0,
        }

    @override
    def ok(self, conf: dict[str, Any], cam: dict[str, Any]) -> bool:
        """Configured size and actual size must match."""
        return conf["sensor"]["output_size"] == cam["sensor"]["output_size"]
