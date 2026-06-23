# pylint: disable=C0415,R0903

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    """Choose a camera implementation based on identity."""

    @staticmethod
    def get(identity: Identity) -> Camera:
        try:
            # import will fail if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            return RealCamera(identity)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            # 1/4 scale
            # return FakeCamera("images/tag_and_board.jpg", (1100, 620), -5)
            # full-size (huge)
            return FakeCamera("images/tag_and_board.jpg", (5504, 3096), -0.1, 0.1)
            # return FakeCamera("images/blob.jpg")
