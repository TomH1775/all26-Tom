# pylint: disable=C0415,E0611,E1101

from platform import system
import cv2
from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection
from typing_extensions import override
from wpimath.geometry import Transform3d
from app.dashboard.display import Display

FONT = cv2.FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class RealDisplay(Display):
    """Annotate and show the captured image through the CameraServer."""

    def __init__(self, width: int, height: int) -> None:
        print("\n*** Display: RealDisplay")
        self._width: int = width
        self._height: int = height
        name: str = "display"
        if system() == "Windows":
            from app.dashboard.mjpeg_streamer import MjpegServer, Stream

            print("Using MJpegServer for Windows")
            # on windows, cvsource breaks with cvnp contiguous-array error
            self._stream = Stream(name, (width, height), quality=50, fps=30)
            self._server = MjpegServer("localhost", 1181)
            self._server.add_stream(self._stream)
            self._server.start()
        else:
            from cscore import CameraServer  # type: ignore

            print("Using CameraServer for Linux.  See localhost:1181,1182,etc")
            self._cvsource = CameraServer.putVideo(name, width, height)

    @override
    def tag(self, image: MatLike, tag: AprilTagDetection, pose: Transform3d) -> None:
        # Draw lines around the tag
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(tag.getCorner(i).x), int(tag.getCorner(i).y))
            point2 = (int(tag.getCorner(j).x), int(tag.getCorner(j).y))
            cv2.line(image, point1, point2, WHITE, 2)

        (c_x, c_y) = (int(tag.getCenter().x), int(tag.getCenter().y))
        cv2.circle(image, (c_x, c_y), 10, WHITE, -1)

        # type the translation into the image, in WPI coords (x-forward)
        t = pose.translation()
        self.text(
            image,
            f"X: {t.z:6.3f}",
            (c_x, c_y),
        )
        self.text(
            image,
            f"Y: {-t.x:6.3f}",
            (c_x, c_y + 80),
        )
        self.text(
            image,
            f"Z: {-t.y:6.3f}",
            (c_x, c_y + 160),
        )

    @override
    def note(self, image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None:
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)
        cv2.circle(image, (c_x, c_y), 7, (0, 0, 0), -1)

    # these are white with black outline
    @override
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None:
        cv2.putText(image, msg, loc, FONT, 3, BLACK, 10)
        cv2.putText(image, msg, loc, FONT, 3, WHITE, 5)

    @override
    def put(self, img: MatLike) -> None:
        """img should be a uint8 array, either:
        single-channel (height, width) grayscale, or
        three-channel (height, width, 3) BGR."""
        img_out = cv2.resize(img, (self._width, self._height))
        if system() == "Windows":
            self._stream.set_frame(img_out)  # type: ignore
        else:
            self._cvsource.putFrame(img_out)  # type: ignore
