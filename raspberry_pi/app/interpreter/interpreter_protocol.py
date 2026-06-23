# pylint: disable=R0903

from typing import Protocol
from app.camera.camera_protocol import Request


class Interpreter(Protocol):
    """Interface for image interpreters."""

    def analyze(self, req: Request) -> None: ...
