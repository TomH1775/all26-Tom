"""Network Tables payloads"""

# pylint: disable=C0301,R0902,R0903,W0212,W2301

import dataclasses

from wpimath.geometry import Rotation3d, Transform3d
from wpiutil import wpistruct


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip:
    """AprilTag pose"""

    timestamp: wpistruct.int64  # server time
    id: wpistruct.int32  # tag id
    pose: Transform3d  # camera-relative


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Target:
    """Game piece target"""

    timestamp: wpistruct.int64  # server time
    sight: Rotation3d  # camera-relative


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncRequest:
    """Clock sync request packet"""

    org: wpistruct.int64


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncReply:
    """Clock sync reply packet"""

    org: wpistruct.int64
    rec: wpistruct.int64
    xmt: wpistruct.int64
