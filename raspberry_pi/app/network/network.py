"""This is a wrapper for network tables."""

# pylint: disable=R0902,R0903,W0212


from threading import Event, Thread
from queue import Queue

import ntcore
from wpimath.geometry import Rotation3d
from app.network.sync_loop import SyncLoop
from app.network.drift import Drift
from app.config.identity import Identity
from app.network.structs import (
    Blip24,
    PoseEstimate25,
)

# global singleton
start_time_us = ntcore._now()


class DoubleSender:
    def __init__(self, drift: Drift, pub: ntcore.DoublePublisher) -> None:
        self.drift = drift
        self.pub = pub

    def send(self, val: float, delay_us: int) -> None:
        self.drift.get()
        self.pub.set(val, int(ntcore._now() - delay_us))


class IntSender:
    def __init__(self, drift: Drift, pub: ntcore.IntegerPublisher) -> None:
        self.drift = drift
        self.pub = pub

    def send(self, val: int, delay_us: int) -> None:
        self.drift.get()
        self.pub.set(val, int(ntcore._now() - delay_us))


class BlipSender:
    def __init__(self, drift: Drift, pub: ntcore.StructArrayPublisher) -> None:
        self.drift = drift
        self.pub = pub

    def send(self, val: list[Blip24], delay_us: int) -> None:
        self.drift.get()
        self.pub.set(val, int(ntcore._now() - delay_us))


class NoteSender:
    def __init__(self, drift: Drift, pub: ntcore.StructArrayPublisher) -> None:
        self.drift = drift
        self.pub = pub

    def send(self, val: list[Rotation3d], delay_us: int) -> None:
        self.drift.get()
        self.pub.set(val, int(ntcore._now() - delay_us))


class PoseSender:
    def __init__(self, drift: Drift, pub: ntcore.StructPublisher) -> None:
        self.drift = drift
        self.pub = pub

    def send(self, val: PoseEstimate25, delay_us: int) -> None:
        self.drift.get()
        self.pub.set(val, int(ntcore._now() - delay_us))


class Network:
    def __init__(self, identity: Identity) -> None:
        # TODO: use identity.name instead
        self._serial: str = identity.value
        self._inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )

        self._inst.startClient4("tag_finder24")

        # ntcore._now()

        # roboRio address. windows machines can impersonate this for simulation.
        # also localhost for testing
        if identity == Identity.UNKNOWN:
            # vasili says this doesn't work, but i need it for testing.
            self._inst.setServer("localhost")
        else:
            # this works
            self._inst.setServer("10.1.0.2")

        self._queue: Queue = Queue()
        done = Event()

        # Fill the queue at 50 hz
        syncloop = SyncLoop(self._inst, self._queue, done)
        self._drift = Drift(self._inst, self._queue)
        Thread(target=syncloop.run).start()

    def now(self) -> int:
        return ntcore._now() - start_time_us

    def get_double_sender(self, name: str) -> DoubleSender:
        return DoubleSender(self._drift, self._inst.getDoubleTopic(name).publish())

    def get_int_sender(self, name: str) -> IntSender:
        return IntSender(self._drift, self._inst.getIntegerTopic(name).publish())

    def get_blip_sender(self, name: str) -> BlipSender:
        return BlipSender(
            self._drift, self._inst.getStructArrayTopic(name, Blip24).publish()
        )

    def get_note_sender(self, name: str) -> NoteSender:
        return NoteSender(
            self._drift, self._inst.getStructArrayTopic(name, Rotation3d).publish()
        )

    def get_pose_sender(self, name: str) -> PoseSender:
        return PoseSender(
            self._drift, self._inst.getStructTopic(name, PoseEstimate25).publish()
        )

    def flush(self) -> None:
        self._inst.flush()
