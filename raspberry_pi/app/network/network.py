"""This is a wrapper for network tables."""

# pylint: disable=R0902,R0903,W0212


from threading import Event, Thread
from queue import Queue

import ntcore
from app.network.sync_loop import SyncLoop
from app.network.drift import Drift
from app.config.identity import Identity
from app.network.structs import Blip, Target


class DoubleSender:
    def __init__(self, pub: ntcore.DoublePublisher) -> None:
        self.pub = pub

    def send(self, val: float) -> None:
        self.pub.set(val)


class IntSender:
    def __init__(self, pub: ntcore.IntegerPublisher) -> None:
        self.pub = pub

    def send(self, val: int) -> None:
        self.pub.set(val)


class BlipSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Blip]) -> None:
        self.pub.set(val)


class TargetSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Target]) -> None:
        self.pub.set(val)


class Network:
    def __init__(self, identity: Identity) -> None:
        self._inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self._inst.startClient4("raspberry_pi")

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
        syncloop = SyncLoop(self._inst, self._queue, identity, done)
        self._drift = Drift(self._inst, self._queue, identity)
        Thread(target=syncloop.run).start()

    def server_time(self, localtime: int) -> int:
        return localtime + self._drift.get()

    def get_double_sender(self, name: str) -> DoubleSender:
        return DoubleSender(self._inst.getDoubleTopic(name).publish())

    def get_int_sender(self, name: str) -> IntSender:
        return IntSender(self._inst.getIntegerTopic(name).publish())

    def get_blip_sender(self, name: str) -> BlipSender:
        return BlipSender(self._inst.getStructArrayTopic(name, Blip).publish())

    def get_target_sender(self, name: str) -> TargetSender:
        return TargetSender(self._inst.getStructArrayTopic(name, Target).publish())

    def flush(self) -> None:
        self._inst.flush()
