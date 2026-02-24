""" This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

from threading import Event, Thread

from app.camera.camera_factory import CameraFactory
from app.camera.interpreter_factory import InterpreterFactory
from app.camera.camera_loop import CameraLoop
from app.config.identity import Identity
from app.network.network import Network


def main() -> None:
    identity: Identity = Identity.get()
    network = Network(identity)

    done = Event()
    try:
        camera = CameraFactory.get(identity)
        detector = InterpreterFactory.get(identity, camera, network)
        camera_loop = CameraLoop(camera, [detector], done)
        Thread(target=camera_loop.run).start()
        done.wait()

    finally:
        done.set()  # exit all threads cleanly
