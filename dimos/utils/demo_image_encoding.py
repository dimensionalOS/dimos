# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
# Usage

Run it with uncompressed LCM:

    python dimos/utils/demo_image_encoding.py

Run it with JPEG LCM:

    python dimos/utils/demo_image_encoding.py --use-jpeg
"""

import argparse
import threading
import time

from reactivex.disposable import Disposable

from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import JpegLcmTransport, LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.fast_image_generator import random_image


class EmitterModule(Module):
    image: Out[Image] = None

    _thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None

    def start(self):
        super().start()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._publish_image, daemon=True)
        self._thread.start()

    def stop(self):
        if self._thread:
            self._stop_event.set()
            self._thread.join(timeout=2)
        super().stop()

    def _publish_image(self):
        open_file = open("/tmp/emitter-times", "w")
        while not self._stop_event.is_set():
            start = time.time()
            data = random_image(1280, 720)
            total = time.time() - start
            print("took", total)
            open_file.write(str(time.time()) + "\n")
            self.image.publish(Image(data=data))
        open_file.close()


class ReceiverModule(Module):
    image: In[Image] = None

    _open_file = None

    def start(self):
        super().start()
        self._disposables.add(Disposable(self.image.subscribe(self._on_image)))
        self._open_file = open("/tmp/receiver-times", "w")

    def stop(self):
        self._open_file.close()
        super().stop()

    def _on_image(self, image: Image):
        self._open_file.write(str(time.time()) + "\n")
        print("image")


def main():
    parser = argparse.ArgumentParser(description="Demo image encoding with transport options")
    parser.add_argument(
        "--use-jpeg",
        action="store_true",
        help="Use JPEG LCM transport instead of regular LCM transport",
    )
    args = parser.parse_args()

    dimos = ModuleCoordinator(n=2)
    dimos.start()
    emitter = dimos.deploy(EmitterModule)
    receiver = dimos.deploy(ReceiverModule)

    if args.use_jpeg:
        emitter.image.transport = JpegLcmTransport("/go2/color_image", Image)
    else:
        emitter.image.transport = LCMTransport("/go2/color_image", Image)
    receiver.image.connect(emitter.image)

    foxglove_bridge = FoxgloveBridge()
    foxglove_bridge.start()

    dimos.start_all_modules()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        foxglove_bridge.stop()
        dimos.close()


if __name__ == "__main__":
    main()
