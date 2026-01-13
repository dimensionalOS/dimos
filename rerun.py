# Copyright 2026 Dimensional Inc.
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

from dashboard import RerunConnect, RerunOptions, rerun_module

import rerun as rr


# basic
class Mod1(Module):
    color_image: Out[Image] = RerunOptions(entity="color_image", rate_limit=1 / 60)
    depth_image: Out[Image] = RerunOptions(disable=True)

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    ...


# full custom
class Mod2(Module):
    color_image: Out[Image] = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        self.rc = RerunConnect(self)  # forces rr.init to happen
        self.color_cycle = 0

        def callback_func(img: Image) -> None:
            self.color_cycle += 1
            if self.color_cycle > 10:
                self.color_cycle = 0

            customized_image = img.hue_shift(offset=self.color_cycle * 0.1 * 256)
            self.rc.log("cam1/color", customized_image.to_rerun())

        ...


# LATER: enable custom rrb.blueprints (layout)
# LATER: get rid of RerunConnect (just use rr.log thanks to rr.init inside of dimos core)

autoconnect(
    Mod1.blueprint(),
    Mod2.blueprint(),
    # rerun only ope up if added here:
    rerun_module(
        open_rerun=True,
        open_browser=False,
        disabled=False,
    ),
).build().loop()
