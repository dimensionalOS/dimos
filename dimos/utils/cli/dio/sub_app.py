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

"""SubApp base class for DIO sub-applications."""

from __future__ import annotations

from textual.dom import DOMNode
from textual.widget import Widget


class SubApp(Widget):
    """Base class for DIO sub-applications.

    Each sub-app is a Widget that renders inside a display pane.
    Subclasses must set TITLE and implement compose().

    Lifecycle:
      - on_mount_subapp() is called exactly ONCE after the widget's
        children have been composed.  Heavy / blocking work (LCM
        connections, etc.) should be dispatched via self.run_worker().
      - on_resume_subapp() is called on every subsequent remount
        (e.g. when the widget is moved between display panels).
        Use this to restart timers killed by remove().
      - on_unmount_subapp() is called when the DIO app is shutting down,
        NOT on every tab switch.
    """

    TITLE: str = "Untitled"

    can_focus = False

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)  # type: ignore[arg-type]
        self._subapp_initialized = False

    @property
    def has_focus(self) -> bool:  # type: ignore[override]
        """True if the currently focused widget is inside this sub-app."""
        focused = self.app.focused
        if focused is None:
            return False
        # Walk up the DOM tree to see if focused widget is a descendant
        node: DOMNode | None = focused
        while node is not None:
            if node is self:
                return True
            node = node.parent
        return False

    def get_focus_target(self) -> Widget | None:
        """Return the widget that should receive focus for this sub-app.

        Override in subclasses for custom focus logic.
        Default: first visible focusable descendant.
        """
        for child in self.query("*"):
            if child.can_focus and child.display and child.styles.display != "none":
                return child
        return None

    def on_mount(self) -> None:
        """Textual lifecycle — fires after compose() children exist.

        Fires on EVERY mount (including after remove+remount when moving
        between display panels). First mount triggers on_mount_subapp();
        subsequent mounts trigger on_resume_subapp().
        """
        if not self._subapp_initialized:
            self._subapp_initialized = True
            self.on_mount_subapp()
        else:
            self.on_resume_subapp()

    def on_mount_subapp(self) -> None:
        """Called exactly once after first mount.

        Override to start LCM subscriptions, timers, etc.
        Heavy / blocking work should use ``self.run_worker()``.
        """

    def on_resume_subapp(self) -> None:
        """Called on every remount after the first.

        Override to restart timers that were killed when the widget
        was removed from the DOM (e.g. during panel rearrangement).
        """

    def on_unmount_subapp(self) -> None:
        """Called when the DIO app tears down this sub-app.

        Override to stop LCM subscriptions, timers, etc.
        """

    def reinit_lcm(self) -> None:
        """Called after autoconf changes network config (e.g. multicast).

        Sub-apps that hold LCM connections should override this to
        tear down and recreate them, since connections created before
        multicast was configured will be dead.
        """
