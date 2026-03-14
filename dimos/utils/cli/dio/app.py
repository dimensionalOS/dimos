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

"""DIO — DimOS Unified TUI."""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import TYPE_CHECKING

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import RichLog, Static

from dimos.utils.cli import theme
from dimos.utils.cli.dio.sub_apps.registry import get_sub_apps

if TYPE_CHECKING:
    from textual.dom import DOMNode
    from textual.events import Click, Key, Resize
    from textual.timer import Timer
    from textual.widget import Widget

    from dimos.utils.cli.dio.sub_app import SubApp

_DUAL_WIDTH = 140  # >= this width: 2 panels
_TRIPLE_WIDTH = 220  # >= this width: 3 panels
_MAX_PANELS = 3
_QUIT_WINDOW = 1.5  # seconds to press again to confirm quit


class DIOApp(App[None]):
    CSS_PATH = "dio.tcss"

    BINDINGS = [
        Binding("alt+up", "tab_prev", "Tab prev", priority=True),
        Binding("alt+down", "tab_next", "Tab next", priority=True),
        Binding("ctrl+up", "tab_prev", "Tab prev", priority=True),
        Binding("ctrl+down", "tab_next", "Tab next", priority=True),
        Binding("alt+left", "focus_prev_panel", "Panel prev", priority=True),
        Binding("alt+right", "focus_next_panel", "Panel next", priority=True),
        Binding("ctrl+left", "focus_prev_panel", "Panel prev", priority=True),
        Binding("ctrl+right", "focus_next_panel", "Panel next", priority=True),
        Binding("escape", "quit_or_esc", "Quit", priority=True),
        Binding("ctrl+c", "copy_text", "Copy", priority=True),
        Binding("ctrl+q", "quit_or_esc", "Quit", priority=True),
    ]

    def __init__(self, *, debug: bool = False) -> None:
        super().__init__()
        # Register all DimOS themes
        for t in theme.get_textual_themes():
            self.register_theme(t)  # type: ignore[arg-type]
        # Load saved theme from config
        saved_theme = self._load_saved_theme()
        theme.set_theme(saved_theme)
        self.theme = f"dimos-{saved_theme}"
        self._debug = debug  # controls the visual debug panel only
        self._sub_app_classes = get_sub_apps()
        n = len(self._sub_app_classes)
        # Which sub-app index each panel shows
        self._panel_idx: list[int] = [i % n for i in range(_MAX_PANELS)]
        self._focused_panel: int = 0
        self._num_panels: int = 1  # how many panels are currently visible
        self._initialized = False
        self._instances: list[SubApp] = []
        self._quit_pressed_at: float = 0.0
        self._quit_timer: Timer | None = None
        # Track which panel each instance is currently mounted in
        self._instance_pane: dict[int, int] = {}  # instance_idx -> panel (0..N-1)
        # Debug log — always write to file, --debug only controls the visual panel
        from pathlib import Path

        log_path = Path.home() / ".dimos" / "dio-debug.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        f = open(log_path, "w")
        self._debug_log_path = str(log_path)
        self._debug_log_file = f

    @staticmethod
    def _load_saved_theme() -> str:
        """Read the saved theme name from dio-config.json, falling back to default."""
        import json
        from pathlib import Path

        try:
            config_path = Path(sys.prefix) / "dio-config.json"
            data = json.loads(config_path.read_text())
            name = str(data.get("theme", theme.DEFAULT_THEME))
            # Migrate old theme names
            _MIGRATION = {"dark": "dark-one"}
            name = _MIGRATION.get(name, name)
            if name in theme.THEME_NAMES:
                return name
        except Exception:
            pass
        return theme.DEFAULT_THEME

    def _dlog(self, msg: str) -> None:
        import re

        plain = re.sub(r"\[/?[^\]]*\]", "", msg)
        # Always write to debug log file
        if self._debug_log_file is not None:
            try:
                self._debug_log_file.write(plain + "\n")  # type: ignore[union-attr]
                self._debug_log_file.flush()  # type: ignore[union-attr]
            except Exception:
                pass
        # Only write to visual panel when --debug
        if self._debug:
            try:
                panel = self.query_one("#debug-log", RichLog)
                panel.write(msg)
            except Exception:
                pass

    def compose(self) -> ComposeResult:
        yield Static("", id="hint-bar")
        with Container(id="sidebar"):
            for i, cls in enumerate(self._sub_app_classes):
                yield Static(cls.TITLE, classes="tab-item", id=f"tab-{i}")
        with Horizontal(id="displays"):
            for p in range(_MAX_PANELS):
                yield Container(id=f"display-{p + 1}", classes="display-pane")
        if self._debug:
            yield RichLog(id="debug-log", markup=True, wrap=True, highlight=False)

    async def on_mount(self) -> None:
        self._instances = [cls() for cls in self._sub_app_classes]
        d1 = self.query_one("#display-1", Container)
        for i, inst in enumerate(self._instances):
            inst.styles.display = "none"
            await d1.mount(inst)
            self._instance_pane[i] = 0

        if not self._initialized:
            self._refresh_panel_count()
            self._initialized = True
        await self._place_instances()
        self._sync_tabs()
        self._sync_hint()
        if self._instances:
            self._force_focus_subapp(self._instances[self._panel_idx[0]])
        self._dlog(f"[dim]mounted {len(self._instances)} sub-apps, debug={self._debug}[/dim]")
        if self._debug_log_path:
            self._dlog(f"[dim]log file: {self._debug_log_path}[/dim]")

    async def on_resize(self, _event: Resize) -> None:
        old = self._num_panels
        self._refresh_panel_count()
        if old != self._num_panels:
            # Clamp focused panel
            if self._focused_panel >= self._num_panels:
                self._focused_panel = self._num_panels - 1
            await self._place_instances()
        self._sync_tabs()
        self._sync_hint()

    async def on_unmount(self) -> None:
        for inst in self._instances:
            inst.on_unmount_subapp()

    def _panel_for_widget(self, widget: Widget | None) -> int | None:
        """Return which panel (0..N-1) contains the given widget, or None."""
        node: DOMNode | None = widget
        while node is not None:
            node_id = getattr(node, "id", None) or ""
            if node_id.startswith("display-"):
                try:
                    p = int(node_id.split("-")[1]) - 1
                    return p if p < self._num_panels else None
                except (ValueError, IndexError):
                    return None
            node = node.parent
        return None

    async def on_click(self, event: Click) -> None:
        """Handle clicks on display panes (focus panel) and sidebar tabs (switch/focus sub-app)."""
        # Check if a sidebar tab was clicked
        tab_idx = self._tab_for_widget(event.widget)
        if tab_idx is not None:
            await self._on_tab_clicked(tab_idx)
            return
        # Otherwise, clicking a display pane focuses that panel
        panel = self._panel_for_widget(event.widget)
        if panel is not None and panel != self._focused_panel:
            self._focus_panel(panel)

    def _tab_for_widget(self, widget: Widget | None) -> int | None:
        """Return sub-app index if the widget is inside a sidebar tab, or None."""
        node: DOMNode | None = widget
        while node is not None:
            node_id = getattr(node, "id", None) or ""
            if node_id.startswith("tab-"):
                try:
                    return int(node_id.split("-")[1])
                except (ValueError, IndexError):
                    return None
            node = node.parent
        return None

    async def _on_tab_clicked(self, tab_idx: int) -> None:
        """Switch to or focus the sub-app at *tab_idx*."""
        # Check if this sub-app is already visible in any panel
        for p in range(self._num_panels):
            if self._panel_idx[p] == tab_idx:
                # Already visible — just focus that panel
                self._focus_panel(p)
                return
        # Not visible — switch the focused panel to show this sub-app
        self._panel_idx[self._focused_panel] = tab_idx
        await self._place_instances()
        self._sync_tabs()
        self._force_focus_subapp(self._instances[tab_idx])
        self._dlog(
            f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] tab_click -> show {self._sub_app_classes[tab_idx].TITLE} in panel {self._focused_panel}"
        )

    def on_key(self, event: Key) -> None:
        focused = self.focused
        focused_name = type(focused).__name__ if focused else "None"
        focused_id = getattr(focused, "id", None) or ""
        panel = self._panel_for_widget(focused)
        self._dlog(
            f"[{theme.DEBUG_KEY}]KEY[/{theme.DEBUG_KEY}] [bold {theme.CYAN}]{event.key!r}[/bold {theme.CYAN}]"
            f"  char={event.character!r}"
            f"  focused=[{theme.DEBUG_FOCUS}]{focused_name}#{focused_id}[/{theme.DEBUG_FOCUS}]"
            f"  _focused_panel={self._focused_panel}  actual_panel={panel}"
        )

    def _refresh_panel_count(self) -> None:
        w = self.size.width
        if w >= _TRIPLE_WIDTH:
            new_count = 3
        elif w >= _DUAL_WIDTH:
            new_count = 2
        else:
            new_count = 1
        # Don't show more panels than sub-apps
        new_count = min(new_count, len(self._sub_app_classes))
        self._num_panels = new_count

        for p in range(_MAX_PANELS):
            pane = self.query_one(f"#display-{p + 1}")
            pane.styles.display = "block" if p < new_count else "none"

    async def _place_instances(self) -> None:
        """Show/hide sub-apps and reparent into the correct display pane."""
        panes = [self.query_one(f"#display-{p + 1}", Container) for p in range(_MAX_PANELS)]

        # Build set of visible sub-app indices
        visible: dict[int, int] = {}  # instance_idx -> panel
        for p in range(self._num_panels):
            visible[self._panel_idx[p]] = p

        for i, inst in enumerate(self._instances):
            target_panel = visible.get(i)
            current_panel = self._instance_pane.get(i)

            if target_panel is not None:
                dest = panes[target_panel]
                if current_panel != target_panel:
                    if inst.parent is not None:
                        await inst.remove()
                    await dest.mount(inst)
                    self._instance_pane[i] = target_panel
                    self._dlog(
                        f"[dim]  moved {self._sub_app_classes[i].TITLE} -> panel{target_panel}[/dim]"
                    )
                inst.styles.display = "block"
            else:
                inst.styles.display = "none"

        names = " ".join(
            f"p{p}={self._sub_app_classes[self._panel_idx[p]].TITLE}"
            for p in range(self._num_panels)
        )
        self._dlog(f"[dim]placed: {names}[/dim]")

    _TAB_SELECTED_CLASSES = [f"--selected-{i}" for i in range(1, _MAX_PANELS + 1)]

    def _sync_tabs(self) -> None:
        for i in range(len(self._sub_app_classes)):
            tab = self.query_one(f"#tab-{i}", Static)
            tab.remove_class(*self._TAB_SELECTED_CLASSES)
            for p in range(self._num_panels):
                if i == self._panel_idx[p]:
                    tab.add_class(f"--selected-{p + 1}")

        # Panel focus borders
        for p in range(_MAX_PANELS):
            pane = self.query_one(f"#display-{p + 1}")
            pane.remove_class("--focused")
        target = self.query_one(f"#display-{self._focused_panel + 1}")
        target.add_class("--focused")

    def _sync_hint(self) -> None:
        bar = self.query_one("#hint-bar", Static)
        parts = ["Alt+Up/Down: switch tab"]
        if self._num_panels > 1:
            parts.append("Alt+Left/Right: switch panel")
        parts.append("Ctrl+C: copy")
        parts.append("Ctrl+Q/Esc: quit")
        bar.update(" | ".join(parts))

    async def action_tab_prev(self) -> None:
        self._dlog(
            f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] tab_prev  panel={self._focused_panel} idx={self._panel_idx[: self._num_panels]}"
        )
        self._clear_quit_pending()
        await self._move_tab(-1)

    async def action_tab_next(self) -> None:
        self._dlog(
            f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] tab_next  panel={self._focused_panel} idx={self._panel_idx[: self._num_panels]}"
        )
        self._clear_quit_pending()
        await self._move_tab(1)

    def action_focus_prev_panel(self) -> None:
        self._dlog(
            f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] focus_prev_panel  (was panel={self._focused_panel})"
        )
        self._clear_quit_pending()
        new = max(0, self._focused_panel - 1)
        self._focus_panel(new)

    def action_focus_next_panel(self) -> None:
        self._dlog(
            f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] focus_next_panel  (was panel={self._focused_panel})"
        )
        self._clear_quit_pending()
        new = min(self._num_panels - 1, self._focused_panel + 1)
        self._focus_panel(new)

    def action_copy_text(self) -> None:
        """Copy selected text to clipboard, or quit if no selection."""
        selected = self.screen.get_selected_text()
        if selected:
            self.copy_to_clipboard(selected)
            self.screen.clear_selection()
            self._dlog(
                f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] copy_text (copied to clipboard)"
            )
        else:
            self._dlog(
                f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] copy_text -> no selection, treating as quit"
            )
            self._handle_quit_press()

    def action_quit_or_esc(self) -> None:
        self._dlog(f"[{theme.DEBUG_ACTION}]ACTION[/{theme.DEBUG_ACTION}] quit_or_esc")
        self._handle_quit_press()

    def _focus_panel(self, panel: int) -> None:
        old = self._focused_panel
        self._focused_panel = panel
        idx = self._panel_idx[panel]
        self._force_focus_subapp(self._instances[idx])
        self._sync_tabs()
        self._sync_hint()
        # Log what actually got focus
        actual = self.focused
        actual_name = type(actual).__name__ if actual else "None"
        actual_id = getattr(actual, "id", None) or ""
        actual_panel = self._panel_for_widget(actual)
        self._dlog(
            f"  -> FOCUS panel {old}->{panel}  sub-app={self._sub_app_classes[idx].TITLE}"
            f"  actual_focus={actual_name}#{actual_id} in panel={actual_panel}"
        )

    def _force_focus_subapp(self, subapp: SubApp) -> None:
        """Force focus onto a widget inside the given sub-app.

        Widget.focus() can silently fail when an Input already has focus.
        We find the target focusable widget and use screen.set_focus() directly.
        """
        target = subapp.get_focus_target()
        if target is not None:
            self.screen.set_focus(target)
        else:
            self._dlog(f"[dim]WARNING: no focusable widget in {subapp.TITLE}[/dim]")

    async def _move_tab(self, delta: int) -> None:
        n = len(self._sub_app_classes)
        panel = self._focused_panel
        old_idx = self._panel_idx[panel]
        idx = (old_idx + delta) % n

        # Skip indices shown in other panels (a widget can't be in two panes)
        other_indices = {self._panel_idx[p] for p in range(self._num_panels) if p != panel}
        attempts = 0
        while idx in other_indices and attempts < n:
            idx = (idx + delta) % n
            attempts += 1

        self._panel_idx[panel] = idx
        self._dlog(
            f"  -> MOVE panel={panel} {self._sub_app_classes[old_idx].TITLE}->{self._sub_app_classes[idx].TITLE} "
            f"idx={self._panel_idx[: self._num_panels]}"
        )
        await self._place_instances()
        self._sync_tabs()
        self._force_focus_subapp(self._instances[idx])
        actual = self.focused
        actual_name = type(actual).__name__ if actual else "None"
        actual_id = getattr(actual, "id", None) or ""
        self._dlog(
            f"  -> after focus: {actual_name}#{actual_id} in panel={self._panel_for_widget(actual)}"
        )

    def _handle_quit_press(self) -> None:
        now = time.monotonic()
        if now - self._quit_pressed_at < _QUIT_WINDOW:
            self.exit()
            return
        self._quit_pressed_at = now
        bar = self.query_one("#hint-bar", Static)
        bar.update("Press Esc or Ctrl+Q again to exit")
        if self._quit_timer is not None:
            self._quit_timer.stop()
        self._quit_timer = self.set_timer(_QUIT_WINDOW, self._clear_quit_pending)

    def _clear_quit_pending(self) -> None:
        self._quit_pressed_at = 0.0
        if self._quit_timer is not None:
            self._quit_timer.stop()
            self._quit_timer = None
        if self._initialized:
            self._sync_hint()

    # _pending_confirms maps message -> (event, result_list) so that
    # concurrent threads asking the same question share one modal.
    _pending_confirms: dict[str, tuple[threading.Event, list[bool]]] = {}
    _pending_confirms_lock = threading.Lock()

    def _handle_confirm(self, message: str, default: bool) -> bool | None:
        from dimos.utils.cli.dio.confirm_screen import ConfirmScreen

        with self._pending_confirms_lock:
            if message not in self._pending_confirms:
                event = threading.Event()
                result: list[bool] = []
                self._pending_confirms[message] = (event, result)

                def _push() -> None:
                    def _on_result(value: bool | None) -> None:
                        if value is not None:
                            result.append(value)
                        event.set()

                    self.push_screen(ConfirmScreen(message, default), callback=_on_result)

                self.call_from_thread(_push)
            else:
                # Another thread is already showing this question — wait for it
                event, result = self._pending_confirms[message]

        event.wait()

        # First thread to wake up after the modal cleans up the entry
        with self._pending_confirms_lock:
            self._pending_confirms.pop(message, None)

        return result[0] if result else default

    _pending_sudos: dict[str, tuple[threading.Event, list[bool]]] = {}
    _pending_sudos_lock = threading.Lock()

    def _handle_sudo(self, message: str) -> bool | None:
        from dimos.utils.cli.dio.confirm_screen import SudoScreen

        with self._pending_sudos_lock:
            if message not in self._pending_sudos:
                event = threading.Event()
                result: list[bool] = []
                self._pending_sudos[message] = (event, result)

                def _push() -> None:
                    def _on_result(value: bool | None) -> None:
                        if value is not None:
                            result.append(value)
                        event.set()

                    self.push_screen(SudoScreen(message), callback=_on_result)

                self.call_from_thread(_push)
            else:
                event, result = self._pending_sudos[message]

        event.wait()

        with self._pending_sudos_lock:
            self._pending_sudos.pop(message, None)

        return result[0] if result else False


def main() -> None:
    from dimos.utils.prompt import clear_dio_hook, set_dio_hook, set_dio_sudo_hook

    debug = "--debug" in sys.argv
    if debug:
        sys.argv.remove("--debug")

    app = DIOApp(debug=debug)
    set_dio_hook(app._handle_confirm)
    set_dio_sudo_hook(app._handle_sudo)

    _real_stdin = sys.stdin
    sys.stdin = open(os.devnull)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        clear_dio_hook()
        sys.stdin.close()
        sys.stdin = _real_stdin
        print(f"Debug log: {app._debug_log_path}")
        if app._debug_log_file:
            try:
                app._debug_log_file.close()  # type: ignore[union-attr]
            except Exception:
                pass
        os._exit(0)


if __name__ == "__main__":
    main()
