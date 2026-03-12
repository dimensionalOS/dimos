"""Modal confirmation and sudo screens for dio."""

from __future__ import annotations

import subprocess

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Center, Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Button, Input, Label, Static


class ConfirmScreen(ModalScreen[bool]):
    """A modal popup that asks a yes/no question."""

    DEFAULT_CSS = """
    ConfirmScreen {
        align: center middle;
        background: rgba(0, 0, 0, 0.7);
    }

    ConfirmScreen > Vertical {
        width: 60;
        height: auto;
        max-height: 20;
        border: solid $dio-accent;
        background: $dio-bg;
        padding: 1 2;
    }

    ConfirmScreen.--warning > Vertical {
        border: solid $dio-yellow;
    }

    ConfirmScreen.--warning > Vertical > Label {
        color: $dio-yellow;
    }

    ConfirmScreen > Vertical > Label {
        width: 100%;
        content-align: center middle;
        color: $dio-text;
        margin-bottom: 1;
    }

    ConfirmScreen > Vertical > Center {
        width: 100%;
        height: auto;
    }

    ConfirmScreen Button {
        margin: 0 2;
        min-width: 14;
        background: transparent;
        color: $dio-dim;
        border: none;
    }

    ConfirmScreen Button:focus {
        background: $dio-accent;
        color: $dio-bg;
        border: solid $dio-accent;
        text-style: bold;
    }

    ConfirmScreen Button:hover {
        background: $dio-accent;
        color: $dio-bg;
        border: solid $dio-accent;
    }

    ConfirmScreen.--warning Button:focus {
        background: $dio-yellow;
        color: $dio-bg;
        border: solid $dio-yellow;
    }

    ConfirmScreen.--warning Button:hover {
        background: $dio-yellow;
        color: $dio-bg;
        border: solid $dio-yellow;
    }
    """

    BINDINGS = [
        Binding("y", "yes", "Yes", priority=True),
        Binding("n", "no", "No", priority=True),
        Binding("escape", "no", "No", priority=True),
        Binding("ctrl+c", "no", "No", priority=True),
        Binding("enter", "submit", "Submit", priority=True),
        Binding("left", "switch_btn", "Left", priority=True),
        Binding("right", "switch_btn", "Right", priority=True),
        Binding("up", "switch_btn", "Up", priority=True),
        Binding("down", "switch_btn", "Down", priority=True),
        Binding("tab", "switch_btn", "Tab", priority=True),
    ]

    def __init__(self, message: str, default: bool = False, warning: bool = False) -> None:
        super().__init__()
        self._message = message
        self._default = default
        self._warning = warning

    def compose(self) -> ComposeResult:
        if self._warning:
            self.add_class("--warning")
        with Vertical():
            yield Label(self._message)
            with Center():
                yield Button("Yes", id="btn-yes")
                yield Button("No", id="btn-no")

    def on_mount(self) -> None:
        btn_id = "#btn-yes" if self._default else "#btn-no"
        self.query_one(btn_id, Button).focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "btn-yes")

    def action_yes(self) -> None:
        self.dismiss(True)

    def action_no(self) -> None:
        self.dismiss(False)

    def action_submit(self) -> None:
        focused = self.focused
        if isinstance(focused, Button):
            self.dismiss(focused.id == "btn-yes")

    def action_switch_btn(self) -> None:
        btn_yes = self.query_one("#btn-yes", Button)
        btn_no = self.query_one("#btn-no", Button)
        if self.focused is btn_yes:
            btn_no.focus()
        else:
            btn_yes.focus()


class SudoScreen(ModalScreen[bool]):
    """A modal popup that asks for a sudo password and caches credentials."""

    DEFAULT_CSS = """
    SudoScreen {
        align: center middle;
        background: rgba(0, 0, 0, 0.7);
    }

    SudoScreen > Vertical {
        width: 50;
        height: auto;
        max-height: 14;
        border: solid $dio-yellow;
        background: $dio-bg;
        padding: 1 2;
    }

    SudoScreen > Vertical > Label {
        width: 100%;
        content-align: center middle;
        color: $dio-text;
        margin-bottom: 1;
    }

    SudoScreen > Vertical > #sudo-error {
        color: $dio-red;
        width: 100%;
        content-align: center middle;
        margin-top: 1;
    }

    SudoScreen Input {
        width: 100%;
    }
    """

    BINDINGS = [
        Binding("escape", "cancel", "Cancel"),
        Binding("ctrl+c", "cancel", "Cancel"),
    ]

    def __init__(self, message: str = "sudo password required") -> None:
        super().__init__()
        self._message = message

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label(self._message)
            yield Input(placeholder="Password", password=True, id="sudo-input")
            yield Static("", id="sudo-error")

    def on_mount(self) -> None:
        self.query_one("#sudo-input", Input).focus()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "sudo-input":
            return
        password = event.value
        if not password:
            return

        # Validate by running sudo -S true with the password on stdin
        result = subprocess.run(
            ["sudo", "-S", "true"],
            input=password + "\n",
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            self.dismiss(True)
        else:
            event.input.value = ""
            error = self.query_one("#sudo-error", Static)
            error.update("Incorrect password, try again")

    def action_cancel(self) -> None:
        self.dismiss(False)
