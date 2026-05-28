from __future__ import annotations

import asyncio
import datetime
import json
import logging
from pathlib import Path
from typing import Any, Awaitable, Callable

from demo_app.types import AlertEvent

logger = logging.getLogger(__name__)


def format_alert_caption(event: AlertEvent) -> str:
    ts = datetime.datetime.utcfromtimestamp(event.timestamp).strftime("%Y-%m-%d %H:%M:%S")
    x, y = event.robot_pose
    wp_part = f" near {event.nearest_waypoint}" if event.nearest_waypoint else ""
    labels = ""
    if event.evidence_detections:
        unique = sorted({det.class_name for det in event.evidence_detections})
        labels = f"\nEvidence: {', '.join(unique)}"
    lines = ["AISLE OBSTRUCTION DETECTED"]
    if event.obstruction_distance_m is not None:
        lines.append(f"Distance: {event.obstruction_distance_m:.2f} m")
    else:
        lines.append("Distance: unknown")
    lines.extend(
        [
            f"Direction: {event.obstruction_direction or 'center'}",
            f"Point count: {event.obstruction_point_count}",
            f"Robot pose: ~({x:.1f}, {y:.1f}) m{wp_part}",
            f"Time: {ts}",
        ]
    )
    if labels:
        lines.append(labels.removeprefix("\n"))
    return "\n".join(lines)


class TelegramBot:
    def __init__(
        self,
        token: str,
        owner_chat_id: int,
        on_patrol_command: Callable[[], Awaitable[None]],
        on_stop_command: Callable[[], Awaitable[None]],
        on_status_query: Callable[[], Awaitable[str]],
        failed_dir: Path | None = None,
    ) -> None:
        self._token = token
        self._owner = owner_chat_id
        self._on_patrol = on_patrol_command
        self._on_stop = on_stop_command
        self._on_status = on_status_query
        self._app: Any = None
        self._failed_dir = failed_dir or Path("failed_alerts")
        self._failed_dir.mkdir(parents=True, exist_ok=True)

    def _is_owner(self, update: Any) -> bool:
        chat_id = update.effective_chat.id if update.effective_chat is not None else None
        allowed = chat_id == self._owner
        if not allowed:
            logger.warning("Ignoring Telegram command from chat_id=%s; expected owner_chat_id=%s", chat_id, self._owner)
        return allowed

    async def _cmd_start(self, update: Any, ctx: Any) -> None:
        if not self._is_owner(update):
            return
        await update.message.reply_text(
            "Go2 demo ready.\n/patrol - start patrol\n/stop - stop patrol\n/status - current state"
        )

    async def _cmd_patrol(self, update: Any, ctx: Any) -> None:
        if not self._is_owner(update):
            return
        await self._on_patrol()
        await update.message.reply_text("Patrol started.")

    async def _cmd_stop(self, update: Any, ctx: Any) -> None:
        if not self._is_owner(update):
            return
        await self._on_stop()
        await update.message.reply_text("Stopping patrol.")

    async def _cmd_status(self, update: Any, ctx: Any) -> None:
        if not self._is_owner(update):
            return
        await update.message.reply_text(await self._on_status())

    async def start(self) -> None:
        try:
            from telegram.ext import Application, CommandHandler
        except Exception as e:
            raise RuntimeError(
                "python-telegram-bot is required for Telegram controls. "
                "Install it with: pip install python-telegram-bot[all]"
            ) from e

        self._app = Application.builder().token(self._token).build()
        self._app.add_handler(CommandHandler("start", self._cmd_start))
        self._app.add_handler(CommandHandler("patrol", self._cmd_patrol))
        self._app.add_handler(CommandHandler("stop", self._cmd_stop))
        self._app.add_handler(CommandHandler("status", self._cmd_status))
        await self._app.initialize()
        await self._app.start()
        await self._app.updater.start_polling()
        logger.info("Telegram bot polling started for owner_chat_id=%s", self._owner)
        try:
            await self._app.bot.send_message(
                self._owner,
                "Go2 demo bot connected. Available commands: /patrol /stop /status",
            )
        except Exception as e:
            logger.warning("Telegram startup message failed: %s", e)

    async def stop(self) -> None:
        if self._app is None:
            return
        await self._app.updater.stop()
        await self._app.stop()
        await self._app.shutdown()
        self._app = None

    async def send_photo_alert(self, event: AlertEvent, jpg_path: Path) -> bool:
        if self._app is None:
            self._dead_letter(event, jpg_path, None)
            return False

        caption = format_alert_caption(event)
        for delay in (0, 1, 2, 4):
            if delay:
                await asyncio.sleep(delay)
            try:
                with open(jpg_path, "rb") as photo:
                    await self._app.bot.send_photo(self._owner, photo=photo, caption=caption)
                return True
            except Exception as e:
                logger.warning("Telegram send failed after %ss: %s", delay, e)

        self._dead_letter(event, jpg_path, None)
        return False

    async def send_alert(self, event: AlertEvent, jpg_path: Path, mp4_path: Path) -> None:
        if self._app is None:
            self._dead_letter(event, jpg_path, mp4_path)
            return

        caption = format_alert_caption(event)
        for delay in (0, 1, 2, 4):
            if delay:
                await asyncio.sleep(delay)
            try:
                with open(jpg_path, "rb") as photo:
                    await self._app.bot.send_photo(self._owner, photo=photo, caption=caption)
                with open(mp4_path, "rb") as video:
                    await self._app.bot.send_video(self._owner, video=video)
                return
            except Exception as e:
                logger.warning("Telegram send failed after %ss: %s", delay, e)

        self._dead_letter(event, jpg_path, mp4_path)

    def _dead_letter(self, event: AlertEvent, jpg: Path, mp4: Path | None) -> None:
        path = self._failed_dir / f"alert_{int(event.timestamp * 1000)}.json"
        path.write_text(json.dumps({
            "timestamp": event.timestamp,
            "class_name": event.class_name,
            "confidence": event.confidence,
            "robot_pose": list(event.robot_pose),
            "nearest_waypoint": event.nearest_waypoint,
            "jpg": str(jpg),
            "mp4": str(mp4) if mp4 is not None else "",
        }))
        logger.error("Telegram alert saved to %s", path)
