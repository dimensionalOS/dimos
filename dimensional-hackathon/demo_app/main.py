from __future__ import annotations

import asyncio
import logging
import signal
import time
from pathlib import Path
from typing import Any

import numpy as np
import uvicorn

from demo_app.aisle import AisleCorridorDetector
from demo_app.audio import AudioAlert
from demo_app.capture import CaptureBuffer
from demo_app.config import load_config
from demo_app.dashboard import create_app
from demo_app.detector import YoloToolDetector
from demo_app.patrol import PatrolController, nearest_waypoint
from demo_app.robot import Go2Runner
from demo_app.telegram_bot import TelegramBot
from demo_app.types import AisleObservation, AlertEvent, Detection, Waypoint
from demo_app.vision_obstruction import CenterLaneVisionDetector

logger = logging.getLogger(__name__)


async def run() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    logger.info("Loading config from config.yaml")
    cfg = load_config(Path("config.yaml"))
    logger.info("Config loaded")
    waypoints = [Waypoint(id=w.id, pos=tuple(w.pos), yaw_deg=w.yaw) for w in cfg.waypoints]

    logger.info("Creating dashboard state")
    web_state: dict[str, Any]
    command_handlers: dict[str, Any] = {}
    app, web_state = create_app(stream_fps=cfg.web.stream_fps, command_handlers=command_handlers)
    web_state["waypoints"] = [{"id": w.id, "pos": list(w.pos), "yaw": w.yaw_deg} for w in waypoints]

    runner = Go2Runner(
        robot_ip=cfg.robot.ip,
        obstacle_avoidance=cfg.robot.obstacle_avoidance,
        camera_resize=tuple(cfg.robot.camera_resize),
        connect_timeout_sec=cfg.robot.connect_timeout_sec,
    )
    logger.info("Starting Go2 runner")
    runner.start()
    logger.info("Go2 runner started")

    patrol = PatrolController(
        runner=runner,
        waypoints=waypoints,
        web_state=web_state,
        loop_forever=cfg.patrol.loop_forever,
        scan_turns=cfg.patrol.scan_turns,
        scan_pause_sec=cfg.patrol.scan_pause_sec,
        motion_settle_sec=cfg.patrol.motion_settle_sec,
        forward_steps_per_cycle=cfg.patrol.forward_steps_per_cycle,
        forward_speed_mps=cfg.patrol.forward_speed_mps,
        forward_step_duration_sec=cfg.patrol.forward_step_duration_sec,
        sweep_yaw_radps=cfg.patrol.sweep_yaw_radps,
        sweep_turn_duration_sec=cfg.patrol.sweep_turn_duration_sec,
    )

    capture = CaptureBuffer(
        buffer_seconds=cfg.alert.buffer_seconds,
        fps=cfg.alert.capture_fps,
        output_dir=Path("captures"),
    )
    audio = AudioAlert(audio_file=cfg.alert.audio_file, runner=runner)

    detector: YoloToolDetector | None = None
    latest_lidar_points: np.ndarray | None = None
    latest_observation: AisleObservation | None = None
    obstruction_task: asyncio.Task | None = None
    obstruction_latched = False
    approach_abort_requested = False
    last_obstruction_alert_ts = 0.0
    last_pose_record = 0.0
    last_no_lidar_log = 0.0
    last_lidar_parse_log = 0.0
    last_aisle_debug_log = 0.0
    clear_streak = 0
    last_vision_debug_log = 0.0

    aisle_detector = AisleCorridorDetector(
        x_min_m=cfg.aisle.x_min_m,
        x_max_m=cfg.aisle.x_max_m,
        half_width_m=cfg.aisle.half_width_m,
        min_points_in_zone=cfg.aisle.min_points_in_zone,
        min_z_m=cfg.aisle.min_z_m,
        max_z_m=cfg.aisle.max_z_m,
        cell_size_m=cfg.aisle.cell_size_m,
        min_occupied_cells=cfg.aisle.min_occupied_cells,
    )
    vision_detector = CenterLaneVisionDetector()
    logger.info(
        "Aisle detector: x=[%.2f, %.2f] half_width=%.2f z=[%.2f, %.2f] min_points=%d min_cells=%d repeat=%.2fs",
        cfg.aisle.x_min_m,
        cfg.aisle.x_max_m,
        cfg.aisle.half_width_m,
        cfg.aisle.min_z_m,
        cfg.aisle.max_z_m,
        cfg.aisle.min_points_in_zone,
        cfg.aisle.min_occupied_cells,
        cfg.aisle.alert_repeat_sec,
    )

    def push_event(event_type: str, payload: dict[str, Any]) -> None:
        web_state.setdefault("event_log", []).append({"type": event_type, "payload": payload})

    def set_mode(mode: str) -> None:
        if web_state.get("mode") == mode:
            return
        web_state["mode"] = mode
        push_event("state", {"mode": mode})

    async def stop_patrol_and_abort() -> None:
        nonlocal approach_abort_requested
        approach_abort_requested = True
        await patrol.stop(return_home=False)

    async def on_patrol() -> None:
        nonlocal approach_abort_requested
        approach_abort_requested = False
        await patrol.start()

    async def on_stop() -> None:
        await stop_patrol_and_abort()
        if web_state.get("mode") == "APPROACHING":
            set_mode("IDLE")

    async def on_status() -> str:
        return await patrol.status()

    async def command_forward() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: forward")
        ok = await patrol.manual_drive(forward_mps=0.25, duration_sec=1.0)
        return "ok" if ok else "failed"

    async def command_back() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: back")
        ok = await patrol.manual_drive(forward_mps=-0.20, duration_sec=1.0)
        return "ok" if ok else "failed"

    async def command_left() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: left")
        ok = await patrol.manual_drive(left_mps=0.15, duration_sec=1.0)
        return "ok" if ok else "failed"

    async def command_right() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: right")
        ok = await patrol.manual_drive(left_mps=-0.15, duration_sec=1.0)
        return "ok" if ok else "failed"

    async def command_turn_left() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: turn_left")
        ok = await patrol.manual_drive(yaw_radps=0.50, duration_sec=0.8)
        return "ok" if ok else "failed"

    async def command_turn_right() -> str:
        await stop_patrol_and_abort()
        logger.info("Dashboard command: turn_right")
        ok = await patrol.manual_drive(yaw_radps=-0.50, duration_sec=0.8)
        return "ok" if ok else "failed"

    command_handlers.update(
        {
            "patrol": on_patrol,
            "stop": on_stop,
            "status": on_status,
            "forward": command_forward,
            "back": command_back,
            "left": command_left,
            "right": command_right,
            "turn_left": command_turn_left,
            "turn_right": command_turn_right,
        }
    )

    telegram = TelegramBot(
        token=cfg.telegram_bot_token,
        owner_chat_id=cfg.telegram_owner_chat_id,
        on_patrol_command=on_patrol,
        on_stop_command=on_stop,
        on_status_query=on_status,
    )

    def on_frame(frame: Any) -> None:
        ts = time.time()
        web_state["latest_frame"] = frame
        capture.push(frame, ts)

    def on_lidar(msg: Any) -> None:
        nonlocal latest_lidar_points
        nonlocal last_lidar_parse_log
        try:
            if hasattr(msg, "points_f32"):
                points = msg.points_f32()
            elif hasattr(msg, "pointcloud_tensor"):
                points = msg.pointcloud_tensor.point["positions"].numpy()
            else:
                raise TypeError(f"Unsupported lidar message type: {type(msg)!r}")
            arr = np.asarray(points, dtype=np.float32)
            if arr.ndim != 2 or arr.shape[1] < 2:
                raise ValueError(f"Unexpected lidar shape: {arr.shape!r}")
            latest_lidar_points = arr[:, :3]
        except Exception as exc:
            now = time.time()
            if now - last_lidar_parse_log >= 3.0:
                logger.warning("Failed to parse lidar frame: %s", exc)
                last_lidar_parse_log = now

    video_sub = runner.video_stream().subscribe(on_next=on_frame)
    lidar_sub = runner.lidar_stream().subscribe(on_next=on_lidar)

    stop_event = asyncio.Event()

    async def ensure_detector() -> YoloToolDetector:
        nonlocal detector
        if detector is None:
            logger.info("Initializing YOLO evidence detector: %s", cfg.detection.model_name)
            detector = await asyncio.to_thread(
                YoloToolDetector,
                cfg.detection.detection_classes,
                cfg.detection.conf_threshold,
                cfg.detection.model_name,
            )
            logger.info(
                "YOLO evidence detector ready. Watching classes=%s conf_threshold=%.2f",
                cfg.detection.detection_classes,
                cfg.detection.conf_threshold,
            )
        return detector

    async def detect_evidence(frame: np.ndarray, timestamp: float) -> list[Detection]:
        if not cfg.detection.enabled:
            web_state["latest_detections"] = []
            return []
        model = await ensure_detector()
        detections = await asyncio.to_thread(model.detect, frame, timestamp)
        if detections:
            logger.info(
                "YOLO evidence detections: %s",
                ", ".join(f"{det.class_name}@{det.confidence:.2f}" for det in detections),
            )
        else:
            logger.info("YOLO evidence detector saw 0 watched objects in obstruction frame")
        web_state["latest_detections"] = [
            {
                "bbox": list(det.bbox),
                "class_name": det.class_name,
                "confidence": det.confidence,
                "timestamp": det.timestamp,
            }
            for det in detections
        ]
        return detections

    async def handle_obstruction(initial_observation: AisleObservation) -> None:
        nonlocal obstruction_latched
        observation = initial_observation
        waypoint_id = patrol.current_waypoint() or nearest_waypoint(runner.get_pose(), waypoints)
        push_event(
            "corridor_blocked",
            {
                "distance_m": observation.obstruction_distance_m,
                "direction": observation.obstruction_direction,
                "point_count": observation.obstruction_point_count,
                "waypoint": waypoint_id,
            },
        )
        logger.info(
            "Corridor blocked: distance=%s direction=%s points=%d waypoint=%s",
            f"{observation.obstruction_distance_m:.2f}m"
            if observation.obstruction_distance_m is not None
            else "unknown",
            observation.obstruction_direction,
            observation.obstruction_point_count,
            waypoint_id,
        )
        set_mode("OBSTRUCTION")
        push_event(
            "approach_started",
            {
                "distance_m": observation.obstruction_distance_m,
                "direction": observation.obstruction_direction,
                "mode": "capture_only",
            },
        )

        frame = web_state.get("latest_frame")
        if frame is None:
            logger.warning("No camera frame available for obstruction capture")
            set_mode("OBSTRUCTION")
            return

        timestamp = time.time()
        evidence = await detect_evidence(frame, timestamp)
        pose = runner.get_pose()
        waypoint_id = patrol.current_waypoint() or nearest_waypoint(pose, waypoints)
        primary = evidence[0] if evidence else None
        event = AlertEvent(
            bbox=primary.bbox if primary is not None else None,
            class_name=primary.class_name if primary is not None else "obstruction",
            confidence=primary.confidence if primary is not None else 1.0,
            frame=frame.copy(),
            robot_pose=(pose.x, pose.y),
            nearest_waypoint=waypoint_id,
            timestamp=timestamp,
            obstruction_distance_m=observation.obstruction_distance_m,
            obstruction_direction=observation.obstruction_direction,
            obstruction_point_count=observation.obstruction_point_count,
            evidence_detections=evidence or None,
        )
        web_state["anomalies"].append(
            {
                "class_name": event.class_name,
                "robot_pose": list(event.robot_pose),
                "nearest_waypoint": event.nearest_waypoint,
                "timestamp": event.timestamp,
            }
        )
        jpg_path = await capture.snapshot(event)
        logger.info("Saved obstruction JPG: %s", jpg_path)
        push_event(
            "capture_saved",
            {
                "class_name": event.class_name,
                "jpg": str(jpg_path),
            },
        )

        sent = await telegram.send_photo_alert(event, jpg_path)
        push_event(
            "telegram_sent" if sent else "telegram_failed",
            {
                "class_name": event.class_name,
                "jpg": str(jpg_path),
            },
        )
        asyncio.create_task(audio.play(event))
        if patrol.is_running():
            set_mode("PATROLLING")
        else:
            set_mode("IDLE")

    async def monitor_loop() -> None:
        nonlocal clear_streak
        nonlocal last_no_lidar_log
        nonlocal last_pose_record
        nonlocal latest_observation
        nonlocal obstruction_latched
        nonlocal obstruction_task
        nonlocal last_obstruction_alert_ts
        nonlocal last_aisle_debug_log
        nonlocal last_vision_debug_log

        loop_interval = 0.25
        while not stop_event.is_set():
            await asyncio.sleep(loop_interval)

            pose = runner.get_pose()
            web_state["robot_pose"] = {"x": pose.x, "y": pose.y, "yaw_deg": pose.yaw_deg}
            now = time.time()
            if now - last_pose_record >= 0.5:
                history = web_state.setdefault("pose_history", [])
                history.append([pose.x, pose.y])
                if len(history) > 400:
                    del history[: len(history) - 400]
                last_pose_record = now

            if obstruction_task is not None and obstruction_task.done():
                try:
                    await obstruction_task
                except asyncio.CancelledError:
                    pass
                except Exception:
                    logger.exception("Obstruction task failed")
                obstruction_task = None

            if not cfg.aisle.enabled:
                continue

            patrol_phase = str(web_state.get("patrol_phase", "IDLE"))
            if patrol_phase not in {"FORWARD_STEP", "PATROLLING"}:
                latest_observation = None
                clear_streak = 0
                obstruction_latched = False
                web_state["corridor_clear"] = True
                web_state["obstruction_distance_m"] = None
                web_state["obstruction_direction"] = "center"
                web_state["obstruction_point_count"] = 0
                continue

            points = latest_lidar_points
            if points is None:
                if now - last_no_lidar_log >= 3.0:
                    logger.info("Aisle monitor waiting for lidar frames")
                    last_no_lidar_log = now
                continue

            latest_observation = await asyncio.to_thread(aisle_detector.analyze, points)
            vision_observation = None
            frame = web_state.get("latest_frame")
            if frame is not None:
                vision_observation = await asyncio.to_thread(vision_detector.detect, frame)
            web_state["corridor_clear"] = latest_observation.corridor_clear
            web_state["obstruction_distance_m"] = latest_observation.obstruction_distance_m
            web_state["obstruction_direction"] = latest_observation.obstruction_direction
            web_state["obstruction_point_count"] = latest_observation.obstruction_point_count
            if now - last_aisle_debug_log >= 2.0:
                logger.info(
                    "Aisle sample: clear=%s points=%d cells=%d dist=%s dir=%s",
                    latest_observation.corridor_clear,
                    latest_observation.obstruction_point_count,
                    latest_observation.occupied_cell_count,
                    f"{latest_observation.obstruction_distance_m:.2f}m"
                    if latest_observation.obstruction_distance_m is not None
                    else "-",
                    latest_observation.obstruction_direction,
                )
                last_aisle_debug_log = now
            if vision_observation is not None and now - last_vision_debug_log >= 2.0:
                logger.info(
                    "Vision lane sample: blocked=%s area=%.0f bbox=%s",
                    vision_observation.blocked,
                    vision_observation.contour_area,
                    vision_observation.bbox,
                )
                last_vision_debug_log = now

            effective_observation = latest_observation
            if latest_observation.corridor_clear and vision_observation is not None and vision_observation.blocked:
                effective_observation = AisleObservation(
                    corridor_clear=False,
                    obstruction_distance_m=1.0,
                    obstruction_direction="center",
                    obstruction_point_count=max(1, int(vision_observation.contour_area // 400)),
                    occupied_cell_count=1,
                    obstruction_center_xy=(1.0, 0.0),
                )
                web_state["corridor_clear"] = False
                web_state["obstruction_distance_m"] = effective_observation.obstruction_distance_m
                web_state["obstruction_direction"] = effective_observation.obstruction_direction
                web_state["obstruction_point_count"] = effective_observation.obstruction_point_count

            if effective_observation.corridor_clear:
                clear_streak += 1
                if clear_streak >= cfg.aisle.reclear_consecutive_frames and obstruction_latched:
                    obstruction_latched = False
                    if web_state.get("mode") == "OBSTRUCTION":
                        set_mode("PATROLLING" if patrol.is_running() else "IDLE")
                    push_event("corridor_cleared", {"point_count": 0})
                continue

            clear_streak = 0
            if obstruction_task is not None:
                continue
            if not patrol.is_running():
                continue
            if obstruction_latched and now - last_obstruction_alert_ts < cfg.aisle.alert_repeat_sec:
                continue

            obstruction_latched = True
            last_obstruction_alert_ts = now
            obstruction_task = asyncio.create_task(handle_obstruction(effective_observation))

    logger.info("Starting Telegram bot")
    await telegram.start()
    logger.info("Telegram bot started")
    push_event("telegram", {"message": f"owner_chat_id={cfg.telegram_owner_chat_id}"})

    server = uvicorn.Server(
        uvicorn.Config(app, host=cfg.web.host, port=cfg.web.port, log_level="info", loop="asyncio")
    )
    logger.info("Starting web dashboard on http://%s:%s", cfg.web.host, cfg.web.port)
    server_task = asyncio.create_task(server.serve())
    monitor_task = asyncio.create_task(monitor_loop())

    shutdown = asyncio.Event()
    loop = asyncio.get_running_loop()

    def request_shutdown() -> None:
        shutdown.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, request_shutdown)
        except (NotImplementedError, RuntimeError):
            signal.signal(sig, lambda *_: request_shutdown())

    await shutdown.wait()

    stop_event.set()
    monitor_task.cancel()
    try:
        await monitor_task
    except asyncio.CancelledError:
        pass
    except Exception:
        pass

    if obstruction_task is not None:
        obstruction_task.cancel()
        try:
            await obstruction_task
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    try:
        await patrol.stop(return_home=False)
    except Exception:
        pass

    try:
        video_sub.dispose()
    except Exception:
        pass

    try:
        lidar_sub.dispose()
    except Exception:
        pass

    try:
        await telegram.stop()
    except Exception:
        pass

    try:
        server.should_exit = True
        await server_task
    except Exception:
        pass

    runner.stop()


if __name__ == "__main__":
    asyncio.run(run())
