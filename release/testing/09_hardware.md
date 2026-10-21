# 9. Hardware checks

The Go2 checks require a connected robot and room to perform the described movements. Agentic checks need the configured credentials. Audio checks need a microphone and speakers. The camera checks need a webcam and a RealSense. Follow the [Go2 setup guide](../../docs/platforms/quadruped/go2/index.md) and the shared setup in the release testing plan.

Run each foreground blueprint separately. Use another terminal for commands that interact with it, and stop it before moving to the next check.

## 9.1 Test launching multiple blueprints

```bash
uv run dimos run unitree-go2 keyboard-teleop
```

You should get the Keyboard Teleop window and be able to control the robot from there.

When you Ctrl+C `dimos`, the robot should lay down.

## 9.2 Commanding the robot from rerun

```bash
uv run dimos run unitree-go2
```

* Click on a point on the 3D view in the Rerun window. The robot should travel there.
* Control the robot with WASD. (This is a bit finicky.)

## 9.3 Agentic movement skills

```bash
uv run dimos run unitree-go2-agentic
uv run humancli
```

And tell it to "move forward 2 meters" and then "move back 3 meters"

Expected result: the robot performs each requested movement.

## 9.4 dimos agent-send

```bash
uv run dimos run unitree-go2-agentic
# Should show up in humancli too
uv run dimos agent-send "move forward 4 meters"

# You should hear it speak in the computer's speakers.
uv run dimos agent-send "say hello"
```

Expected result: the robot moves forward, the command appears in `humancli` when it is running, and the greeting plays through the speakers.

## 9.5 Audio commands

```bash
uv run dimos run unitree-go2-agentic
```

* Go to http://localhost:5555/
* Click on the microphone and say "please rotate 180 degrees" and watch it turn back. You should also see the command displayed in `humancli`.

## 9.6 Follow person

Needs a person in the camera view and `ALIBABA_API_KEY` with access to the
`qwen-vl-max` model used by `follow_person` for its initial detection.

```bash
uv run dimos run unitree-go2-agentic
sleep 10; uv run dimos agent-send "follow the person wearing ..."
```

Replace `...` with a description of the person present. Expected result: the robot follows that person.

## 9.7 Sport commands

```bash
uv run dimos run unitree-go2-agentic
uv run dimos agent-send "show the heart symbol"
```

Expected result: the robot performs the requested gesture.

## 9.8 demo-camera

```bash
uv run dimos run demo-camera
```

* Webcam feed appears in rerun.

## 9.9 Go2 discovery and connection

Needs a Bluetooth adapter and the Unitree account the robot is bound to. Go2 firmware 1.1.15 and newer needs the AES-128 key.

```bash
uv run dimos go2tool discover
uv run dimos go2tool connect-wifi --ssid <wifi> --password <password>
uv run dimos go2tool discover
uv run unitree-fetch-aes-key --email <email> --device-type Go2 --sn <serial>
GO2CONNECTION__AES_128_KEY=<key> uv run dimos --robot-ip <ip> run unitree-go2
```

Expected result: the first `discover` lists the robot as a BLE row with its serial. `connect-wifi` prints "Provisioned." with the serial. The second `discover` also lists a LAN row with the robot's IP. The key tool prints a box with the `Key` line. `unitree-go2` connects with that key.

## 9.10 Go2 rage mode

Clear the area first. Rage mode drives at up to about 2.5 m/s.

```bash
uv run dimos run unitree-go2-webrtc-rage-keyboard-teleop
```

Expected result: WASD drives the robot noticeably faster than the normal keyboard teleop.

## 9.11 Go2 drive and record

```bash
uv run dimos --record run unitree-go2-joystick-record
uv run dimos --replay-db recordings/<run-id>/memory.db run replay
```

* Drive with W/S, A/D and Q/E in the keyboard window, then press Ctrl+C. The "Recording to ..." log line names the recording.

Expected result: Rerun shows the camera, the lidar and a `cmd_vel` plot that moves while you drive. The replay plays the drive back without the robot.

## 9.12 RealSense camera

Needs an Intel RealSense D4xx on USB 3. The first start builds the capture binary with nix.

```bash
uv run dimos run real-sense-camera-vis --enable-pointcloud true
```

Expected result: Rerun shows live Color, Depth, ir_left and ir_right panes, and the 3D view shows the colored point cloud in front of the camera.
