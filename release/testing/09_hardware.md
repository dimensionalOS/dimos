# 9. Hardware checks

The Go2 checks require a connected robot and room to perform the described movements. The camera checks need a webcam and a RealSense. Follow the [Go2 setup guide](../../docs/platforms/quadruped/go2/index.md) and the shared setup in the release testing plan.

Run each foreground blueprint separately. Use another terminal for commands that interact with it, and stop it before moving to the next check.

## 9.1 Test launching multiple blueprints

```bash
uv run dimos run unitree-go2 keyboard-teleop
```

Expected result: the Keyboard Teleop window drives the robot, and after Ctrl+C on `dimos` the robot lies down.

## 9.2 Commanding the robot from rerun

```bash
uv run dimos run unitree-go2
```

* Click on a point on the 3D view in the Rerun window. The robot should travel there.
* Control the robot with WASD. (This is a bit finicky.)

Expected result: the robot walks to the clicked point, and WASD drives it.

## 9.3 demo-camera

```bash
uv run dimos run demo-camera
```

Expected result: the webcam feed appears in Rerun.

## 9.4 Go2 connection with an AES key

Needs a Bluetooth adapter and the Unitree account the robot is bound to. Go2 firmware 1.1.15 and newer needs the AES-128 key. Wifi provisioning and discovery use [go2tool](https://github.com/jeff-hykin/go2tool).

```bash
uvx go2tool discover
uvx go2tool connect-wifi --ssid <wifi> --password <password>
uvx go2tool discover
uv run unitree-fetch-aes-key --email <email> --device-type Go2 --sn <serial>
GO2WEBRTC__AES_128_KEY=<key> uv run dimos --robot-ip <ip> run unitree-go2
```

Expected result: the first `discover` lists the robot as a BLE row with its serial. `connect-wifi` prints "Provisioned." with the serial. The second `discover` also lists a LAN row with the robot's IP. The key tool prints a box with the `Key` line. `unitree-go2` connects with that key.

## 9.5 Go2 rage mode

Clear the area first. Rage mode drives at up to about 2.5 m/s.

```bash
uv run dimos run unitree-go2 keyboard-teleop --go2webrtc.mode rage
```

Expected result: WASD drives the robot noticeably faster than the normal keyboard teleop.

## 9.6 Go2 drive and record

```bash
uv run dimos --record run unitree-go2 keyboard-teleop
uv run dimos --replay-db recordings/<run-id>/memory.db run replay
```

* Drive with W/S, A/D and Q/E in the keyboard window, then press Ctrl+C. The "Recording to ..." log line names the recording.

Expected result: Rerun shows the camera and the lidar while you drive. The replay plays the drive back without the robot.

## 9.7 RealSense camera

Needs an Intel RealSense D4xx on USB 3. The first start builds the capture binary with nix.

```bash
uv run dimos run real-sense-camera-vis --enable-pointcloud true
```

Expected result: Rerun shows live Color, Depth, ir_left and ir_right panes, and the 3D view shows the colored point cloud in front of the camera.
