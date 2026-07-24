# Hosted Teleop Recovery Design

## Goal

Restore the proven Go2 control path:

`DimOS Studio -> teleop-hosted-go2-transport -> teleop.dimensionalos.com`

Studio remains a launcher and development workbench. It does not duplicate the
official video, map, joystick, emergency-stop, or click-to-navigate cockpit.

## User flow

1. Power on the Go2 and put the Mac on the same network.
2. Click **自动发现机器狗**. Studio records the discovered IP.
3. Paste the Dimensional API key once. Studio stores it in macOS Keychain, never
   in the project settings or logs.
4. Type `START TELEOP` and click **启动官方遥控连接**.
5. Open <https://teleop.dimensionalos.com/> and select the online robot.

## Safety and reliability

- Discovery and signal-port checks never send movement commands.
- Hosted teleop starts with `go2connection.auto_stand=false`.
- Starting a movement-capable hosted session requires the explicit phrase
  `START TELEOP`.
- The custom Agent runtime keeps its independent movement lock.
- Studio rejects duplicate launches while a launcher is still starting.
- Stop handles both registered DimOS runs and pre-registration launcher
  processes.
- The broker API key is passed only through the child process environment.

## Validation

- Unit/API tests cover discovery parsing, key status, hosted command/env
  construction, duplicate-launch reporting, and existing safety behavior.
- Live validation stops at connectivity, video/status, and hosted robot
  presence. It does not send a movement command.
