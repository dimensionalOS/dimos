# Lua Robot Control Example

Subscribes to robot odometry and publishes twist commands using LCM.

## Prerequisites

- Lua 5.4
- LuaSocket (`sudo luarocks install luasocket`)
- System dependencies: `glib`, `cmake`

## Setup

```bash
./setup.sh
```

This will:
1. Clone and build official [LCM](https://github.com/lcm-proj/lcm) Lua bindings
2. Clone [dimos-lcm](https://github.com/dimensionalOS/dimos-lcm) message definitions

## Run

```bash
lua main.lua
```

## Output

```
Robot control started
Subscribing to /odom, publishing to /cmd_vel
Press Ctrl+C to stop.

[pose] x=15.29 y=9.62 z=0.00 | qw=0.57
[twist] linear=0.50 angular=0.00
[pose] x=15.28 y=9.63 z=0.00 | qw=0.57
...
```
