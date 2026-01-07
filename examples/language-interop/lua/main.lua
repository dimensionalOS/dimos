#!/usr/bin/env lua

-- Lua robot control example
-- Subscribes to robot pose and publishes twist commands

-- Add local msgs folder to path
local script_dir = arg[0]:match("(.*/)")  or "./"
package.path = script_dir .. "msgs/?.lua;" .. package.path
package.path = script_dir .. "msgs/?/init.lua;" .. package.path

local lcm = require("lcm")
local PoseStamped = require("geometry_msgs.PoseStamped")
local Twist = require("geometry_msgs.Twist")
local Vector3 = require("geometry_msgs.Vector3")

local lc = lcm.lcm.new()

print("Robot control started")
print("Subscribing to /odom, publishing to /cmd_vel")
print("Press Ctrl+C to stop.\n")

-- Subscribe to pose
lc:subscribe("/odom#geometry_msgs.PoseStamped", function(channel, msg)
  msg = PoseStamped.decode(msg)
  local pos = msg.pose.position
  local ori = msg.pose.orientation
  print(string.format("[pose] x=%.2f y=%.2f z=%.2f | qw=%.2f",
    pos.x, pos.y, pos.z, ori.w))
end)

-- Publisher loop
local t = 0
local socket = require("socket")
local last_pub = socket.gettime()

while true do
  -- Handle incoming messages
  lc:handle()

  -- Publish at ~10 Hz
  local now = socket.gettime()
  if now - last_pub >= 0.1 then
    local twist = Twist:new()
    twist.linear = Vector3:new()
    twist.linear.x = 0.5
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular = Vector3:new()
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = math.sin(t) * 0.3

    lc:publish("/cmd_vel#geometry_msgs.Twist", twist:encode())
    print(string.format("[twist] linear=%.2f angular=%.2f", twist.linear.x, twist.angular.z))

    t = t + 0.1
    last_pub = now
  end
end
