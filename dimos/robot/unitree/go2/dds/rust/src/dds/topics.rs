// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Topic names and sport api ids for the Go2 (unitree_sdk2 vocabulary).

pub const CLOUD_DESKEWED: &str = "rt/utlidar/cloud_deskewed";
pub const ROBOT_ODOM: &str = "rt/utlidar/robot_odom";
pub const SPORT_REQUEST: &str = "rt/api/sport/request";
/// Head L1 on/off, a `String_` "ON"/"OFF".
pub const LIDAR_SWITCH: &str = "rt/utlidar/switch";

/// Sport-mode api ids (`unitree_sdk2py` go2/sport/sport_api.py).
pub mod sport {
    pub const DAMP: i64 = 1001;
    pub const BALANCE_STAND: i64 = 1002;
    pub const STOP_MOVE: i64 = 1003;
    pub const STAND_UP: i64 = 1004;
    pub const STAND_DOWN: i64 = 1005;
    pub const RECOVERY_STAND: i64 = 1006;
    pub const MOVE: i64 = 1008; // param {"x":vx,"y":vy,"z":vyaw}
    pub const SIT: i64 = 1009;
    pub const RISE_SIT: i64 = 1010;
    pub const HELLO: i64 = 1016;
    pub const STRETCH: i64 = 1017;
    pub const WALLOW: i64 = 1021;
    pub const DANCE1: i64 = 1022;
    pub const DANCE2: i64 = 1023;
    pub const SCRAPE: i64 = 1029;
    pub const FRONT_FLIP: i64 = 1030;
    pub const FRONT_JUMP: i64 = 1031;
    pub const FRONT_POUNCE: i64 = 1032;
    pub const WIGGLE_HIPS: i64 = 1033;
    pub const HEART: i64 = 1036;
}

/// The `command` port vocabulary, shared with go2web and `Go2Base` in python.
pub fn sport_id(name: &str) -> Option<i64> {
    Some(match name {
        "damp" => sport::DAMP,
        "balance" => sport::BALANCE_STAND,
        "stand-up" | "standup" => sport::STAND_UP,
        "stand-down" | "standdown" => sport::STAND_DOWN,
        "recovery" => sport::RECOVERY_STAND,
        "sit" => sport::SIT,
        "rise" => sport::RISE_SIT,
        "hello" => sport::HELLO,
        "stretch" => sport::STRETCH,
        "wallow" => sport::WALLOW,
        "dance1" => sport::DANCE1,
        "dance2" => sport::DANCE2,
        "scrape" => sport::SCRAPE,
        "flip" | "frontflip" => sport::FRONT_FLIP,
        "jump" | "frontjump" => sport::FRONT_JUMP,
        "pounce" | "frontpounce" => sport::FRONT_POUNCE,
        "wiggle" | "wigglehips" => sport::WIGGLE_HIPS,
        "heart" => sport::HEART,
        _ => return None,
    })
}
