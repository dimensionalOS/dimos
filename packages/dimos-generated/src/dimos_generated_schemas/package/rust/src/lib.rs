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

use std::io;

// Generated from ROS2 .msg definitions. Do not edit.
pub mod codec;
pub mod builtin_interfaces {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Duration {
            pub sec: i32,
            pub nanosec: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Duration {
            fn default() -> Self {
                Self {
                    sec: ::std::default::Default::default(),
                    nanosec: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Duration {
            const NAME: &'static str = "builtin_interfaces/msg/Duration";
            const SCHEMA: &'static str = "# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Time {
            pub sec: i32,
            pub nanosec: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Time {
            fn default() -> Self {
                Self {
                    sec: ::std::default::Default::default(),
                    nanosec: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Time {
            const NAME: &'static str = "builtin_interfaces/msg/Time";
            const SCHEMA: &'static str = "# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
    }
}
pub mod dimos_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox2DArray {
            pub header: crate::std_msgs::msg::Header,
            pub boxes: ::std::vec::Vec<crate::vision_msgs::msg::BoundingBox2D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox2DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    boxes: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox2DArray {
            const NAME: &'static str = "dimos_msgs/msg/BoundingBox2DArray";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\nstd_msgs/Header header\nvision_msgs/BoundingBox2D[] boxes\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox2D\n# A 2D bounding box that can be rotated about its center.\n# All dimensions are in pixels, but represented using floating-point\n#   values to allow sub-pixel precision. If an exact pixel crop is required\n#   for a rotated bounding box, it can be calculated using Bresenham's line\n#   algorithm.\n\n# The 2D position (in pixels) and orientation of the bounding box center.\nvision_msgs/Pose2D center\n\n# The total size (in pixels) of the bounding box surrounding the object relative\n#   to the pose of its center.\nfloat64 size_x\nfloat64 size_y\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n================================================================================\nMSG: vision_msgs/Pose2D\n# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.boxes {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox3DArray {
            pub header: crate::std_msgs::msg::Header,
            pub boxes: ::std::vec::Vec<crate::vision_msgs::msg::BoundingBox3D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox3DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    boxes: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox3DArray {
            const NAME: &'static str = "dimos_msgs/msg/BoundingBox3DArray";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\nstd_msgs/Header header\nvision_msgs/BoundingBox3D[] boxes\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox3D\n# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n# Dimensions of this box are in meters, and as such, it may be migrated to\n#   another package, such as geometry_msgs, in the future.\n\n# The 3D position and orientation of the bounding box center\ngeometry_msgs/Pose center\n\n# The total size of the bounding box, in meters, surrounding the object's center\n#   pose.\ngeometry_msgs/Vector3 size\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.boxes {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct EntityMarker {
            pub entity_id: ::std::string::String,
            pub label: ::std::string::String,
            pub entity_type: ::std::string::String,
            pub position: crate::geometry_msgs::msg::Point,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for EntityMarker {
            fn default() -> Self {
                Self {
                    entity_id: ::std::default::Default::default(),
                    label: ::std::default::Default::default(),
                    entity_type: ::std::default::Default::default(),
                    position: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for EntityMarker {
            const NAME: &'static str = "dimos_msgs/msg/EntityMarker";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# A labeled world-space entity; rendering belongs to viewer adapters.\nstring entity_id\nstring label\nstring entity_type\ngeometry_msgs/Point position\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.position)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct EntityMarkers {
            pub header: crate::std_msgs::msg::Header,
            pub markers: ::std::vec::Vec<crate::dimos_msgs::msg::EntityMarker>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for EntityMarkers {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    markers: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for EntityMarkers {
            const NAME: &'static str = "dimos_msgs/msg/EntityMarkers";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\nstd_msgs/Header header\ndimos_msgs/EntityMarker[] markers\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: dimos_msgs/EntityMarker\n# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# A labeled world-space entity; rendering belongs to viewer adapters.\nstring entity_id\nstring label\nstring entity_type\ngeometry_msgs/Point position\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.markers {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct EpisodeStatus {
            pub ts: f64,
            pub state: ::std::string::String,
            pub episodes_saved: i64,
            pub episodes_discarded: i64,
            pub last_event: ::std::string::String,
            pub task_label: ::std::vec::Vec<::std::string::String>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for EpisodeStatus {
            fn default() -> Self {
                Self {
                    ts: ::std::default::Default::default(),
                    state: ::std::default::Default::default(),
                    episodes_saved: ::std::default::Default::default(),
                    episodes_discarded: ::std::default::Default::default(),
                    last_event: "init".to_owned(),
                    task_label: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for EpisodeStatus {
            const NAME: &'static str = "dimos_msgs/msg/EpisodeStatus";
            const SCHEMA: &'static str = "# EpisodeStatus internal model from robot-learning PR4343, ef5e5f2710c482fb1d43d43ec50fd73b0fbe1dde.\n# Seconds; application validation requires a finite value.\nfloat64 ts\n# Application values: idle, recording. Required by the source model.\nstring state\n# Source Python ints use signed 64-bit wire storage; negative values are retained.\nint64 episodes_saved\nint64 episodes_discarded\n# Application values: start, save, discard, init.\nstring last_event \"init\"\n# Nullable string: [] means None; [\"\"] preserves an explicitly empty label.\nstring[<=1] task_label\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                if self.task_label.len() > 1 {
                    return Err("task_label exceeds sequence bound".into());
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct GraspCandidate {
            pub pose: crate::geometry_msgs::msg::Pose,
            pub score: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for GraspCandidate {
            fn default() -> Self {
                Self {
                    pose: ::std::default::Default::default(),
                    score: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for GraspCandidate {
            const NAME: &'static str = "dimos_msgs/msg/GraspCandidate";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Target TCP pose and generator-local ranking score.\ngeometry_msgs/Pose pose\nfloat64 score\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct GraspCandidateArray {
            pub header: crate::std_msgs::msg::Header,
            pub candidates: ::std::vec::Vec<crate::dimos_msgs::msg::GraspCandidate>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for GraspCandidateArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    candidates: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for GraspCandidateArray {
            const NAME: &'static str = "dimos_msgs/msg/GraspCandidateArray";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Candidates share the input cloud's frame and timestamp.\nstd_msgs/Header header\nGraspCandidate[] candidates\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: dimos_msgs/GraspCandidate\n# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Target TCP pose and generator-local ranking score.\ngeometry_msgs/Pose pose\nfloat64 score\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.candidates {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ImuInfo {
            pub header: crate::std_msgs::msg::Header,
            pub gyro_noise_density: f64,
            pub gyro_random_walk: f64,
            pub accel_noise_density: f64,
            pub accel_random_walk: f64,
            pub frequency: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ImuInfo {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    gyro_noise_density: ::std::default::Default::default(),
                    gyro_random_walk: ::std::default::Default::default(),
                    accel_noise_density: ::std::default::Default::default(),
                    accel_random_walk: ::std::default::Default::default(),
                    frequency: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ImuInfo {
            const NAME: &'static str = "dimos_msgs/msg/ImuInfo";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Continuous-time IMU noise model. Extrinsics are supplied separately through TF.\nstd_msgs/Header header\n# rad/s/sqrt(Hz)\nfloat64 gyro_noise_density\n# rad/s^2/sqrt(Hz)\nfloat64 gyro_random_walk\n# m/s^2/sqrt(Hz)\nfloat64 accel_noise_density\n# m/s^3/sqrt(Hz)\nfloat64 accel_random_walk\n# Delivered sample rate in Hz.\nfloat64 frequency\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JointCommand {
            pub header: crate::std_msgs::msg::Header,
            pub positions: ::std::vec::Vec<f64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JointCommand {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    positions: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JointCommand {
            const NAME: &'static str = "dimos_msgs/msg/JointCommand";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Joint position targets in radians, ordered by the controller's joint list.\nstd_msgs/Header header\nfloat64[] positions\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct LineSegment3D {
            pub start: crate::geometry_msgs::msg::Point,
            pub end: crate::geometry_msgs::msg::Point,
            pub weight: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for LineSegment3D {
            fn default() -> Self {
                Self {
                    start: ::std::default::Default::default(),
                    end: ::std::default::Default::default(),
                    weight: 1.0,
                }
            }
        }
        impl crate::codec::Message for LineSegment3D {
            const NAME: &'static str = "dimos_msgs/msg/LineSegment3D";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\ngeometry_msgs/Point start\ngeometry_msgs/Point end\nfloat64 weight 1.0\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.start)?;
                crate::codec::Message::validate(&self.end)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct LineSegments3D {
            pub header: crate::std_msgs::msg::Header,
            pub segments: ::std::vec::Vec<crate::dimos_msgs::msg::LineSegment3D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for LineSegments3D {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    segments: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for LineSegments3D {
            const NAME: &'static str = "dimos_msgs/msg/LineSegments3D";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Weighted segments in a common coordinate frame.\nstd_msgs/Header header\nLineSegment3D[] segments\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: dimos_msgs/LineSegment3D\n# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\ngeometry_msgs/Point start\ngeometry_msgs/Point end\nfloat64 weight 1.0\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.segments {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MotorCommandArray {
            pub header: crate::std_msgs::msg::Header,
            pub q: ::std::vec::Vec<f64>,
            pub dq: ::std::vec::Vec<f64>,
            pub kp: ::std::vec::Vec<f64>,
            pub kd: ::std::vec::Vec<f64>,
            pub tau: ::std::vec::Vec<f64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MotorCommandArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    q: ::std::default::Default::default(),
                    dq: ::std::default::Default::default(),
                    kp: ::std::default::Default::default(),
                    kd: ::std::default::Default::default(),
                    tau: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MotorCommandArray {
            const NAME: &'static str = "dimos_msgs/msg/MotorCommandArray";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Per-joint hybrid commands. All arrays must have the same length.\n# q: rad; dq: rad/s; tau: N*m. Gains follow the actuator controller's units.\nstd_msgs/Header header\nfloat64[] q\nfloat64[] dq\nfloat64[] kp\nfloat64[] kd\nfloat64[] tau\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct RobotState {
            pub header: crate::std_msgs::msg::Header,
            pub state: i32,
            pub mode: i32,
            pub error_code: i32,
            pub warn_code: i32,
            pub cmdnum: i32,
            pub mt_brake: i32,
            pub mt_able: i32,
            pub tcp_pose: ::std::vec::Vec<f64>,
            pub tcp_offset: ::std::vec::Vec<f64>,
            pub joints: ::std::vec::Vec<f64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for RobotState {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    state: ::std::default::Default::default(),
                    mode: ::std::default::Default::default(),
                    error_code: ::std::default::Default::default(),
                    warn_code: ::std::default::Default::default(),
                    cmdnum: ::std::default::Default::default(),
                    mt_brake: ::std::default::Default::default(),
                    mt_able: ::std::default::Default::default(),
                    tcp_pose: ::std::default::Default::default(),
                    tcp_offset: ::std::default::Default::default(),
                    joints: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for RobotState {
            const NAME: &'static str = "dimos_msgs/msg/RobotState";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\n# Controller feedback. TCP values retain the driver's x/y/z/roll/pitch/yaw convention.\nstd_msgs/Header header\nint32 state\nint32 mode\nint32 error_code\nint32 warn_code\nint32 cmdnum\nint32 mt_brake\nint32 mt_able\nfloat64[] tcp_pose\nfloat64[] tcp_offset\nfloat64[] joints\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TrajectoryStatus {
            pub header: crate::std_msgs::msg::Header,
            pub state: u8,
            pub progress: f64,
            pub time_elapsed: crate::builtin_interfaces::msg::Duration,
            pub time_remaining: crate::builtin_interfaces::msg::Duration,
            pub error: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TrajectoryStatus {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    state: ::std::default::Default::default(),
                    progress: ::std::default::Default::default(),
                    time_elapsed: ::std::default::Default::default(),
                    time_remaining: ::std::default::Default::default(),
                    error: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TrajectoryStatus {
            const NAME: &'static str = "dimos_msgs/msg/TrajectoryStatus";
            const SCHEMA: &'static str = "# Copyright 2026 Dimensional Inc.\n# SPDX-License-Identifier: Apache-2.0\n\nstd_msgs/Header header\nuint8 IDLE=0\nuint8 EXECUTING=1\nuint8 COMPLETED=2\nuint8 ABORTED=3\nuint8 FAULT=4\nuint8 state\n# Fraction completed, in [0, 1].\nfloat64 progress\nbuiltin_interfaces/Duration time_elapsed\nbuiltin_interfaces/Duration time_remaining\nstring error\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.time_elapsed)?;
                crate::codec::Message::validate(&self.time_remaining)?;
                Ok(())
            }
        }
        impl TrajectoryStatus {
            pub const IDLE: u8 = 0;
            pub const EXECUTING: u8 = 1;
            pub const COMPLETED: u8 = 2;
            pub const ABORTED: u8 = 3;
            pub const FAULT: u8 = 4;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct VideoStats {
            pub header: crate::std_msgs::msg::Header,
            pub fps: f64,
            pub kbps: f64,
            pub width: u32,
            pub height: u32,
            pub loss_pct: f64,
            pub jitter_buffer_ms: f64,
            pub decode_ms: f64,
            pub frames_dropped: u64,
            pub freezes: u64,
            pub e2e_latency_ms: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for VideoStats {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    fps: ::std::default::Default::default(),
                    kbps: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    loss_pct: ::std::default::Default::default(),
                    jitter_buffer_ms: ::std::default::Default::default(),
                    decode_ms: ::std::default::Default::default(),
                    frames_dropped: ::std::default::Default::default(),
                    freezes: ::std::default::Default::default(),
                    e2e_latency_ms: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for VideoStats {
            const NAME: &'static str = "dimos_msgs/msg/VideoStats";
            const SCHEMA: &'static str = "std_msgs/Header header\nfloat64 fps\nfloat64 kbps\nuint32 width\nuint32 height\nfloat64 loss_pct\nfloat64 jitter_buffer_ms\nfloat64 decode_ms\nuint64 frames_dropped\nuint64 freezes\nfloat64 e2e_latency_ms\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
    }
}
pub mod foxglove_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct CompressedVideo {
            pub timestamp: crate::builtin_interfaces::msg::Time,
            pub frame_id: ::std::string::String,
            pub data: ::std::vec::Vec<u8>,
            pub format: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for CompressedVideo {
            fn default() -> Self {
                Self {
                    timestamp: ::std::default::Default::default(),
                    frame_id: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                    format: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for CompressedVideo {
            const NAME: &'static str = "foxglove_msgs/msg/CompressedVideo";
            const SCHEMA: &'static str = "# foxglove_msgs/msg/CompressedVideo\n# A single frame of a compressed video bitstream\n\n# Generated by https://github.com/foxglove/foxglove-sdk\n\n# Timestamp of video frame\nbuiltin_interfaces/Time timestamp\n\n# Frame of reference for the video.\n# \n# The origin of the frame is the optical center of the camera. +x points to the right in the video, +y points down, and +z points into the plane of the video.\nstring frame_id\n\n# Compressed video frame data.\n# \n# For packet-based video codecs this data must begin and end on packet boundaries (no partial packets), and must contain enough video packets to decode exactly one image (either a keyframe or delta frame). Note: Foxglove does not support video streams that include B frames because they require lookahead.\n# \n# Specifically, the requirements for different `format` values are:\n# \n# - `h264`\n#   - Use Annex B formatted data\n#   - Each CompressedVideo message should contain enough NAL units to decode exactly one video frame\n#   - Each message containing a key frame (IDR) must also include a SPS NAL unit\n# \n# - `h265` (HEVC)\n#   - Use Annex B formatted data\n#   - Each CompressedVideo message should contain enough NAL units to decode exactly one video frame\n#   - Each message containing a key frame (IRAP) must also include relevant VPS/SPS/PPS NAL units\n# \n# - `vp9`\n#   - Each CompressedVideo message should contain exactly one video frame\n# \n# - `av1`\n#   - Use the \"Low overhead bitstream format\" (section 5.2)\n#   - Each CompressedVideo message should contain enough OBUs to decode exactly one video frame\n#   - Each message containing a key frame must also include a Sequence Header OBU\nuint8[] data\n\n# Video format.\n# \n# Supported values: `h264`, `h265`, `vp9`, `av1`.\n# \n# Note: compressed video support is subject to hardware limitations and patent licensing, so not all encodings may be supported on all platforms. See more about [H.265 support](https://caniuse.com/hevc), [VP9 support](https://caniuse.com/webm), and [AV1 support](https://caniuse.com/av1).\nstring format\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.timestamp)?;
                Ok(())
            }
        }
    }
}
pub mod geometry_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Point {
            pub x: f64,
            pub y: f64,
            pub z: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Point {
            fn default() -> Self {
                Self {
                    x: ::std::default::Default::default(),
                    y: ::std::default::Default::default(),
                    z: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Point {
            const NAME: &'static str = "geometry_msgs/msg/Point";
            const SCHEMA: &'static str = "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Quaternion {
            pub x: f64,
            pub y: f64,
            pub z: f64,
            pub w: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Quaternion {
            fn default() -> Self {
                Self {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                }
            }
        }
        impl crate::codec::Message for Quaternion {
            const NAME: &'static str = "geometry_msgs/msg/Quaternion";
            const SCHEMA: &'static str = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Pose {
            pub position: crate::geometry_msgs::msg::Point,
            pub orientation: crate::geometry_msgs::msg::Quaternion,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Pose {
            fn default() -> Self {
                Self {
                    position: ::std::default::Default::default(),
                    orientation: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Pose {
            const NAME: &'static str = "geometry_msgs/msg/Pose";
            const SCHEMA: &'static str = "# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.position)?;
                crate::codec::Message::validate(&self.orientation)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Vector3 {
            pub x: f64,
            pub y: f64,
            pub z: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Vector3 {
            fn default() -> Self {
                Self {
                    x: ::std::default::Default::default(),
                    y: ::std::default::Default::default(),
                    z: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Vector3 {
            const NAME: &'static str = "geometry_msgs/msg/Vector3";
            const SCHEMA: &'static str = "# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Accel {
            pub linear: crate::geometry_msgs::msg::Vector3,
            pub angular: crate::geometry_msgs::msg::Vector3,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Accel {
            fn default() -> Self {
                Self {
                    linear: ::std::default::Default::default(),
                    angular: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Accel {
            const NAME: &'static str = "geometry_msgs/msg/Accel";
            const SCHEMA: &'static str = "# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.linear)?;
                crate::codec::Message::validate(&self.angular)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct AccelStamped {
            pub header: crate::std_msgs::msg::Header,
            pub accel: crate::geometry_msgs::msg::Accel,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for AccelStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    accel: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for AccelStamped {
            const NAME: &'static str = "geometry_msgs/msg/AccelStamped";
            const SCHEMA: &'static str = "# An accel with reference coordinate frame and timestamp\nstd_msgs/Header header\nAccel accel\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.accel)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct AccelWithCovariance {
            pub accel: crate::geometry_msgs::msg::Accel,
            #[serde(with = "serde_big_array::BigArray")]
            pub covariance: [f64; 36],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for AccelWithCovariance {
            fn default() -> Self {
                Self {
                    accel: ::std::default::Default::default(),
                    covariance: ::std::array::from_fn(|_| ::std::default::Default::default()),
                }
            }
        }
        impl crate::codec::Message for AccelWithCovariance {
            const NAME: &'static str = "geometry_msgs/msg/AccelWithCovariance";
            const SCHEMA: &'static str = "# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.accel)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct AccelWithCovarianceStamped {
            pub header: crate::std_msgs::msg::Header,
            pub accel: crate::geometry_msgs::msg::AccelWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for AccelWithCovarianceStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    accel: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for AccelWithCovarianceStamped {
            const NAME: &'static str = "geometry_msgs/msg/AccelWithCovarianceStamped";
            const SCHEMA: &'static str = "# This represents an estimated accel with reference coordinate frame and timestamp.\nstd_msgs/Header header\nAccelWithCovariance accel\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/AccelWithCovariance\n# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.accel)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Inertia {
            pub m: f64,
            pub com: crate::geometry_msgs::msg::Vector3,
            pub ixx: f64,
            pub ixy: f64,
            pub ixz: f64,
            pub iyy: f64,
            pub iyz: f64,
            pub izz: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Inertia {
            fn default() -> Self {
                Self {
                    m: ::std::default::Default::default(),
                    com: ::std::default::Default::default(),
                    ixx: ::std::default::Default::default(),
                    ixy: ::std::default::Default::default(),
                    ixz: ::std::default::Default::default(),
                    iyy: ::std::default::Default::default(),
                    iyz: ::std::default::Default::default(),
                    izz: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Inertia {
            const NAME: &'static str = "geometry_msgs/msg/Inertia";
            const SCHEMA: &'static str = "# Mass [kg]\nfloat64 m\n\n# Center of mass [m]\ngeometry_msgs/Vector3 com\n\n# Inertia Tensor [kg-m^2] about the center of mass\n#     | ixx ixy ixz |\n# I = | ixy iyy iyz |\n#     | ixz iyz izz |\nfloat64 ixx\nfloat64 ixy\nfloat64 ixz\nfloat64 iyy\nfloat64 iyz\nfloat64 izz\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.com)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InertiaStamped {
            pub header: crate::std_msgs::msg::Header,
            pub inertia: crate::geometry_msgs::msg::Inertia,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InertiaStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    inertia: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InertiaStamped {
            const NAME: &'static str = "geometry_msgs/msg/InertiaStamped";
            const SCHEMA: &'static str = "# An Inertia with a time stamp and reference frame.\n\nstd_msgs/Header header\nInertia inertia\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Inertia\n# Mass [kg]\nfloat64 m\n\n# Center of mass [m]\ngeometry_msgs/Vector3 com\n\n# Inertia Tensor [kg-m^2] about the center of mass\n#     | ixx ixy ixz |\n# I = | ixy iyy iyz |\n#     | ixz iyz izz |\nfloat64 ixx\nfloat64 ixy\nfloat64 ixz\nfloat64 iyy\nfloat64 iyz\nfloat64 izz\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.inertia)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Point32 {
            pub x: f32,
            pub y: f32,
            pub z: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Point32 {
            fn default() -> Self {
                Self {
                    x: ::std::default::Default::default(),
                    y: ::std::default::Default::default(),
                    z: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Point32 {
            const NAME: &'static str = "geometry_msgs/msg/Point32";
            const SCHEMA: &'static str = "# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PointStamped {
            pub header: crate::std_msgs::msg::Header,
            pub point: crate::geometry_msgs::msg::Point,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PointStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    point: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PointStamped {
            const NAME: &'static str = "geometry_msgs/msg/PointStamped";
            const SCHEMA: &'static str = "# This represents a Point with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPoint point\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.point)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Polygon {
            pub points: ::std::vec::Vec<crate::geometry_msgs::msg::Point32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Polygon {
            fn default() -> Self {
                Self {
                    points: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Polygon {
            const NAME: &'static str = "geometry_msgs/msg/Polygon";
            const SCHEMA: &'static str = "# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PolygonInstance {
            pub polygon: crate::geometry_msgs::msg::Polygon,
            pub id: i64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PolygonInstance {
            fn default() -> Self {
                Self {
                    polygon: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PolygonInstance {
            const NAME: &'static str = "geometry_msgs/msg/PolygonInstance";
            const SCHEMA: &'static str = "# A specification of a polygon where the first and last points are assumed to be connected\n# It includes a unique identification field for disambiguating multiple instances\n\ngeometry_msgs/Polygon polygon\nint64 id\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n================================================================================\nMSG: geometry_msgs/Polygon\n# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.polygon)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PolygonInstanceStamped {
            pub header: crate::std_msgs::msg::Header,
            pub polygon: crate::geometry_msgs::msg::PolygonInstance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PolygonInstanceStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    polygon: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PolygonInstanceStamped {
            const NAME: &'static str = "geometry_msgs/msg/PolygonInstanceStamped";
            const SCHEMA: &'static str = "# This represents a Polygon with reference coordinate frame and timestamp\n# It includes a unique identification field for disambiguating multiple instances\n\nstd_msgs/Header header\ngeometry_msgs/PolygonInstance polygon\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n================================================================================\nMSG: geometry_msgs/Polygon\n# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points\n================================================================================\nMSG: geometry_msgs/PolygonInstance\n# A specification of a polygon where the first and last points are assumed to be connected\n# It includes a unique identification field for disambiguating multiple instances\n\ngeometry_msgs/Polygon polygon\nint64 id\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.polygon)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PolygonStamped {
            pub header: crate::std_msgs::msg::Header,
            pub polygon: crate::geometry_msgs::msg::Polygon,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PolygonStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    polygon: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PolygonStamped {
            const NAME: &'static str = "geometry_msgs/msg/PolygonStamped";
            const SCHEMA: &'static str = "# This represents a Polygon with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPolygon polygon\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n================================================================================\nMSG: geometry_msgs/Polygon\n# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.polygon)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Pose2D {
            pub x: f64,
            pub y: f64,
            pub theta: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Pose2D {
            fn default() -> Self {
                Self {
                    x: ::std::default::Default::default(),
                    y: ::std::default::Default::default(),
                    theta: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Pose2D {
            const NAME: &'static str = "geometry_msgs/msg/Pose2D";
            const SCHEMA: &'static str = "# Deprecated as of Foxy and will potentially be removed in any following release.\n# Please use the full 3D pose.\n\n# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\n# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.# This expresses a position and orientation on a 2D manifold.\n\nfloat64 x\nfloat64 y\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PoseArray {
            pub header: crate::std_msgs::msg::Header,
            pub poses: ::std::vec::Vec<crate::geometry_msgs::msg::Pose>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PoseArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    poses: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PoseArray {
            const NAME: &'static str = "geometry_msgs/msg/PoseArray";
            const SCHEMA: &'static str = "# An array of poses with a header for global reference.\n\nstd_msgs/Header header\n\nPose[] poses\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.poses {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PoseStamped {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::Pose,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PoseStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PoseStamped {
            const NAME: &'static str = "geometry_msgs/msg/PoseStamped";
            const SCHEMA: &'static str = "# A Pose with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPose pose\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PoseWithCovariance {
            pub pose: crate::geometry_msgs::msg::Pose,
            #[serde(with = "serde_big_array::BigArray")]
            pub covariance: [f64; 36],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PoseWithCovariance {
            fn default() -> Self {
                Self {
                    pose: ::std::default::Default::default(),
                    covariance: ::std::array::from_fn(|_| ::std::default::Default::default()),
                }
            }
        }
        impl crate::codec::Message for PoseWithCovariance {
            const NAME: &'static str = "geometry_msgs/msg/PoseWithCovariance";
            const SCHEMA: &'static str = "# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PoseWithCovarianceStamped {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::PoseWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PoseWithCovarianceStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PoseWithCovarianceStamped {
            const NAME: &'static str = "geometry_msgs/msg/PoseWithCovarianceStamped";
            const SCHEMA: &'static str = "# This expresses an estimated pose with a reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPoseWithCovariance pose\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct QuaternionStamped {
            pub header: crate::std_msgs::msg::Header,
            pub quaternion: crate::geometry_msgs::msg::Quaternion,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for QuaternionStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    quaternion: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for QuaternionStamped {
            const NAME: &'static str = "geometry_msgs/msg/QuaternionStamped";
            const SCHEMA: &'static str = "# This represents an orientation with reference coordinate frame and timestamp.\n\nstd_msgs/Header header\nQuaternion quaternion\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.quaternion)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Transform {
            pub translation: crate::geometry_msgs::msg::Vector3,
            pub rotation: crate::geometry_msgs::msg::Quaternion,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Transform {
            fn default() -> Self {
                Self {
                    translation: ::std::default::Default::default(),
                    rotation: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Transform {
            const NAME: &'static str = "geometry_msgs/msg/Transform";
            const SCHEMA: &'static str = "# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.translation)?;
                crate::codec::Message::validate(&self.rotation)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TransformStamped {
            pub header: crate::std_msgs::msg::Header,
            pub child_frame_id: ::std::string::String,
            pub transform: crate::geometry_msgs::msg::Transform,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TransformStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    child_frame_id: ::std::default::Default::default(),
                    transform: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TransformStamped {
            const NAME: &'static str = "geometry_msgs/msg/TransformStamped";
            const SCHEMA: &'static str = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id at the time of header.stamp\n#\n# This message is mostly used by the\n# <a href=\"https://docs.ros.org/en/rolling/p/tf2/\">tf2</a> package.\n# See its documentation for more information.\n#\n# The child_frame_id is necessary in addition to the frame_id\n# in the Header to communicate the full reference for the transform\n# in a self contained message.\n\n# The frame id in the header is used as the reference frame of this transform.\nstd_msgs/Header header\n\n# The frame id of the child frame to which this transform points.\nstring child_frame_id\n\n# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.\nTransform transform\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Transform\n# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.transform)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Twist {
            pub linear: crate::geometry_msgs::msg::Vector3,
            pub angular: crate::geometry_msgs::msg::Vector3,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Twist {
            fn default() -> Self {
                Self {
                    linear: ::std::default::Default::default(),
                    angular: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Twist {
            const NAME: &'static str = "geometry_msgs/msg/Twist";
            const SCHEMA: &'static str = "# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.linear)?;
                crate::codec::Message::validate(&self.angular)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TwistStamped {
            pub header: crate::std_msgs::msg::Header,
            pub twist: crate::geometry_msgs::msg::Twist,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TwistStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    twist: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TwistStamped {
            const NAME: &'static str = "geometry_msgs/msg/TwistStamped";
            const SCHEMA: &'static str = "# A twist with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nTwist twist\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.twist)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TwistWithCovariance {
            pub twist: crate::geometry_msgs::msg::Twist,
            #[serde(with = "serde_big_array::BigArray")]
            pub covariance: [f64; 36],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TwistWithCovariance {
            fn default() -> Self {
                Self {
                    twist: ::std::default::Default::default(),
                    covariance: ::std::array::from_fn(|_| ::std::default::Default::default()),
                }
            }
        }
        impl crate::codec::Message for TwistWithCovariance {
            const NAME: &'static str = "geometry_msgs/msg/TwistWithCovariance";
            const SCHEMA: &'static str = "# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.twist)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TwistWithCovarianceStamped {
            pub header: crate::std_msgs::msg::Header,
            pub twist: crate::geometry_msgs::msg::TwistWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TwistWithCovarianceStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    twist: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TwistWithCovarianceStamped {
            const NAME: &'static str = "geometry_msgs/msg/TwistWithCovarianceStamped";
            const SCHEMA: &'static str = "# This represents an estimated twist with reference coordinate frame and timestamp.\n\nstd_msgs/Header header\nTwistWithCovariance twist\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/TwistWithCovariance\n# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.twist)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Vector3Stamped {
            pub header: crate::std_msgs::msg::Header,
            pub vector: crate::geometry_msgs::msg::Vector3,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Vector3Stamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    vector: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Vector3Stamped {
            const NAME: &'static str = "geometry_msgs/msg/Vector3Stamped";
            const SCHEMA: &'static str = "# This represents a Vector3 with reference coordinate frame and timestamp\n\n# Note that this follows vector semantics with it always anchored at the origin,\n# so the rotational elements of a transform are the only parts applied when transforming.\n\nstd_msgs/Header header\nVector3 vector\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.vector)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct VelocityStamped {
            pub header: crate::std_msgs::msg::Header,
            pub body_frame_id: ::std::string::String,
            pub reference_frame_id: ::std::string::String,
            pub velocity: crate::geometry_msgs::msg::Twist,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for VelocityStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    body_frame_id: ::std::default::Default::default(),
                    reference_frame_id: ::std::default::Default::default(),
                    velocity: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for VelocityStamped {
            const NAME: &'static str = "geometry_msgs/msg/VelocityStamped";
            const SCHEMA: &'static str = "# This expresses the timestamped velocity vector of a frame 'body_frame_id' in the reference frame 'reference_frame_id' expressed from arbitrary observation frame 'header.frame_id'.\n# - If the 'body_frame_id' and 'header.frame_id' are identical, the velocity is observed and defined in the local coordinates system of the body\n#   which is the usual use-case in mobile robotics and is also known as a body twist.\n\nstd_msgs/Header header\nstring body_frame_id\nstring reference_frame_id\nTwist velocity\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.velocity)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct VelocityWithCovarianceStamped {
            pub header: crate::std_msgs::msg::Header,
            pub body_frame_id: ::std::string::String,
            pub reference_frame_id: ::std::string::String,
            pub velocity: crate::geometry_msgs::msg::TwistWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for VelocityWithCovarianceStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    body_frame_id: ::std::default::Default::default(),
                    reference_frame_id: ::std::default::Default::default(),
                    velocity: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for VelocityWithCovarianceStamped {
            const NAME: &'static str = "geometry_msgs/msg/VelocityWithCovarianceStamped";
            const SCHEMA: &'static str = "# A timestamped velocity of a body whose frame is 'body_frame_id', measured\n# relative to the reference frame 'reference_frame_id', with the velocity and\n# covariance both expressed in the basis of the observation frame\n# 'header.frame_id'.\n#\n# - If 'body_frame_id' and 'header.frame_id' are identical, the velocity and\n#   covariance are expressed in the body's own basis. This is functionally\n#   equivalent to the body-twist convention used by\n#   'geometry_msgs/TwistStamped'.\n#\n# This message is the covariance-bearing analogue of\n# 'geometry_msgs/VelocityStamped'.\n\nstd_msgs/Header header\nstring body_frame_id\nstring reference_frame_id\nTwistWithCovariance velocity\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/TwistWithCovariance\n# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.velocity)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Wrench {
            pub force: crate::geometry_msgs::msg::Vector3,
            pub torque: crate::geometry_msgs::msg::Vector3,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Wrench {
            fn default() -> Self {
                Self {
                    force: ::std::default::Default::default(),
                    torque: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Wrench {
            const NAME: &'static str = "geometry_msgs/msg/Wrench";
            const SCHEMA: &'static str = "# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.force)?;
                crate::codec::Message::validate(&self.torque)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct WrenchStamped {
            pub header: crate::std_msgs::msg::Header,
            pub wrench: crate::geometry_msgs::msg::Wrench,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for WrenchStamped {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    wrench: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for WrenchStamped {
            const NAME: &'static str = "geometry_msgs/msg/WrenchStamped";
            const SCHEMA: &'static str = "# A wrench with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nWrench wrench\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.wrench)?;
                Ok(())
            }
        }
    }
}
pub mod nav_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Goals {
            pub header: crate::std_msgs::msg::Header,
            pub goals: ::std::vec::Vec<crate::geometry_msgs::msg::PoseStamped>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Goals {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    goals: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Goals {
            const NAME: &'static str = "nav_msgs/msg/Goals";
            const SCHEMA: &'static str = "# An array of navigation goals\n\n\n# This header will store the time at which the poses were computed (not to be confused with the stamps of the poses themselves)\n# In the case that individual poses do not have their frame_id set or their timetamp set they will use the default value here.\nstd_msgs/Header header\n\n# An array of goals to for navigation to achieve.\n# The goals should be executed in the order of the array.\n# The header and stamp are intended to be used for computing the position of the goals.\n# They may vary to support cases of goals that are moving with respect to the robot.\ngeometry_msgs/PoseStamped[] goals\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseStamped\n# A Pose with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPose pose\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.goals {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct GridCells {
            pub header: crate::std_msgs::msg::Header,
            pub cell_width: f32,
            pub cell_height: f32,
            pub cells: ::std::vec::Vec<crate::geometry_msgs::msg::Point>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for GridCells {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    cell_width: ::std::default::Default::default(),
                    cell_height: ::std::default::Default::default(),
                    cells: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for GridCells {
            const NAME: &'static str = "nav_msgs/msg/GridCells";
            const SCHEMA: &'static str = "# An array of cells in a 2D grid\n\nstd_msgs/Header header\n\n# Width of each cell\nfloat32 cell_width\n\n# Height of each cell\nfloat32 cell_height\n\n# Each cell is represented by the Point at the center of the cell\ngeometry_msgs/Point[] cells\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.cells {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MapMetaData {
            pub map_load_time: crate::builtin_interfaces::msg::Time,
            pub resolution: f32,
            pub width: u32,
            pub height: u32,
            pub origin: crate::geometry_msgs::msg::Pose,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MapMetaData {
            fn default() -> Self {
                Self {
                    map_load_time: ::std::default::Default::default(),
                    resolution: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    origin: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MapMetaData {
            const NAME: &'static str = "nav_msgs/msg/MapMetaData";
            const SCHEMA: &'static str = "# This hold basic information about the characteristics of the OccupancyGrid\n\n# The time at which the map was loaded\nbuiltin_interfaces/Time map_load_time\n\n# The map resolution [m/cell]\nfloat32 resolution\n\n# Map width [cells]\nuint32 width\n\n# Map height [cells]\nuint32 height\n\n# The origin of the map [m, m, rad].  This is the real-world pose of the\n# bottom left corner of cell (0,0) in the map.\ngeometry_msgs/Pose origin\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.map_load_time)?;
                crate::codec::Message::validate(&self.origin)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct OccupancyGrid {
            pub header: crate::std_msgs::msg::Header,
            pub info: crate::nav_msgs::msg::MapMetaData,
            pub data: ::std::vec::Vec<i8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for OccupancyGrid {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    info: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for OccupancyGrid {
            const NAME: &'static str = "nav_msgs/msg/OccupancyGrid";
            const SCHEMA: &'static str = "# This represents a 2-D grid map\nstd_msgs/Header header\n\n# MetaData for the map\nMapMetaData info\n\n# The map data, in row-major order, starting with (0,0). \n# Cell (1, 0) will be listed second, representing the next cell in the x direction. \n# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).\n# The values inside are application dependent, but frequently, \n# 0 represents unoccupied, 1 represents definitely occupied, and\n# -1 represents unknown. \nint8[] data\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: nav_msgs/MapMetaData\n# This hold basic information about the characteristics of the OccupancyGrid\n\n# The time at which the map was loaded\nbuiltin_interfaces/Time map_load_time\n\n# The map resolution [m/cell]\nfloat32 resolution\n\n# Map width [cells]\nuint32 width\n\n# Map height [cells]\nuint32 height\n\n# The origin of the map [m, m, rad].  This is the real-world pose of the\n# bottom left corner of cell (0,0) in the map.\ngeometry_msgs/Pose origin\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.info)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Odometry {
            pub header: crate::std_msgs::msg::Header,
            pub child_frame_id: ::std::string::String,
            pub pose: crate::geometry_msgs::msg::PoseWithCovariance,
            pub twist: crate::geometry_msgs::msg::TwistWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Odometry {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    child_frame_id: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    twist: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Odometry {
            const NAME: &'static str = "nav_msgs/msg/Odometry";
            const SCHEMA: &'static str = "# This represents an estimate of a position and velocity in free space.\n# The pose in this message should be specified in the coordinate frame given by header.frame_id\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\n# Includes the frame id of the pose parent.\nstd_msgs/Header header\n\n# Frame id the pose points to. The twist is in this coordinate frame.\nstring child_frame_id\n\n# Estimated pose that is typically relative to a fixed world frame.\ngeometry_msgs/PoseWithCovariance pose\n\n# Estimated linear and angular velocity relative to child_frame_id.\ngeometry_msgs/TwistWithCovariance twist\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/TwistWithCovariance\n# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                crate::codec::Message::validate(&self.twist)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Path {
            pub header: crate::std_msgs::msg::Header,
            pub poses: ::std::vec::Vec<crate::geometry_msgs::msg::PoseStamped>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Path {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    poses: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Path {
            const NAME: &'static str = "nav_msgs/msg/Path";
            const SCHEMA: &'static str = "# An array of poses that represents a Path for a robot to follow.\n\n# Indicates the frame_id of the path.\nstd_msgs/Header header\n\n# Array of poses to follow.\ngeometry_msgs/PoseStamped[] poses\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseStamped\n# A Pose with reference coordinate frame and timestamp\n\nstd_msgs/Header header\nPose pose\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.poses {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TrajectoryPoint {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub velocity: crate::geometry_msgs::msg::Twist,
            pub acceleration: crate::geometry_msgs::msg::Accel,
            pub effort: crate::geometry_msgs::msg::Wrench,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TrajectoryPoint {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    velocity: ::std::default::Default::default(),
                    acceleration: ::std::default::Default::default(),
                    effort: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TrajectoryPoint {
            const NAME: &'static str = "nav_msgs/msg/TrajectoryPoint";
            const SCHEMA: &'static str = "# Trajectory point state\n\n# Absolute time and frame of reference of the point along a trajectory\nstd_msgs/Header header\n\n# Pose of the trajectory sample.\ngeometry_msgs/Pose pose\n\n# Velocity of the trajectory sample.\ngeometry_msgs/Twist velocity\n\n# Acceleration of the trajectory (optional, linear.x component is NaN if not set).\ngeometry_msgs/Accel acceleration\n\n# Force/Torque to apply at trajectory sample (optional, force.x component is NaN if not set).\ngeometry_msgs/Wrench effort\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                crate::codec::Message::validate(&self.velocity)?;
                crate::codec::Message::validate(&self.acceleration)?;
                crate::codec::Message::validate(&self.effort)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Trajectory {
            pub header: crate::std_msgs::msg::Header,
            pub points: ::std::vec::Vec<crate::nav_msgs::msg::TrajectoryPoint>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Trajectory {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Trajectory {
            const NAME: &'static str = "nav_msgs/msg/Trajectory";
            const SCHEMA: &'static str = "# An array of trajectory points that represents a trajectory for a robot to follow.\n\n# Indicates the frame_id in which the planner ran and timestamp represents when the trajectory was generated.\nstd_msgs/Header header\n\n# Array of trajectory points to follow.\nTrajectoryPoint[] points\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Accel\n# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque\n================================================================================\nMSG: nav_msgs/TrajectoryPoint\n# Trajectory point state\n\n# Absolute time and frame of reference of the point along a trajectory\nstd_msgs/Header header\n\n# Pose of the trajectory sample.\ngeometry_msgs/Pose pose\n\n# Velocity of the trajectory sample.\ngeometry_msgs/Twist velocity\n\n# Acceleration of the trajectory (optional, linear.x component is NaN if not set).\ngeometry_msgs/Accel acceleration\n\n# Force/Torque to apply at trajectory sample (optional, force.x component is NaN if not set).\ngeometry_msgs/Wrench effort\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
    }
}
pub mod sensor_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BatteryState {
            pub header: crate::std_msgs::msg::Header,
            pub voltage: f32,
            pub temperature: f32,
            pub current: f32,
            pub charge: f32,
            pub capacity: f32,
            pub design_capacity: f32,
            pub percentage: f32,
            pub power_supply_status: u8,
            pub power_supply_health: u8,
            pub power_supply_technology: u8,
            pub present: bool,
            pub cell_voltage: ::std::vec::Vec<f32>,
            pub cell_temperature: ::std::vec::Vec<f32>,
            pub location: ::std::string::String,
            pub serial_number: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BatteryState {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    voltage: ::std::default::Default::default(),
                    temperature: ::std::default::Default::default(),
                    current: ::std::default::Default::default(),
                    charge: ::std::default::Default::default(),
                    capacity: ::std::default::Default::default(),
                    design_capacity: ::std::default::Default::default(),
                    percentage: ::std::default::Default::default(),
                    power_supply_status: ::std::default::Default::default(),
                    power_supply_health: ::std::default::Default::default(),
                    power_supply_technology: ::std::default::Default::default(),
                    present: ::std::default::Default::default(),
                    cell_voltage: ::std::default::Default::default(),
                    cell_temperature: ::std::default::Default::default(),
                    location: ::std::default::Default::default(),
                    serial_number: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BatteryState {
            const NAME: &'static str = "sensor_msgs/msg/BatteryState";
            const SCHEMA: &'static str = "\n# Constants are chosen to match the enums in the linux kernel\n# defined in include/linux/power_supply.h as of version 3.7\n# The one difference is for style reasons the constants are\n# all uppercase not mixed case.\n\n# Power supply status constants\nuint8 POWER_SUPPLY_STATUS_UNKNOWN = 0\nuint8 POWER_SUPPLY_STATUS_CHARGING = 1\nuint8 POWER_SUPPLY_STATUS_DISCHARGING = 2\nuint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3\nuint8 POWER_SUPPLY_STATUS_FULL = 4\n\n# Power supply health constants\nuint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0\nuint8 POWER_SUPPLY_HEALTH_GOOD = 1\nuint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2\nuint8 POWER_SUPPLY_HEALTH_DEAD = 3\nuint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4\nuint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5\nuint8 POWER_SUPPLY_HEALTH_COLD = 6\nuint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7\nuint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8\n\n# Power supply technology (chemistry) constants\nuint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0 # Unknown battery technology\nuint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1    # Nickel-Metal Hydride battery\nuint8 POWER_SUPPLY_TECHNOLOGY_LION = 2    # Lithium-ion battery\nuint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3    # Lithium Polymer battery\nuint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4    # Lithium Iron Phosphate battery\nuint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5    # Nickel-Cadmium battery\nuint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6    # Lithium Manganese Dioxide battery\nuint8 POWER_SUPPLY_TECHNOLOGY_TERNARY = 7 # Ternary Lithium battery\nuint8 POWER_SUPPLY_TECHNOLOGY_VRLA = 8    # Valve Regulated Lead-Acid battery\n\nstd_msgs/Header  header\nfloat32 voltage          # Voltage in Volts (Mandatory)\nfloat32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)\nfloat32 current          # Negative when discharging (A)  (If unmeasured NaN)\nfloat32 charge           # Current charge in Ah  (If unmeasured NaN)\nfloat32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)\nfloat32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)\nfloat32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)\nuint8   power_supply_status     # The charging status as reported. Values defined above\nuint8   power_supply_health     # The battery health metric. Values defined above\nuint8   power_supply_technology # The battery chemistry. Values defined above\nbool    present          # True if the battery is present\n\nfloat32[] cell_voltage   # An array of individual cell voltages for each cell in the pack\n                         # If individual voltages unknown but number of cells known set each to NaN\nfloat32[] cell_temperature # An array of individual cell temperatures for each cell in the pack\n                           # If individual temperatures unknown but number of cells known set each to NaN\nstring location          # The location into which the battery is inserted. (slot number or plug)\nstring serial_number     # The best approximation of the battery serial number\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        impl BatteryState {
            pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
            pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
            pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
            pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
            pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;
            pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
            pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
            pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
            pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
            pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
            pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
            pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
            pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
            pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
            pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
            pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
            pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
            pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
            pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
            pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
            pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
            pub const POWER_SUPPLY_TECHNOLOGY_TERNARY: u8 = 7;
            pub const POWER_SUPPLY_TECHNOLOGY_VRLA: u8 = 8;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct RegionOfInterest {
            pub x_offset: u32,
            pub y_offset: u32,
            pub height: u32,
            pub width: u32,
            pub do_rectify: bool,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for RegionOfInterest {
            fn default() -> Self {
                Self {
                    x_offset: ::std::default::Default::default(),
                    y_offset: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    do_rectify: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for RegionOfInterest {
            const NAME: &'static str = "sensor_msgs/msg/RegionOfInterest";
            const SCHEMA: &'static str = "# This message is used to specify a region of interest within an image.\n#\n# When used to specify the ROI setting of the camera when the image was\n# taken, the height and width fields should either match the height and\n# width fields for the associated image; or height = width = 0\n# indicates that the full resolution image was captured.\n\nuint32 x_offset  # Leftmost pixel of the ROI\n                 # (0 if the ROI includes the left edge of the image)\nuint32 y_offset  # Topmost pixel of the ROI\n                 # (0 if the ROI includes the top edge of the image)\nuint32 height    # Height of ROI\nuint32 width     # Width of ROI\n\n# True if a distinct rectified ROI should be calculated from the \"raw\"\n# ROI in this message. Typically this should be False if the full image\n# is captured (ROI not used), and True if a subwindow is captured (ROI\n# used).\nbool do_rectify\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct CameraInfo {
            pub header: crate::std_msgs::msg::Header,
            pub height: u32,
            pub width: u32,
            pub distortion_model: ::std::string::String,
            pub d: ::std::vec::Vec<f64>,
            #[serde(with = "serde_big_array::BigArray")]
            pub k: [f64; 9],
            #[serde(with = "serde_big_array::BigArray")]
            pub r: [f64; 9],
            #[serde(with = "serde_big_array::BigArray")]
            pub p: [f64; 12],
            pub binning_x: u32,
            pub binning_y: u32,
            pub roi: crate::sensor_msgs::msg::RegionOfInterest,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for CameraInfo {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    distortion_model: ::std::default::Default::default(),
                    d: ::std::default::Default::default(),
                    k: ::std::array::from_fn(|_| ::std::default::Default::default()),
                    r: ::std::array::from_fn(|_| ::std::default::Default::default()),
                    p: ::std::array::from_fn(|_| ::std::default::Default::default()),
                    binning_x: ::std::default::Default::default(),
                    binning_y: ::std::default::Default::default(),
                    roi: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for CameraInfo {
            const NAME: &'static str = "sensor_msgs/msg/CameraInfo";
            const SCHEMA: &'static str = "# This message defines meta information for a camera. It should be in a\n# camera namespace on topic \"camera_info\" and accompanied by up to five\n# image topics named:\n#\n#   image_raw - raw data from the camera driver, possibly Bayer encoded\n#   image            - monochrome, distorted\n#   image_color      - color, distorted\n#   image_rect       - monochrome, rectified\n#   image_rect_color - color, rectified\n#\n# The image_pipeline contains packages (image_proc, stereo_image_proc)\n# for producing the four processed image topics from image_raw and\n# camera_info. The meaning of the camera parameters are described in\n# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.\n#\n# The image_geometry package provides a user-friendly interface to\n# common operations using this meta information. If you want to, e.g.,\n# project a 3d point into image coordinates, we strongly recommend\n# using image_geometry.\n#\n# If the camera is uncalibrated, the matrices D, K, R, P should be left\n# zeroed out. In particular, clients may assume that K[0] == 0.0\n# indicates an uncalibrated camera.\n\n#######################################################################\n#                     Image acquisition info                          #\n#######################################################################\n\n# Time of image acquisition, camera coordinate frame ID\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of camera\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into the plane of the image\n\n\n#######################################################################\n#                      Calibration Parameters                         #\n#######################################################################\n# These are fixed during camera calibration. Their values will be the #\n# same in all messages until the camera is recalibrated. Note that    #\n# self-calibrating systems may \"recalibrate\" frequently.              #\n#                                                                     #\n# The internal parameters can be used to warp a raw (distorted) image #\n# to:                                                                 #\n#   1. An undistorted image (requires D and K)                        #\n#   2. A rectified image (requires D, K, R)                           #\n# The projection matrix P projects 3D points into the rectified image.#\n#######################################################################\n\n# The image dimensions with which the camera was calibrated.\n# Normally this will be the full camera resolution in pixels.\nuint32 height\nuint32 width\n\n# The distortion model used. Supported models are listed in\n# sensor_msgs/distortion_models.hpp. For most cameras, \"plumb_bob\" - a\n# simple model of radial and tangential distortion - is sufficent.\nstring distortion_model\n\n# The distortion parameters, size depending on the distortion model.\n# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).\nfloat64[] d\n\n# Intrinsic camera matrix for the raw (distorted) images.\n#     [fx  0 cx]\n# K = [ 0 fy cy]\n#     [ 0  0  1]\n# Projects 3D points in the camera coordinate frame to 2D pixel\n# coordinates using the focal lengths (fx, fy) and principal point\n# (cx, cy).\nfloat64[9]  k # 3x3 row-major matrix\n\n# Rectification matrix (stereo cameras only)\n# A rotation matrix aligning the camera coordinate system to the ideal\n# stereo image plane so that epipolar lines in both stereo images are\n# parallel.\nfloat64[9]  r # 3x3 row-major matrix\n\n# Projection/camera matrix\n#     [fx'  0  cx' Tx]\n# P = [ 0  fy' cy' Ty]\n#     [ 0   0   1   0]\n# By convention, this matrix specifies the intrinsic (camera) matrix\n#  of the processed (rectified) image. That is, the left 3x3 portion\n#  is the normal camera intrinsic matrix for the rectified image.\n# It projects 3D points in the camera coordinate frame to 2D pixel\n#  coordinates using the focal lengths (fx', fy') and principal point\n#  (cx', cy') - these may differ from the values in K.\n# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will\n#  also have R = the identity and P[1:3,1:3] = K.\n# For a stereo pair, the fourth column [Tx Ty 0]' is related to the\n#  position of the optical center of the second camera in the first\n#  camera's frame. We assume Tz = 0 so both cameras are in the same\n#  stereo image plane. The first camera always has Tx = Ty = 0. For\n#  the right (second) camera of a horizontal stereo pair, Ty = 0 and\n#  Tx = -fx' * B, where B is the baseline between the cameras.\n# Given a 3D point [X Y Z]', the projection (x, y) of the point onto\n#  the rectified image is given by:\n#  [u v w]' = P * [X Y Z 1]'\n#         x = u / w\n#         y = v / w\n#  This holds for both images of a stereo pair.\nfloat64[12] p # 3x4 row-major matrix\n\n\n#######################################################################\n#                      Operational Parameters                         #\n#######################################################################\n# These define the image region actually captured by the camera       #\n# driver. Although they affect the geometry of the output image, they #\n# may be changed freely without recalibrating the camera.             #\n#######################################################################\n\n# Binning refers here to any camera setting which combines rectangular\n#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the\n#  resolution of the output image to\n#  (width / binning_x) x (height / binning_y).\n# The default values binning_x = binning_y = 0 is considered the same\n#  as binning_x = binning_y = 1 (no subsampling).\nuint32 binning_x\nuint32 binning_y\n\n# Region of interest (subwindow of full camera resolution), given in\n#  full resolution (unbinned) image coordinates. A particular ROI\n#  always denotes the same window of pixels on the camera sensor,\n#  regardless of binning settings.\n# The default setting of roi (all values 0) is considered the same as\n#  full resolution (roi.width = width, roi.height = height).\nRegionOfInterest roi\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: sensor_msgs/RegionOfInterest\n# This message is used to specify a region of interest within an image.\n#\n# When used to specify the ROI setting of the camera when the image was\n# taken, the height and width fields should either match the height and\n# width fields for the associated image; or height = width = 0\n# indicates that the full resolution image was captured.\n\nuint32 x_offset  # Leftmost pixel of the ROI\n                 # (0 if the ROI includes the left edge of the image)\nuint32 y_offset  # Topmost pixel of the ROI\n                 # (0 if the ROI includes the top edge of the image)\nuint32 height    # Height of ROI\nuint32 width     # Width of ROI\n\n# True if a distinct rectified ROI should be calculated from the \"raw\"\n# ROI in this message. Typically this should be False if the full image\n# is captured (ROI not used), and True if a subwindow is captured (ROI\n# used).\nbool do_rectify\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.roi)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ChannelFloat32 {
            pub name: ::std::string::String,
            pub values: ::std::vec::Vec<f32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ChannelFloat32 {
            fn default() -> Self {
                Self {
                    name: ::std::default::Default::default(),
                    values: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ChannelFloat32 {
            const NAME: &'static str = "sensor_msgs/msg/ChannelFloat32";
            const SCHEMA: &'static str = "# This message is used by the PointCloud message to hold optional data\n# associated with each point in the cloud. The length of the values\n# array should be the same as the length of the points array in the\n# PointCloud, and each value should be associated with the corresponding\n# point.\n#\n# Channel names in existing practice include:\n#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n#              This is opposite to usual conventions but remains for\n#              historical reasons. The newer PointCloud2 message has no\n#              such problem.\n#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n#           (R,G,B) values packed into the least significant 24 bits,\n#           in order.\n#   \"intensity\" - laser or pixel intensity.\n#   \"distance\"\n\n# The channel name should give semantics of the channel (e.g.\n# \"intensity\" instead of \"value\").\nstring name\n\n# The values array should be 1-1 with the elements of the associated\n# PointCloud.\nfloat32[] values\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct CompressedImage {
            pub header: crate::std_msgs::msg::Header,
            pub format: ::std::string::String,
            pub data: ::std::vec::Vec<u8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for CompressedImage {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    format: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for CompressedImage {
            const NAME: &'static str = "sensor_msgs/msg/CompressedImage";
            const SCHEMA: &'static str = "# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct FluidPressure {
            pub header: crate::std_msgs::msg::Header,
            pub fluid_pressure: f64,
            pub variance: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for FluidPressure {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    fluid_pressure: ::std::default::Default::default(),
                    variance: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for FluidPressure {
            const NAME: &'static str = "sensor_msgs/msg/FluidPressure";
            const SCHEMA: &'static str = "# Single pressure reading.  This message is appropriate for measuring the\n# pressure inside of a fluid (air, water, etc).  This also includes\n# atmospheric or barometric pressure.\n#\n# This message is not appropriate for force/pressure contact sensors.\n\nstd_msgs/Header header # timestamp of the measurement\n                             # frame_id is the location of the pressure sensor\n\nfloat64 fluid_pressure       # Absolute pressure reading in Pascals.\n\nfloat64 variance             # 0 is interpreted as variance unknown\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Illuminance {
            pub header: crate::std_msgs::msg::Header,
            pub illuminance: f64,
            pub variance: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Illuminance {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    illuminance: ::std::default::Default::default(),
                    variance: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Illuminance {
            const NAME: &'static str = "sensor_msgs/msg/Illuminance";
            const SCHEMA: &'static str = "# Single photometric illuminance measurement.  Light should be assumed to be\n# measured along the sensor's x-axis (the area of detection is the y-z plane).\n# The illuminance should have a 0 or positive value and be received with\n# the sensor's +X axis pointing toward the light source.\n#\n# Photometric illuminance is the measure of the human eye's sensitivity of the\n# intensity of light encountering or passing through a surface.\n#\n# All other Photometric and Radiometric measurements should not use this message.\n# This message cannot represent:\n#  - Luminous intensity (candela/light source output)\n#  - Luminance (nits/light output per area)\n#  - Irradiance (watt/area), etc.\n\nstd_msgs/Header header # timestamp is the time the illuminance was measured\n                             # frame_id is the location and direction of the reading\n\nfloat64 illuminance          # Measurement of the Photometric Illuminance in Lux.\n\nfloat64 variance             # 0 is interpreted as variance unknown\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Image {
            pub header: crate::std_msgs::msg::Header,
            pub height: u32,
            pub width: u32,
            pub encoding: ::std::string::String,
            pub is_bigendian: u8,
            pub step: u32,
            pub data: ::std::vec::Vec<u8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Image {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    encoding: ::std::default::Default::default(),
                    is_bigendian: ::std::default::Default::default(),
                    step: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Image {
            const NAME: &'static str = "sensor_msgs/msg/Image";
            const SCHEMA: &'static str = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n                             # If the frame_id here and the frame_id of the CameraInfo\n                             # message associated with the image conflict\n                             # the behavior is undefined\n\nuint32 height                # image height, that is, number of rows\nuint32 width                 # image width, that is, number of columns\n\n# The legal values for encoding are in file include/sensor_msgs/image_encodings.hpp\n# If you want to standardize a new string format, join\n# ros-users@lists.ros.org and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Imu {
            pub header: crate::std_msgs::msg::Header,
            pub orientation: crate::geometry_msgs::msg::Quaternion,
            #[serde(with = "serde_big_array::BigArray")]
            pub orientation_covariance: [f64; 9],
            pub angular_velocity: crate::geometry_msgs::msg::Vector3,
            #[serde(with = "serde_big_array::BigArray")]
            pub angular_velocity_covariance: [f64; 9],
            pub linear_acceleration: crate::geometry_msgs::msg::Vector3,
            #[serde(with = "serde_big_array::BigArray")]
            pub linear_acceleration_covariance: [f64; 9],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Imu {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    orientation: ::std::default::Default::default(),
                    orientation_covariance: ::std::array::from_fn(|_| {
                        ::std::default::Default::default()
                    }),
                    angular_velocity: ::std::default::Default::default(),
                    angular_velocity_covariance: ::std::array::from_fn(|_| {
                        ::std::default::Default::default()
                    }),
                    linear_acceleration: ::std::default::Default::default(),
                    linear_acceleration_covariance: ::std::array::from_fn(|_| {
                        ::std::default::Default::default()
                    }),
                }
            }
        }
        impl crate::codec::Message for Imu {
            const NAME: &'static str = "sensor_msgs/msg/Imu";
            const SCHEMA: &'static str = "# This is a message to hold data from an IMU (Inertial Measurement Unit)\n#\n# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec\n#\n# If the covariance of the measurement is known, it should be filled in (if all you know is the\n# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the\n# data a covariance will have to be assumed or gotten from some other source\n#\n# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an\n# orientation estimate), please set element 0 of the associated covariance matrix to -1\n# If you are interpreting this message, please check for a value of -1 in the first element of each\n# covariance matrix, and disregard the associated estimate.\n\nstd_msgs/Header header\n\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance # Row major x, y z\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.orientation)?;
                crate::codec::Message::validate(&self.angular_velocity)?;
                crate::codec::Message::validate(&self.linear_acceleration)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JointState {
            pub header: crate::std_msgs::msg::Header,
            pub name: ::std::vec::Vec<::std::string::String>,
            pub position: ::std::vec::Vec<f64>,
            pub velocity: ::std::vec::Vec<f64>,
            pub effort: ::std::vec::Vec<f64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JointState {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    name: ::std::default::Default::default(),
                    position: ::std::default::Default::default(),
                    velocity: ::std::default::Default::default(),
                    effort: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JointState {
            const NAME: &'static str = "sensor_msgs/msg/JointState";
            const SCHEMA: &'static str = "# This is a message that holds data to describe the state of a set of torque controlled joints.\n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and\n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state.\n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty.\n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nstd_msgs/Header header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Joy {
            pub header: crate::std_msgs::msg::Header,
            pub axes: ::std::vec::Vec<f32>,
            pub buttons: ::std::vec::Vec<i32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Joy {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    axes: ::std::default::Default::default(),
                    buttons: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Joy {
            const NAME: &'static str = "sensor_msgs/msg/Joy";
            const SCHEMA: &'static str = "# Reports the state of a joystick's axes and buttons.\n\n# The timestamp is the time at which data is received from the joystick.\nstd_msgs/Header header\n\n# The axes measurements from a joystick.\nfloat32[] axes\n\n# The buttons measurements from a joystick.\nint32[] buttons\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JoyFeedback {
            #[serde(rename = "type")]
            pub type_: u8,
            pub id: u8,
            pub intensity: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JoyFeedback {
            fn default() -> Self {
                Self {
                    type_: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                    intensity: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JoyFeedback {
            const NAME: &'static str = "sensor_msgs/msg/JoyFeedback";
            const SCHEMA: &'static str = "# Declare of the type of feedback\nuint8 TYPE_LED    = 0\nuint8 TYPE_RUMBLE = 1\nuint8 TYPE_BUZZER = 2\n\nuint8 type\n\n# This will hold an id number for each type of each feedback.\n# Example, the first led would be id=0, the second would be id=1\nuint8 id\n\n# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is\n# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.\nfloat32 intensity\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        impl JoyFeedback {
            pub const TYPE_LED: u8 = 0;
            pub const TYPE_RUMBLE: u8 = 1;
            pub const TYPE_BUZZER: u8 = 2;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JoyFeedbackArray {
            pub array: ::std::vec::Vec<crate::sensor_msgs::msg::JoyFeedback>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JoyFeedbackArray {
            fn default() -> Self {
                Self {
                    array: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JoyFeedbackArray {
            const NAME: &'static str = "sensor_msgs/msg/JoyFeedbackArray";
            const SCHEMA: &'static str = "# This message publishes values for multiple feedback at once.\nJoyFeedback[] array\n================================================================================\nMSG: sensor_msgs/JoyFeedback\n# Declare of the type of feedback\nuint8 TYPE_LED    = 0\nuint8 TYPE_RUMBLE = 1\nuint8 TYPE_BUZZER = 2\n\nuint8 type\n\n# This will hold an id number for each type of each feedback.\n# Example, the first led would be id=0, the second would be id=1\nuint8 id\n\n# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is\n# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.\nfloat32 intensity\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.array {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct LaserEcho {
            pub echoes: ::std::vec::Vec<f32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for LaserEcho {
            fn default() -> Self {
                Self {
                    echoes: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for LaserEcho {
            const NAME: &'static str = "sensor_msgs/msg/LaserEcho";
            const SCHEMA: &'static str = "# This message is a submessage of MultiEchoLaserScan and is not intended\n# to be used separately.\n\nfloat32[] echoes  # Multiple values of ranges or intensities.\n                  # Each array represents data from the same angle increment.\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct LaserScan {
            pub header: crate::std_msgs::msg::Header,
            pub angle_min: f32,
            pub angle_max: f32,
            pub angle_increment: f32,
            pub time_increment: f32,
            pub scan_time: f32,
            pub range_min: f32,
            pub range_max: f32,
            pub ranges: ::std::vec::Vec<f32>,
            pub intensities: ::std::vec::Vec<f32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for LaserScan {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    angle_min: ::std::default::Default::default(),
                    angle_max: ::std::default::Default::default(),
                    angle_increment: ::std::default::Default::default(),
                    time_increment: ::std::default::Default::default(),
                    scan_time: ::std::default::Default::default(),
                    range_min: ::std::default::Default::default(),
                    range_max: ::std::default::Default::default(),
                    ranges: ::std::default::Default::default(),
                    intensities: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for LaserScan {
            const NAME: &'static str = "sensor_msgs/msg/LaserScan";
            const SCHEMA: &'static str = "# Single scan from a planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nstd_msgs/Header header # timestamp in the header is the acquisition time of\n                             # the first ray in the scan.\n                             #\n                             # in frame frame_id, angles are measured around\n                             # the positive Z axis (counterclockwise, if Z is up)\n                             # with zero angle being forward along the x axis\n\nfloat32 angle_min            # start angle of the scan [rad]\nfloat32 angle_max            # end angle of the scan [rad]\nfloat32 angle_increment      # angular distance between measurements [rad]\n\nfloat32 time_increment       # time between measurements [seconds] - if your scanner\n                             # is moving, this will be used in interpolating position\n                             # of 3d points\nfloat32 scan_time            # time between scans [seconds]\n\nfloat32 range_min            # minimum range value [m]\nfloat32 range_max            # maximum range value [m]\n\nfloat32[] ranges             # range data [m]\n                             # (Note: values < range_min or > range_max should be discarded)\nfloat32[] intensities        # intensity data [device-specific units].  If your\n                             # device does not provide intensities, please leave\n                             # the array empty.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MagneticField {
            pub header: crate::std_msgs::msg::Header,
            pub magnetic_field: crate::geometry_msgs::msg::Vector3,
            #[serde(with = "serde_big_array::BigArray")]
            pub magnetic_field_covariance: [f64; 9],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MagneticField {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    magnetic_field: ::std::default::Default::default(),
                    magnetic_field_covariance: ::std::array::from_fn(|_| {
                        ::std::default::Default::default()
                    }),
                }
            }
        }
        impl crate::codec::Message for MagneticField {
            const NAME: &'static str = "sensor_msgs/msg/MagneticField";
            const SCHEMA: &'static str = "# Measurement of the Magnetic Field vector at a specific location.\n#\n# If the covariance of the measurement is known, it should be filled in.\n# If all you know is the variance of each measurement, e.g. from the datasheet,\n# just put those along the diagonal.\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\",\n# and to use the data a covariance will have to be assumed or gotten from some\n# other source.\n\nstd_msgs/Header header               # timestamp is the time the\n                                           # field was measured\n                                           # frame_id is the location and orientation\n                                           # of the field measurement\n\ngeometry_msgs/Vector3 magnetic_field # x, y, and z components of the\n                                           # field vector in Tesla\n                                           # If your sensor does not output 3 axes,\n                                           # put NaNs in the components not reported.\n\nfloat64[9] magnetic_field_covariance       # Row major about x, y, z axes\n                                           # 0 is interpreted as variance unknown\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.magnetic_field)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiDOFJointState {
            pub header: crate::std_msgs::msg::Header,
            pub joint_names: ::std::vec::Vec<::std::string::String>,
            pub transforms: ::std::vec::Vec<crate::geometry_msgs::msg::Transform>,
            pub twist: ::std::vec::Vec<crate::geometry_msgs::msg::Twist>,
            pub wrench: ::std::vec::Vec<crate::geometry_msgs::msg::Wrench>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiDOFJointState {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    joint_names: ::std::default::Default::default(),
                    transforms: ::std::default::Default::default(),
                    twist: ::std::default::Default::default(),
                    wrench: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiDOFJointState {
            const NAME: &'static str = "sensor_msgs/msg/MultiDOFJointState";
            const SCHEMA: &'static str = "# Representation of state for joints with multiple degrees of freedom,\n# following the structure of JointState which can only represent a single degree of freedom.\n#\n# It is assumed that a joint in a system corresponds to a transform that gets applied\n# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)\n# and those 3DOF can be expressed as a transformation matrix, and that transformation\n# matrix can be converted back to (x, y, yaw)\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state.\n# The goal is to make each of the fields optional. When e.g. your joints have no\n# wrench associated with them, you can leave the wrench array empty.\n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nstd_msgs/Header header\n\nstring[] joint_names\ngeometry_msgs/Transform[] transforms\ngeometry_msgs/Twist[] twist\ngeometry_msgs/Wrench[] wrench\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Transform\n# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into its linear and angular parts.\n\nVector3  force\nVector3  torque\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.transforms {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.twist {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.wrench {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiEchoLaserScan {
            pub header: crate::std_msgs::msg::Header,
            pub angle_min: f32,
            pub angle_max: f32,
            pub angle_increment: f32,
            pub time_increment: f32,
            pub scan_time: f32,
            pub range_min: f32,
            pub range_max: f32,
            pub ranges: ::std::vec::Vec<crate::sensor_msgs::msg::LaserEcho>,
            pub intensities: ::std::vec::Vec<crate::sensor_msgs::msg::LaserEcho>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiEchoLaserScan {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    angle_min: ::std::default::Default::default(),
                    angle_max: ::std::default::Default::default(),
                    angle_increment: ::std::default::Default::default(),
                    time_increment: ::std::default::Default::default(),
                    scan_time: ::std::default::Default::default(),
                    range_min: ::std::default::Default::default(),
                    range_max: ::std::default::Default::default(),
                    ranges: ::std::default::Default::default(),
                    intensities: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiEchoLaserScan {
            const NAME: &'static str = "sensor_msgs/msg/MultiEchoLaserScan";
            const SCHEMA: &'static str = "# Single scan from a multi-echo planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nstd_msgs/Header header # timestamp in the header is the acquisition time of\n                             # the first ray in the scan.\n                             #\n                             # in frame frame_id, angles are measured around\n                             # the positive Z axis (counterclockwise, if Z is up)\n                             # with zero angle being forward along the x axis\n\nfloat32 angle_min            # start angle of the scan [rad]\nfloat32 angle_max            # end angle of the scan [rad]\nfloat32 angle_increment      # angular distance between measurements [rad]\n\nfloat32 time_increment       # time between measurements [seconds] - if your scanner\n                             # is moving, this will be used in interpolating position\n                             # of 3d points\nfloat32 scan_time            # time between scans [seconds]\n\nfloat32 range_min            # minimum range value [m]\nfloat32 range_max            # maximum range value [m]\n\nLaserEcho[] ranges           # range data [m]\n                             # (Note: NaNs, values < range_min or > range_max should be discarded)\n                             # +Inf measurements are out of range\n                             # -Inf measurements are too close to determine exact distance.\nLaserEcho[] intensities      # intensity data [device-specific units].  If your\n                             # device does not provide intensities, please leave\n                             # the array empty.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: sensor_msgs/LaserEcho\n# This message is a submessage of MultiEchoLaserScan and is not intended\n# to be used separately.\n\nfloat32[] echoes  # Multiple values of ranges or intensities.\n                  # Each array represents data from the same angle increment.\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.ranges {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.intensities {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct NavSatStatus {
            pub status: i8,
            pub service: u16,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for NavSatStatus {
            fn default() -> Self {
                Self {
                    status: -2,
                    service: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for NavSatStatus {
            const NAME: &'static str = "sensor_msgs/msg/NavSatStatus";
            const SCHEMA: &'static str = "# Navigation Satellite fix status for any Global Navigation Satellite System.\n#\n# Whether to output an augmented fix is determined by both the fix\n# type and the last time differential corrections were received.  A\n# fix is valid when status >= STATUS_FIX.\n\nint8 STATUS_UNKNOWN = -2        # status is not yet set\nint8 STATUS_NO_FIX =  -1        # unable to fix position\nint8 STATUS_FIX =      0        # unaugmented fix\nint8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\nint8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n\nint8 status -2 # STATUS_UNKNOWN\n\n# Bits defining which Global Navigation Satellite System signals were\n# used by the receiver.\n\nuint16 SERVICE_UNKNOWN = 0  # Remember service is a bitfield, so checking (service & SERVICE_UNKNOWN) will not work. Use == instead.\nuint16 SERVICE_GPS =     1\nuint16 SERVICE_GLONASS = 2\nuint16 SERVICE_COMPASS = 4      # includes BeiDou.\nuint16 SERVICE_GALILEO = 8\n\nuint16 service\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        impl NavSatStatus {
            pub const STATUS_UNKNOWN: i8 = -2;
            pub const STATUS_NO_FIX: i8 = -1;
            pub const STATUS_FIX: i8 = 0;
            pub const STATUS_SBAS_FIX: i8 = 1;
            pub const STATUS_GBAS_FIX: i8 = 2;
            pub const SERVICE_UNKNOWN: u16 = 0;
            pub const SERVICE_GPS: u16 = 1;
            pub const SERVICE_GLONASS: u16 = 2;
            pub const SERVICE_COMPASS: u16 = 4;
            pub const SERVICE_GALILEO: u16 = 8;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct NavSatFix {
            pub header: crate::std_msgs::msg::Header,
            pub status: crate::sensor_msgs::msg::NavSatStatus,
            pub latitude: f64,
            pub longitude: f64,
            pub altitude: f64,
            #[serde(with = "serde_big_array::BigArray")]
            pub position_covariance: [f64; 9],
            pub position_covariance_type: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for NavSatFix {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    status: ::std::default::Default::default(),
                    latitude: ::std::default::Default::default(),
                    longitude: ::std::default::Default::default(),
                    altitude: ::std::default::Default::default(),
                    position_covariance: ::std::array::from_fn(|_| {
                        ::std::default::Default::default()
                    }),
                    position_covariance_type: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for NavSatFix {
            const NAME: &'static str = "sensor_msgs/msg/NavSatFix";
            const SCHEMA: &'static str = "# Navigation Satellite fix for any Global Navigation Satellite System\n#\n# Specified using the WGS 84 reference ellipsoid\n\n# header.stamp specifies the ROS time for this measurement (the\n#        corresponding satellite time may be reported using the\n#        sensor_msgs/TimeReference message).\n#\n# header.frame_id is the frame of reference reported by the satellite\n#        receiver, usually the location of the antenna.  This is a\n#        Euclidean frame relative to the vehicle, not a reference\n#        ellipsoid.\nstd_msgs/Header header\n\n# Satellite fix status information.\nNavSatStatus status\n\n# Latitude [degrees]. Positive is north of equator; negative is south.\nfloat64 latitude\n\n# Longitude [degrees]. Positive is east of prime meridian; negative is west.\nfloat64 longitude\n\n# Altitude [m]. Positive is above the WGS 84 ellipsoid\n# (quiet NaN if no altitude is available).\nfloat64 altitude\n\n# Position covariance [m^2] defined relative to a tangential plane\n# through the reported position. The components are East, North, and\n# Up (ENU), in row-major order.\n#\n# Beware: this coordinate system exhibits singularities at the poles.\nfloat64[9] position_covariance\n\n# If the covariance of the fix is known, fill it in completely. If the\n# GPS receiver provides the variance of each measurement, put them\n# along the diagonal. If only Dilution of Precision is available,\n# estimate an approximate covariance from that.\n\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\nuint8 COVARIANCE_TYPE_APPROXIMATED = 1\nuint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\nuint8 COVARIANCE_TYPE_KNOWN = 3\n\nuint8 position_covariance_type\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: sensor_msgs/NavSatStatus\n# Navigation Satellite fix status for any Global Navigation Satellite System.\n#\n# Whether to output an augmented fix is determined by both the fix\n# type and the last time differential corrections were received.  A\n# fix is valid when status >= STATUS_FIX.\n\nint8 STATUS_UNKNOWN = -2        # status is not yet set\nint8 STATUS_NO_FIX =  -1        # unable to fix position\nint8 STATUS_FIX =      0        # unaugmented fix\nint8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\nint8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n\nint8 status -2 # STATUS_UNKNOWN\n\n# Bits defining which Global Navigation Satellite System signals were\n# used by the receiver.\n\nuint16 SERVICE_UNKNOWN = 0  # Remember service is a bitfield, so checking (service & SERVICE_UNKNOWN) will not work. Use == instead.\nuint16 SERVICE_GPS =     1\nuint16 SERVICE_GLONASS = 2\nuint16 SERVICE_COMPASS = 4      # includes BeiDou.\nuint16 SERVICE_GALILEO = 8\n\nuint16 service\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.status)?;
                Ok(())
            }
        }
        impl NavSatFix {
            pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
            pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
            pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
            pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PointCloud {
            pub header: crate::std_msgs::msg::Header,
            pub points: ::std::vec::Vec<crate::geometry_msgs::msg::Point32>,
            pub channels: ::std::vec::Vec<crate::sensor_msgs::msg::ChannelFloat32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PointCloud {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                    channels: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PointCloud {
            const NAME: &'static str = "sensor_msgs/msg/PointCloud";
            const SCHEMA: &'static str = "## THIS MESSAGE IS DEPRECATED AS OF FOXY\n## Please use sensor_msgs/PointCloud2\n\n# This message holds a collection of 3d points, plus optional additional\n# information about each point.\n\n# Time of sensor data acquisition, coordinate frame ID.\nstd_msgs/Header header\n\n# Array of 3d points. Each Point32 should be interpreted as a 3d point\n# in the frame given in the header.\ngeometry_msgs/Point32[] points\n\n# Each channel should have the same number of elements as points array,\n# and the data in each channel should correspond 1:1 with each point.\n# Channel names in common practice are listed in ChannelFloat32.msg.\nChannelFloat32[] channels\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n================================================================================\nMSG: sensor_msgs/ChannelFloat32\n# This message is used by the PointCloud message to hold optional data\n# associated with each point in the cloud. The length of the values\n# array should be the same as the length of the points array in the\n# PointCloud, and each value should be associated with the corresponding\n# point.\n#\n# Channel names in existing practice include:\n#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n#              This is opposite to usual conventions but remains for\n#              historical reasons. The newer PointCloud2 message has no\n#              such problem.\n#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n#           (R,G,B) values packed into the least significant 24 bits,\n#           in order.\n#   \"intensity\" - laser or pixel intensity.\n#   \"distance\"\n\n# The channel name should give semantics of the channel (e.g.\n# \"intensity\" instead of \"value\").\nstring name\n\n# The values array should be 1-1 with the elements of the associated\n# PointCloud.\nfloat32[] values\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.channels {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PointField {
            pub name: ::std::string::String,
            pub offset: u32,
            pub datatype: u8,
            pub count: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PointField {
            fn default() -> Self {
                Self {
                    name: ::std::default::Default::default(),
                    offset: ::std::default::Default::default(),
                    datatype: ::std::default::Default::default(),
                    count: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PointField {
            const NAME: &'static str = "sensor_msgs/msg/PointField";
            const SCHEMA: &'static str = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\n# Common PointField names are x, y, z, intensity, rgb, rgba\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        impl PointField {
            pub const INT8: u8 = 1;
            pub const UINT8: u8 = 2;
            pub const INT16: u8 = 3;
            pub const UINT16: u8 = 4;
            pub const INT32: u8 = 5;
            pub const UINT32: u8 = 6;
            pub const FLOAT32: u8 = 7;
            pub const FLOAT64: u8 = 8;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct PointCloud2 {
            pub header: crate::std_msgs::msg::Header,
            pub height: u32,
            pub width: u32,
            pub fields: ::std::vec::Vec<crate::sensor_msgs::msg::PointField>,
            pub is_bigendian: bool,
            pub point_step: u32,
            pub row_step: u32,
            pub data: ::std::vec::Vec<u8>,
            pub is_dense: bool,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for PointCloud2 {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    height: ::std::default::Default::default(),
                    width: ::std::default::Default::default(),
                    fields: ::std::default::Default::default(),
                    is_bigendian: ::std::default::Default::default(),
                    point_step: ::std::default::Default::default(),
                    row_step: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                    is_dense: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for PointCloud2 {
            const NAME: &'static str = "sensor_msgs/msg/PointCloud2";
            const SCHEMA: &'static str = "# This message holds a collection of N-dimensional points, which may\n# contain additional information such as normals, intensity, etc. The\n# point data is stored as a binary blob, its layout described by the\n# contents of the \"fields\" array.\n#\n# The point cloud data may be organized 2d (image-like) or 1d (unordered).\n# Point clouds organized as 2d images may be produced by camera depth sensors\n# such as stereo or time-of-flight.\n\n# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).\nstd_msgs/Header header\n\n# 2D structure of the point cloud. If the cloud is unordered, height is\n# 1 and width is the length of the point cloud.\nuint32 height\nuint32 width\n\n# Describes the channels and their layout in the binary data blob.\nPointField[] fields\n\nbool    is_bigendian # Is this data bigendian?\nuint32  point_step   # Length of a point in bytes\nuint32  row_step     # Length of a row in bytes\nuint8[] data         # Actual point data, size is (row_step*height)\n\nbool is_dense        # True if there are no invalid points\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: sensor_msgs/PointField\n# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\n# Common PointField names are x, y, z, intensity, rgb, rgba\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.fields {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Range {
            pub header: crate::std_msgs::msg::Header,
            pub radiation_type: u8,
            pub field_of_view: f32,
            pub min_range: f32,
            pub max_range: f32,
            pub range: f32,
            pub variance: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Range {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    radiation_type: ::std::default::Default::default(),
                    field_of_view: ::std::default::Default::default(),
                    min_range: ::std::default::Default::default(),
                    max_range: ::std::default::Default::default(),
                    range: ::std::default::Default::default(),
                    variance: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Range {
            const NAME: &'static str = "sensor_msgs/msg/Range";
            const SCHEMA: &'static str = "# Single range reading from an active ranger that emits energy and reports\n# one range reading that is valid along an arc at the distance measured.\n# This message is  not appropriate for laser scanners. See the LaserScan\n# message if you are working with a laser scanner.\n#\n# This message also can represent a fixed-distance (binary) ranger.  This\n# sensor will have min_range===max_range===distance of detection.\n# These sensors follow REP 117 and will output -Inf if the object is detected\n# and +Inf if the object is outside of the detection range.\n\nstd_msgs/Header header # timestamp in the header is the time the ranger\n                             # returned the distance reading\n\n# Radiation type enums\n# If you want a value added to this list, send an email to the ros-users list\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\n\nuint8 radiation_type    # the type of radiation used by the sensor\n                        # (sound, IR, etc) [enum]\n\nfloat32 field_of_view   # the size of the arc that the distance reading is\n                        # valid for [rad]\n                        # the object causing the range reading may have\n                        # been anywhere within -field_of_view/2 and\n                        # field_of_view/2 at the measured range.\n                        # 0 angle corresponds to the x-axis of the sensor.\n\nfloat32 min_range       # minimum range value [m]\nfloat32 max_range       # maximum range value [m]\n                        # Fixed distance rangers require min_range==max_range\n\nfloat32 range           # range data [m]\n                        # (Note: values < range_min or > range_max should be discarded)\n                        # Fixed distance rangers only output -Inf or +Inf.\n                        # -Inf represents a detection within fixed distance.\n                        # (Detection too close to the sensor to quantify)\n                        # +Inf represents no detection within the fixed distance.\n                        # (Object out of range)\n\nfloat32 variance        # variance of the range sensor\n                        # 0 is interpreted as variance unknown\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        impl Range {
            pub const ULTRASOUND: u8 = 0;
            pub const INFRARED: u8 = 1;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct RelativeHumidity {
            pub header: crate::std_msgs::msg::Header,
            pub relative_humidity: f64,
            pub variance: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for RelativeHumidity {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    relative_humidity: ::std::default::Default::default(),
                    variance: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for RelativeHumidity {
            const NAME: &'static str = "sensor_msgs/msg/RelativeHumidity";
            const SCHEMA: &'static str = "# Single reading from a relative humidity sensor.\n# Defines the ratio of partial pressure of water vapor to the saturated vapor\n# pressure at a temperature.\n\nstd_msgs/Header header # timestamp of the measurement\n                             # frame_id is the location of the humidity sensor\n\nfloat64 relative_humidity    # Expression of the relative humidity\n                             # from 0.0 to 1.0.\n                             # 0.0 is no partial pressure of water vapor\n                             # 1.0 represents partial pressure of saturation\n\nfloat64 variance             # 0 is interpreted as variance unknown\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Temperature {
            pub header: crate::std_msgs::msg::Header,
            pub temperature: f64,
            pub variance: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Temperature {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    temperature: ::std::default::Default::default(),
                    variance: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Temperature {
            const NAME: &'static str = "sensor_msgs/msg/Temperature";
            const SCHEMA: &'static str = "# Single temperature reading.\n\nstd_msgs/Header header # timestamp is the time the temperature was measured\n                             # frame_id is the location of the temperature reading\n\nfloat64 temperature          # Measurement of the Temperature in Degrees Celsius.\n\nfloat64 variance             # 0 is interpreted as variance unknown.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TimeReference {
            pub header: crate::std_msgs::msg::Header,
            pub time_ref: crate::builtin_interfaces::msg::Time,
            pub source: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TimeReference {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    time_ref: ::std::default::Default::default(),
                    source: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TimeReference {
            const NAME: &'static str = "sensor_msgs/msg/TimeReference";
            const SCHEMA: &'static str = "# Measurement from an external time source not actively synchronized with the system clock.\n\nstd_msgs/Header header      # stamp is system time for which measurement was valid\n                                  # frame_id is not used\n\nbuiltin_interfaces/Time time_ref  # corresponding time from this external source\nstring source                     # (optional) name of time source\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.time_ref)?;
                Ok(())
            }
        }
    }
}
pub mod shape_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MeshTriangle {
            #[serde(with = "serde_big_array::BigArray")]
            pub vertex_indices: [u32; 3],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MeshTriangle {
            fn default() -> Self {
                Self {
                    vertex_indices: ::std::array::from_fn(|_| ::std::default::Default::default()),
                }
            }
        }
        impl crate::codec::Message for MeshTriangle {
            const NAME: &'static str = "shape_msgs/msg/MeshTriangle";
            const SCHEMA: &'static str =
                "# Definition of a triangle's vertices.\n\nuint32[3] vertex_indices\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Mesh {
            pub triangles: ::std::vec::Vec<crate::shape_msgs::msg::MeshTriangle>,
            pub vertices: ::std::vec::Vec<crate::geometry_msgs::msg::Point>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Mesh {
            fn default() -> Self {
                Self {
                    triangles: ::std::default::Default::default(),
                    vertices: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Mesh {
            const NAME: &'static str = "shape_msgs/msg/Mesh";
            const SCHEMA: &'static str = "# Definition of a mesh.\n\n# List of triangles; the index values refer to positions in vertices[].\nMeshTriangle[] triangles\n\n# The actual vertices that make up the mesh.\ngeometry_msgs/Point[] vertices\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: shape_msgs/MeshTriangle\n# Definition of a triangle's vertices.\n\nuint32[3] vertex_indices\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.triangles {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.vertices {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Plane {
            #[serde(with = "serde_big_array::BigArray")]
            pub coef: [f64; 4],
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Plane {
            fn default() -> Self {
                Self {
                    coef: ::std::array::from_fn(|_| ::std::default::Default::default()),
                }
            }
        }
        impl crate::codec::Message for Plane {
            const NAME: &'static str = "shape_msgs/msg/Plane";
            const SCHEMA: &'static str = "# Representation of a plane, using the plane equation ax + by + cz + d = 0.\n#\n# a := coef[0]\n# b := coef[1]\n# c := coef[2]\n# d := coef[3]\nfloat64[4] coef\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct SolidPrimitive {
            #[serde(rename = "type")]
            pub type_: u8,
            pub dimensions: ::std::vec::Vec<f64>,
            pub polygon: crate::geometry_msgs::msg::Polygon,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for SolidPrimitive {
            fn default() -> Self {
                Self {
                    type_: ::std::default::Default::default(),
                    dimensions: ::std::default::Default::default(),
                    polygon: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for SolidPrimitive {
            const NAME: &'static str = "shape_msgs/msg/SolidPrimitive";
            const SCHEMA: &'static str = "# Defines box, sphere, cylinder, cone and prism.\n# All shapes are defined to have their bounding boxes centered around 0,0,0.\n\nuint8 BOX=1\nuint8 SPHERE=2\nuint8 CYLINDER=3\nuint8 CONE=4\nuint8 PRISM=5\n\n# The type of the shape\nuint8 type\n\n# The dimensions of the shape\nfloat64[<=3] dimensions  # At no point will dimensions have a length > 3.\n\n# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.\n\n# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.\nuint8 BOX_X=0\nuint8 BOX_Y=1\nuint8 BOX_Z=2\n\n# For the SPHERE type, only one component is used, and it gives the radius of the sphere.\nuint8 SPHERE_RADIUS=0\n\n# For the CYLINDER and CONE types, the center line is oriented along the Z axis.\n# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the\n# height of the cylinder (cone).\n# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of\n# the base of the cylinder (cone).\n# Cone and cylinder primitives are defined to be circular. The tip of the cone\n# is pointing up, along +Z axis.\n\nuint8 CYLINDER_HEIGHT=0\nuint8 CYLINDER_RADIUS=1\n\nuint8 CONE_HEIGHT=0\nuint8 CONE_RADIUS=1\n\n# For the type PRISM, the center line is oriented along Z axis.\n# The PRISM_HEIGHT component of dimensions gives the\n# height of the prism.\n# The polygon defines the Z axis centered base of the prism.\n# The prism is constructed by extruding the base in +Z and -Z\n# directions by half of the PRISM_HEIGHT\n# Only x and y fields of the points are used in the polygon.\n# Points of the polygon are ordered counter-clockwise.\n\nuint8 PRISM_HEIGHT=0\ngeometry_msgs/Polygon polygon\n================================================================================\nMSG: geometry_msgs/Point32\n# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommended to use Point wherever possible instead of Point32.\n#\n# This recommendation is to promote interoperability.\n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.\n\nfloat32 x\nfloat32 y\nfloat32 z\n================================================================================\nMSG: geometry_msgs/Polygon\n# A specification of a polygon where the first and last points are assumed to be connected\n\nPoint32[] points\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                if self.dimensions.len() > 3 {
                    return Err("dimensions exceeds sequence bound".into());
                }
                crate::codec::Message::validate(&self.polygon)?;
                Ok(())
            }
        }
        impl SolidPrimitive {
            pub const BOX: u8 = 1;
            pub const SPHERE: u8 = 2;
            pub const CYLINDER: u8 = 3;
            pub const CONE: u8 = 4;
            pub const PRISM: u8 = 5;
            pub const BOX_X: u8 = 0;
            pub const BOX_Y: u8 = 1;
            pub const BOX_Z: u8 = 2;
            pub const SPHERE_RADIUS: u8 = 0;
            pub const CYLINDER_HEIGHT: u8 = 0;
            pub const CYLINDER_RADIUS: u8 = 1;
            pub const CONE_HEIGHT: u8 = 0;
            pub const CONE_RADIUS: u8 = 1;
            pub const PRISM_HEIGHT: u8 = 0;
        }
    }
}
pub mod std_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Header {
            pub stamp: crate::builtin_interfaces::msg::Time,
            pub frame_id: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Header {
            fn default() -> Self {
                Self {
                    stamp: ::std::default::Default::default(),
                    frame_id: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Header {
            const NAME: &'static str = "std_msgs/msg/Header";
            const SCHEMA: &'static str = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.stamp)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Bool {
            pub data: bool,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Bool {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Bool {
            const NAME: &'static str = "std_msgs/msg/Bool";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nbool data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Byte {
            pub data: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Byte {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Byte {
            const NAME: &'static str = "std_msgs/msg/Byte";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nbyte data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiArrayDimension {
            pub label: ::std::string::String,
            pub size: u32,
            pub stride: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiArrayDimension {
            fn default() -> Self {
                Self {
                    label: ::std::default::Default::default(),
                    size: ::std::default::Default::default(),
                    stride: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiArrayDimension {
            const NAME: &'static str = "std_msgs/msg/MultiArrayDimension";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiArrayLayout {
            pub dim: ::std::vec::Vec<crate::std_msgs::msg::MultiArrayDimension>,
            pub data_offset: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiArrayLayout {
            fn default() -> Self {
                Self {
                    dim: ::std::default::Default::default(),
                    data_offset: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiArrayLayout {
            const NAME: &'static str = "std_msgs/msg/MultiArrayLayout";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.dim {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ByteMultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<u8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ByteMultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ByteMultiArray {
            const NAME: &'static str = "std_msgs/msg/ByteMultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nbyte[]            data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Char {
            pub data: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Char {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Char {
            const NAME: &'static str = "std_msgs/msg/Char";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nchar data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ColorRGBA {
            pub r: f32,
            pub g: f32,
            pub b: f32,
            pub a: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ColorRGBA {
            fn default() -> Self {
                Self {
                    r: ::std::default::Default::default(),
                    g: ::std::default::Default::default(),
                    b: ::std::default::Default::default(),
                    a: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ColorRGBA {
            const NAME: &'static str = "std_msgs/msg/ColorRGBA";
            const SCHEMA: &'static str = "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Empty {
            #[serde(default)]
            _unused: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Empty {
            fn default() -> Self {
                Self { _unused: 0 }
            }
        }
        impl crate::codec::Message for Empty {
            const NAME: &'static str = "std_msgs/msg/Empty";
            const SCHEMA: &'static str = "\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Float32 {
            pub data: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Float32 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Float32 {
            const NAME: &'static str = "std_msgs/msg/Float32";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nfloat32 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Float32MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<f32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Float32MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Float32MultiArray {
            const NAME: &'static str = "std_msgs/msg/Float32MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat32[]         data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Float64 {
            pub data: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Float64 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Float64 {
            const NAME: &'static str = "std_msgs/msg/Float64";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nfloat64 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Float64MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<f64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Float64MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Float64MultiArray {
            const NAME: &'static str = "std_msgs/msg/Float64MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat64[]         data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int16 {
            pub data: i16,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int16 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int16 {
            const NAME: &'static str = "std_msgs/msg/Int16";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint16 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int16MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<i16>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int16MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int16MultiArray {
            const NAME: &'static str = "std_msgs/msg/Int16MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint16[]           data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int32 {
            pub data: i32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int32 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int32 {
            const NAME: &'static str = "std_msgs/msg/Int32";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint32 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int32MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<i32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int32MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int32MultiArray {
            const NAME: &'static str = "std_msgs/msg/Int32MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint32[]           data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int64 {
            pub data: i64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int64 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int64 {
            const NAME: &'static str = "std_msgs/msg/Int64";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint64 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int64MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<i64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int64MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int64MultiArray {
            const NAME: &'static str = "std_msgs/msg/Int64MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint64[]           data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int8 {
            pub data: i8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int8 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int8 {
            const NAME: &'static str = "std_msgs/msg/Int8";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nint8 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Int8MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<i8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Int8MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Int8MultiArray {
            const NAME: &'static str = "std_msgs/msg/Int8MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint8[]            data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct String {
            pub data: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for String {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for String {
            const NAME: &'static str = "std_msgs/msg/String";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt16 {
            pub data: u16,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt16 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt16 {
            const NAME: &'static str = "std_msgs/msg/UInt16";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint16 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt16MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<u16>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt16MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt16MultiArray {
            const NAME: &'static str = "std_msgs/msg/UInt16MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint16[]            data        # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt32 {
            pub data: u32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt32 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt32 {
            const NAME: &'static str = "std_msgs/msg/UInt32";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint32 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt32MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<u32>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt32MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt32MultiArray {
            const NAME: &'static str = "std_msgs/msg/UInt32MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint32[]          data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt64 {
            pub data: u64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt64 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt64 {
            const NAME: &'static str = "std_msgs/msg/UInt64";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint64 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt64MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<u64>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt64MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt64MultiArray {
            const NAME: &'static str = "std_msgs/msg/UInt64MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint64[]          data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt8 {
            pub data: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt8 {
            fn default() -> Self {
                Self {
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt8 {
            const NAME: &'static str = "std_msgs/msg/UInt8";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nuint8 data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UInt8MultiArray {
            pub layout: crate::std_msgs::msg::MultiArrayLayout,
            pub data: ::std::vec::Vec<u8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UInt8MultiArray {
            fn default() -> Self {
                Self {
                    layout: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UInt8MultiArray {
            const NAME: &'static str = "std_msgs/msg/UInt8MultiArray";
            const SCHEMA: &'static str = "# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint8[]           data          # array of data\n================================================================================\nMSG: std_msgs/MultiArrayDimension\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\nstring label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension\n================================================================================\nMSG: std_msgs/MultiArrayLayout\n# This was originally provided as an example message.\n# It is deprecated as of Foxy\n# It is recommended to create your own semantically meaningful message.\n# However if you would like to continue using this please use the equivalent in example_msgs.\n\n# The multiarray declares a generic multi-dimensional array of a\n# particular data type.  Dimensions are ordered from outer most\n# to inner most.\n#\n# Accessors should ALWAYS be written in terms of dimension stride\n# and specified outer-most dimension first.\n#\n# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n#\n# A standard, 3-channel 640x480 image with interleaved color channels\n# would be specified as:\n#\n# dim[0].label  = \"height\"\n# dim[0].size   = 480\n# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n# dim[1].label  = \"width\"\n# dim[1].size   = 640\n# dim[1].stride = 3*640 = 1920\n# dim[2].label  = \"channel\"\n# dim[2].size   = 3\n# dim[2].stride = 3\n#\n# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\nMultiArrayDimension[] dim # Array of dimension properties\nuint32 data_offset        # padding bytes at front of data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.layout)?;
                Ok(())
            }
        }
    }
}
pub mod tf2_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TF2Error {
            pub error: u8,
            pub error_string: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TF2Error {
            fn default() -> Self {
                Self {
                    error: ::std::default::Default::default(),
                    error_string: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TF2Error {
            const NAME: &'static str = "tf2_msgs/msg/TF2Error";
            const SCHEMA: &'static str = "uint8 NO_ERROR = 0\nuint8 LOOKUP_ERROR = 1\nuint8 CONNECTIVITY_ERROR = 2\nuint8 EXTRAPOLATION_ERROR = 3\nuint8 INVALID_ARGUMENT_ERROR = 4\nuint8 TIMEOUT_ERROR = 5\nuint8 TRANSFORM_ERROR = 6\n\nuint8 error\nstring error_string\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        impl TF2Error {
            pub const NO_ERROR: u8 = 0;
            pub const LOOKUP_ERROR: u8 = 1;
            pub const CONNECTIVITY_ERROR: u8 = 2;
            pub const EXTRAPOLATION_ERROR: u8 = 3;
            pub const INVALID_ARGUMENT_ERROR: u8 = 4;
            pub const TIMEOUT_ERROR: u8 = 5;
            pub const TRANSFORM_ERROR: u8 = 6;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct TFMessage {
            pub transforms: ::std::vec::Vec<crate::geometry_msgs::msg::TransformStamped>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for TFMessage {
            fn default() -> Self {
                Self {
                    transforms: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for TFMessage {
            const NAME: &'static str = "tf2_msgs/msg/TFMessage";
            const SCHEMA: &'static str = "geometry_msgs/TransformStamped[] transforms\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Transform\n# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/TransformStamped\n# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id at the time of header.stamp\n#\n# This message is mostly used by the\n# <a href=\"https://docs.ros.org/en/rolling/p/tf2/\">tf2</a> package.\n# See its documentation for more information.\n#\n# The child_frame_id is necessary in addition to the frame_id\n# in the Header to communicate the full reference for the transform\n# in a self contained message.\n\n# The frame id in the header is used as the reference frame of this transform.\nstd_msgs/Header header\n\n# The frame id of the child frame to which this transform points.\nstring child_frame_id\n\n# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.\nTransform transform\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.transforms {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
    }
}
pub mod trajectory_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JointTrajectoryPoint {
            pub positions: ::std::vec::Vec<f64>,
            pub velocities: ::std::vec::Vec<f64>,
            pub accelerations: ::std::vec::Vec<f64>,
            pub effort: ::std::vec::Vec<f64>,
            pub time_from_start: crate::builtin_interfaces::msg::Duration,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JointTrajectoryPoint {
            fn default() -> Self {
                Self {
                    positions: ::std::default::Default::default(),
                    velocities: ::std::default::Default::default(),
                    accelerations: ::std::default::Default::default(),
                    effort: ::std::default::Default::default(),
                    time_from_start: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JointTrajectoryPoint {
            const NAME: &'static str = "trajectory_msgs/msg/JointTrajectoryPoint";
            const SCHEMA: &'static str = "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n# or positions[, effort] for the trajectory to be executed.\n# All specified values are in the same order as the joint names in JointTrajectory.msg.\n\n# Single DOF joint positions for each joint relative to their \"0\" position.\n# The units depend on the specific joint type: radians for revolute or\n# continuous joints, and meters for prismatic joints.\nfloat64[] positions\n\n# The rate of change in position of each joint. Units are joint type dependent.\n# Radians/second for revolute or continuous joints, and meters/second for\n# prismatic joints.\nfloat64[] velocities\n\n# Rate of change in velocity of each joint. Units are joint type dependent.\n# Radians/second^2 for revolute or continuous joints, and meters/second^2 for\n# prismatic joints.\nfloat64[] accelerations\n\n# The torque or the force to be applied at each joint. For revolute/continuous\n# joints effort denotes a torque in newton-meters. For prismatic joints, effort\n# denotes a force in newtons.\nfloat64[] effort\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.time_from_start)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct JointTrajectory {
            pub header: crate::std_msgs::msg::Header,
            pub joint_names: ::std::vec::Vec<::std::string::String>,
            pub points: ::std::vec::Vec<crate::trajectory_msgs::msg::JointTrajectoryPoint>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for JointTrajectory {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    joint_names: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for JointTrajectory {
            const NAME: &'static str = "trajectory_msgs/msg/JointTrajectory";
            const SCHEMA: &'static str = "# The header is used to specify the coordinate frame and the reference time for\n# the trajectory durations\nstd_msgs/Header header\n\n# The names of the active joints in each trajectory point. These names are\n# ordered and must correspond to the values in each trajectory point.\nstring[] joint_names\n\n# Array of trajectory points, which describe the positions, velocities,\n# accelerations and/or efforts of the joints at each time point.\nJointTrajectoryPoint[] points\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: trajectory_msgs/JointTrajectoryPoint\n# Each trajectory point specifies either positions[, velocities[, accelerations]]\n# or positions[, effort] for the trajectory to be executed.\n# All specified values are in the same order as the joint names in JointTrajectory.msg.\n\n# Single DOF joint positions for each joint relative to their \"0\" position.\n# The units depend on the specific joint type: radians for revolute or\n# continuous joints, and meters for prismatic joints.\nfloat64[] positions\n\n# The rate of change in position of each joint. Units are joint type dependent.\n# Radians/second for revolute or continuous joints, and meters/second for\n# prismatic joints.\nfloat64[] velocities\n\n# Rate of change in velocity of each joint. Units are joint type dependent.\n# Radians/second^2 for revolute or continuous joints, and meters/second^2 for\n# prismatic joints.\nfloat64[] accelerations\n\n# The torque or the force to be applied at each joint. For revolute/continuous\n# joints effort denotes a torque in newton-meters. For prismatic joints, effort\n# denotes a force in newtons.\nfloat64[] effort\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiDOFJointTrajectoryPoint {
            pub transforms: ::std::vec::Vec<crate::geometry_msgs::msg::Transform>,
            pub velocities: ::std::vec::Vec<crate::geometry_msgs::msg::Twist>,
            pub accelerations: ::std::vec::Vec<crate::geometry_msgs::msg::Twist>,
            pub time_from_start: crate::builtin_interfaces::msg::Duration,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiDOFJointTrajectoryPoint {
            fn default() -> Self {
                Self {
                    transforms: ::std::default::Default::default(),
                    velocities: ::std::default::Default::default(),
                    accelerations: ::std::default::Default::default(),
                    time_from_start: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiDOFJointTrajectoryPoint {
            const NAME: &'static str = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint";
            const SCHEMA: &'static str = "# Each multi-dof joint can specify a transform (up to 6 DOF).\ngeometry_msgs/Transform[] transforms\n\n# There can be a velocity specified for the origin of the joint.\ngeometry_msgs/Twist[] velocities\n\n# There can be an acceleration specified for the origin of the joint.\ngeometry_msgs/Twist[] accelerations\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Transform\n# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.transforms {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.velocities {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.accelerations {
                    crate::codec::Message::validate(item)?;
                }
                crate::codec::Message::validate(&self.time_from_start)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MultiDOFJointTrajectory {
            pub header: crate::std_msgs::msg::Header,
            pub joint_names: ::std::vec::Vec<::std::string::String>,
            pub points: ::std::vec::Vec<crate::trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MultiDOFJointTrajectory {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    joint_names: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MultiDOFJointTrajectory {
            const NAME: &'static str = "trajectory_msgs/msg/MultiDOFJointTrajectory";
            const SCHEMA: &'static str = "# The header is used to specify the coordinate frame and the reference time for the trajectory durations\nstd_msgs/Header header\n\n# A representation of a multi-dof joint trajectory (each point is a transformation)\n# Each point along the trajectory will include an array of positions/velocities/accelerations\n# that has the same length as the array of joint names, and has the same order of joints as \n# the joint names array.\n\nstring[] joint_names\nMultiDOFJointTrajectoryPoint[] points\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Transform\n# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n\nVector3  linear\nVector3  angular\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: trajectory_msgs/MultiDOFJointTrajectoryPoint\n# Each multi-dof joint can specify a transform (up to 6 DOF).\ngeometry_msgs/Transform[] transforms\n\n# There can be a velocity specified for the origin of the joint.\ngeometry_msgs/Twist[] velocities\n\n# There can be an acceleration specified for the origin of the joint.\ngeometry_msgs/Twist[] accelerations\n\n# Desired time from the trajectory start to arrive at this trajectory point.\nbuiltin_interfaces/Duration time_from_start\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
    }
}
pub mod vision_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Point2D {
            pub x: f64,
            pub y: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Point2D {
            fn default() -> Self {
                Self {
                    x: ::std::default::Default::default(),
                    y: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Point2D {
            const NAME: &'static str = "vision_msgs/msg/Point2D";
            const SCHEMA: &'static str = "# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Pose2D {
            pub position: crate::vision_msgs::msg::Point2D,
            pub theta: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Pose2D {
            fn default() -> Self {
                Self {
                    position: ::std::default::Default::default(),
                    theta: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Pose2D {
            const NAME: &'static str = "vision_msgs/msg/Pose2D";
            const SCHEMA: &'static str = "# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.position)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox2D {
            pub center: crate::vision_msgs::msg::Pose2D,
            pub size_x: f64,
            pub size_y: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox2D {
            fn default() -> Self {
                Self {
                    center: ::std::default::Default::default(),
                    size_x: ::std::default::Default::default(),
                    size_y: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox2D {
            const NAME: &'static str = "vision_msgs/msg/BoundingBox2D";
            const SCHEMA: &'static str = "# A 2D bounding box that can be rotated about its center.\n# All dimensions are in pixels, but represented using floating-point\n#   values to allow sub-pixel precision. If an exact pixel crop is required\n#   for a rotated bounding box, it can be calculated using Bresenham's line\n#   algorithm.\n\n# The 2D position (in pixels) and orientation of the bounding box center.\nvision_msgs/Pose2D center\n\n# The total size (in pixels) of the bounding box surrounding the object relative\n#   to the pose of its center.\nfloat64 size_x\nfloat64 size_y\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n================================================================================\nMSG: vision_msgs/Pose2D\n# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.center)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox3D {
            pub center: crate::geometry_msgs::msg::Pose,
            pub size: crate::geometry_msgs::msg::Vector3,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox3D {
            fn default() -> Self {
                Self {
                    center: ::std::default::Default::default(),
                    size: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox3D {
            const NAME: &'static str = "vision_msgs/msg/BoundingBox3D";
            const SCHEMA: &'static str = "# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n# Dimensions of this box are in meters, and as such, it may be migrated to\n#   another package, such as geometry_msgs, in the future.\n\n# The 3D position and orientation of the bounding box center\ngeometry_msgs/Pose center\n\n# The total size of the bounding box, in meters, surrounding the object's center\n#   pose.\ngeometry_msgs/Vector3 size\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.center)?;
                crate::codec::Message::validate(&self.size)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox2DArray {
            pub header: crate::std_msgs::msg::Header,
            pub boxes: ::std::vec::Vec<crate::vision_msgs::msg::BoundingBox2D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox2DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    boxes: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox2DArray {
            const NAME: &'static str = "vision_msgs/msg/BoundingBox2DArray";
            const SCHEMA: &'static str = "std_msgs/Header header\nvision_msgs/BoundingBox2D[] boxes\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox2D\n# A 2D bounding box that can be rotated about its center.\n# All dimensions are in pixels, but represented using floating-point\n#   values to allow sub-pixel precision. If an exact pixel crop is required\n#   for a rotated bounding box, it can be calculated using Bresenham's line\n#   algorithm.\n\n# The 2D position (in pixels) and orientation of the bounding box center.\nvision_msgs/Pose2D center\n\n# The total size (in pixels) of the bounding box surrounding the object relative\n#   to the pose of its center.\nfloat64 size_x\nfloat64 size_y\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n================================================================================\nMSG: vision_msgs/Pose2D\n# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.boxes {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct BoundingBox3DArray {
            pub header: crate::std_msgs::msg::Header,
            pub boxes: ::std::vec::Vec<crate::vision_msgs::msg::BoundingBox3D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for BoundingBox3DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    boxes: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for BoundingBox3DArray {
            const NAME: &'static str = "vision_msgs/msg/BoundingBox3DArray";
            const SCHEMA: &'static str = "std_msgs/Header header\nvision_msgs/BoundingBox3D[] boxes\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox3D\n# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n# Dimensions of this box are in meters, and as such, it may be migrated to\n#   another package, such as geometry_msgs, in the future.\n\n# The 3D position and orientation of the bounding box center\ngeometry_msgs/Pose center\n\n# The total size of the bounding box, in meters, surrounding the object's center\n#   pose.\ngeometry_msgs/Vector3 size\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.boxes {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ObjectHypothesis {
            pub class_id: ::std::string::String,
            pub score: f64,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ObjectHypothesis {
            fn default() -> Self {
                Self {
                    class_id: ::std::default::Default::default(),
                    score: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ObjectHypothesis {
            const NAME: &'static str = "vision_msgs/msg/ObjectHypothesis";
            const SCHEMA: &'static str = "# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Classification {
            pub header: crate::std_msgs::msg::Header,
            pub results: ::std::vec::Vec<crate::vision_msgs::msg::ObjectHypothesis>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Classification {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    results: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Classification {
            const NAME: &'static str = "vision_msgs/msg/Classification";
            const SCHEMA: &'static str = "# Defines a classification result.\n#\n# This result does not contain any position information. It is designed for\n#   classifiers, which simply provide class probabilities given an instance of\n#   source data (e.g., an image or a point cloud).\n\nstd_msgs/Header header\n\n# A list of class probabilities. This list need not provide a probability for\n#   every possible class, just ones that are nonzero, or above some\n#   user-defined threshold.\nObjectHypothesis[] results\n\n# Source data that generated this classification are not a part of the message.\n# If you need to access them, use an exact or approximate time synchronizer in\n# your code, as this message's header should match the header of the source\n# data.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.results {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ObjectHypothesisWithPose {
            pub hypothesis: crate::vision_msgs::msg::ObjectHypothesis,
            pub pose: crate::geometry_msgs::msg::PoseWithCovariance,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ObjectHypothesisWithPose {
            fn default() -> Self {
                Self {
                    hypothesis: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ObjectHypothesisWithPose {
            const NAME: &'static str = "vision_msgs/msg/ObjectHypothesisWithPose";
            const SCHEMA: &'static str = "# An object hypothesis that contains pose information.\n# If you would like to define an array of ObjectHypothesisWithPose messages,\n#   please see the Detection2D or Detection3D message types.\n\n# The object hypothesis (ID and score).\nObjectHypothesis hypothesis\n\n# The 6D pose of the object hypothesis. This pose should be\n#   defined as the pose of some fixed reference point on the object, such as\n#   the geometric center of the bounding box, the center of mass of the\n#   object or the origin of a reference mesh of the object.\n# Note that this pose is not stamped; frame information can be defined by\n#   parent messages.\n# Also note that different classes predicted for the same input data may have\n#   different predicted 6D poses.\ngeometry_msgs/PoseWithCovariance pose\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.hypothesis)?;
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Detection2D {
            pub header: crate::std_msgs::msg::Header,
            pub results: ::std::vec::Vec<crate::vision_msgs::msg::ObjectHypothesisWithPose>,
            pub bbox: crate::vision_msgs::msg::BoundingBox2D,
            pub id: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Detection2D {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    results: ::std::default::Default::default(),
                    bbox: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Detection2D {
            const NAME: &'static str = "vision_msgs/msg/Detection2D";
            const SCHEMA: &'static str = "# Defines a 2D detection result.\n#\n# This is similar to a 2D classification, but includes position information,\n#   allowing a classification result for a specific crop or image point to\n#   to be located in the larger image.\n\nstd_msgs/Header header\n\n# Class probabilities\nObjectHypothesisWithPose[] results\n\n# 2D bounding box surrounding the object.\nBoundingBox2D bbox\n\n# ID used for consistency across multiple detection messages. Detections\n# of the same object in different detection messages should have the same id.\n# This field may be empty.\nstring id\n\n# Source data that generated this detection are not a part of the message.\n# If you need to access them, use an exact or approximate time synchronizer in\n# your code, as this message's header should match the header of the source\n# data.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox2D\n# A 2D bounding box that can be rotated about its center.\n# All dimensions are in pixels, but represented using floating-point\n#   values to allow sub-pixel precision. If an exact pixel crop is required\n#   for a rotated bounding box, it can be calculated using Bresenham's line\n#   algorithm.\n\n# The 2D position (in pixels) and orientation of the bounding box center.\nvision_msgs/Pose2D center\n\n# The total size (in pixels) of the bounding box surrounding the object relative\n#   to the pose of its center.\nfloat64 size_x\nfloat64 size_y\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n================================================================================\nMSG: vision_msgs/ObjectHypothesisWithPose\n# An object hypothesis that contains pose information.\n# If you would like to define an array of ObjectHypothesisWithPose messages,\n#   please see the Detection2D or Detection3D message types.\n\n# The object hypothesis (ID and score).\nObjectHypothesis hypothesis\n\n# The 6D pose of the object hypothesis. This pose should be\n#   defined as the pose of some fixed reference point on the object, such as\n#   the geometric center of the bounding box, the center of mass of the\n#   object or the origin of a reference mesh of the object.\n# Note that this pose is not stamped; frame information can be defined by\n#   parent messages.\n# Also note that different classes predicted for the same input data may have\n#   different predicted 6D poses.\ngeometry_msgs/PoseWithCovariance pose\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n================================================================================\nMSG: vision_msgs/Pose2D\n# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.results {
                    crate::codec::Message::validate(item)?;
                }
                crate::codec::Message::validate(&self.bbox)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Detection2DArray {
            pub header: crate::std_msgs::msg::Header,
            pub detections: ::std::vec::Vec<crate::vision_msgs::msg::Detection2D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Detection2DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    detections: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Detection2DArray {
            const NAME: &'static str = "vision_msgs/msg/Detection2DArray";
            const SCHEMA: &'static str = "# A list of 2D detections, for a multi-object 2D detector.\n\nstd_msgs/Header header\n\n# A list of the detected proposals. A multi-proposal detector might generate\n#   this list with many candidate detections generated from a single input.\nDetection2D[] detections\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox2D\n# A 2D bounding box that can be rotated about its center.\n# All dimensions are in pixels, but represented using floating-point\n#   values to allow sub-pixel precision. If an exact pixel crop is required\n#   for a rotated bounding box, it can be calculated using Bresenham's line\n#   algorithm.\n\n# The 2D position (in pixels) and orientation of the bounding box center.\nvision_msgs/Pose2D center\n\n# The total size (in pixels) of the bounding box surrounding the object relative\n#   to the pose of its center.\nfloat64 size_x\nfloat64 size_y\n================================================================================\nMSG: vision_msgs/Detection2D\n# Defines a 2D detection result.\n#\n# This is similar to a 2D classification, but includes position information,\n#   allowing a classification result for a specific crop or image point to\n#   to be located in the larger image.\n\nstd_msgs/Header header\n\n# Class probabilities\nObjectHypothesisWithPose[] results\n\n# 2D bounding box surrounding the object.\nBoundingBox2D bbox\n\n# ID used for consistency across multiple detection messages. Detections\n# of the same object in different detection messages should have the same id.\n# This field may be empty.\nstring id\n\n# Source data that generated this detection are not a part of the message.\n# If you need to access them, use an exact or approximate time synchronizer in\n# your code, as this message's header should match the header of the source\n# data.\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n================================================================================\nMSG: vision_msgs/ObjectHypothesisWithPose\n# An object hypothesis that contains pose information.\n# If you would like to define an array of ObjectHypothesisWithPose messages,\n#   please see the Detection2D or Detection3D message types.\n\n# The object hypothesis (ID and score).\nObjectHypothesis hypothesis\n\n# The 6D pose of the object hypothesis. This pose should be\n#   defined as the pose of some fixed reference point on the object, such as\n#   the geometric center of the bounding box, the center of mass of the\n#   object or the origin of a reference mesh of the object.\n# Note that this pose is not stamped; frame information can be defined by\n#   parent messages.\n# Also note that different classes predicted for the same input data may have\n#   different predicted 6D poses.\ngeometry_msgs/PoseWithCovariance pose\n================================================================================\nMSG: vision_msgs/Point2D\n# Represents a 2D point in pixel coordinates.\n# XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.\n\nfloat64 x\nfloat64 y\n================================================================================\nMSG: vision_msgs/Pose2D\n# Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.\n\nvision_msgs/Point2D position\nfloat64 theta\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.detections {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Detection3D {
            pub header: crate::std_msgs::msg::Header,
            pub results: ::std::vec::Vec<crate::vision_msgs::msg::ObjectHypothesisWithPose>,
            pub bbox: crate::vision_msgs::msg::BoundingBox3D,
            pub id: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Detection3D {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    results: ::std::default::Default::default(),
                    bbox: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Detection3D {
            const NAME: &'static str = "vision_msgs/msg/Detection3D";
            const SCHEMA: &'static str = "# Defines a 3D detection result.\n#\n# This extends a basic 3D classification by including the pose of the\n# detected object.\n\nstd_msgs/Header header\n\n# Class probabilities. Does not have to include hypotheses for all possible\n#   object ids, the scores for any ids not listed are assumed to be 0.\nObjectHypothesisWithPose[] results\n\n# 3D bounding box surrounding the object.\nBoundingBox3D bbox\n\n# ID used for consistency across multiple detection messages. Detections\n# of the same object in different detection messages should have the same id.\n# This field may be empty.\nstring id\n\n# Source data that generated this classification are not a part of the message.\n# If you need to access them, use an exact or approximate time synchronizer in\n# your code, as this message's header should match the header of the source\n# data.\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox3D\n# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n# Dimensions of this box are in meters, and as such, it may be migrated to\n#   another package, such as geometry_msgs, in the future.\n\n# The 3D position and orientation of the bounding box center\ngeometry_msgs/Pose center\n\n# The total size of the bounding box, in meters, surrounding the object's center\n#   pose.\ngeometry_msgs/Vector3 size\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n================================================================================\nMSG: vision_msgs/ObjectHypothesisWithPose\n# An object hypothesis that contains pose information.\n# If you would like to define an array of ObjectHypothesisWithPose messages,\n#   please see the Detection2D or Detection3D message types.\n\n# The object hypothesis (ID and score).\nObjectHypothesis hypothesis\n\n# The 6D pose of the object hypothesis. This pose should be\n#   defined as the pose of some fixed reference point on the object, such as\n#   the geometric center of the bounding box, the center of mass of the\n#   object or the origin of a reference mesh of the object.\n# Note that this pose is not stamped; frame information can be defined by\n#   parent messages.\n# Also note that different classes predicted for the same input data may have\n#   different predicted 6D poses.\ngeometry_msgs/PoseWithCovariance pose\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.results {
                    crate::codec::Message::validate(item)?;
                }
                crate::codec::Message::validate(&self.bbox)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Detection3DArray {
            pub header: crate::std_msgs::msg::Header,
            pub detections: ::std::vec::Vec<crate::vision_msgs::msg::Detection3D>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Detection3DArray {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    detections: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Detection3DArray {
            const NAME: &'static str = "vision_msgs/msg/Detection3DArray";
            const SCHEMA: &'static str = "# A list of 3D detections, for a multi-object 3D detector.\n\nstd_msgs/Header header\n\n# A list of the detected proposals. A multi-proposal detector might generate\n#   this list with many candidate detections generated from a single input.\nDetection3D[] detections\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/BoundingBox3D\n# A 3D bounding box that can be positioned and rotated about its center (6 DOF)\n# Dimensions of this box are in meters, and as such, it may be migrated to\n#   another package, such as geometry_msgs, in the future.\n\n# The 3D position and orientation of the bounding box center\ngeometry_msgs/Pose center\n\n# The total size of the bounding box, in meters, surrounding the object's center\n#   pose.\ngeometry_msgs/Vector3 size\n================================================================================\nMSG: vision_msgs/Detection3D\n# Defines a 3D detection result.\n#\n# This extends a basic 3D classification by including the pose of the\n# detected object.\n\nstd_msgs/Header header\n\n# Class probabilities. Does not have to include hypotheses for all possible\n#   object ids, the scores for any ids not listed are assumed to be 0.\nObjectHypothesisWithPose[] results\n\n# 3D bounding box surrounding the object.\nBoundingBox3D bbox\n\n# ID used for consistency across multiple detection messages. Detections\n# of the same object in different detection messages should have the same id.\n# This field may be empty.\nstring id\n\n# Source data that generated this classification are not a part of the message.\n# If you need to access them, use an exact or approximate time synchronizer in\n# your code, as this message's header should match the header of the source\n# data.\n================================================================================\nMSG: vision_msgs/ObjectHypothesis\n# An object hypothesis that contains no pose information.\n# If you would like to define an array of ObjectHypothesis messages,\n#   please see the Classification message type.\n\n# The unique ID of the object class. To get additional information about\n#   this ID, such as its human-readable class name, listeners should perform a\n#   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\nstring class_id\n\n# The probability or confidence value of the detected object. By convention,\n#   this value should lie in the range [0-1].\nfloat64 score\n================================================================================\nMSG: vision_msgs/ObjectHypothesisWithPose\n# An object hypothesis that contains pose information.\n# If you would like to define an array of ObjectHypothesisWithPose messages,\n#   please see the Detection2D or Detection3D message types.\n\n# The object hypothesis (ID and score).\nObjectHypothesis hypothesis\n\n# The 6D pose of the object hypothesis. This pose should be\n#   defined as the pose of some fixed reference point on the object, such as\n#   the geometric center of the bounding box, the center of mass of the\n#   object or the origin of a reference mesh of the object.\n# Note that this pose is not stamped; frame information can be defined by\n#   parent messages.\n# Also note that different classes predicted for the same input data may have\n#   different predicted 6D poses.\ngeometry_msgs/PoseWithCovariance pose\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.detections {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct VisionClass {
            pub class_id: u16,
            pub class_name: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for VisionClass {
            fn default() -> Self {
                Self {
                    class_id: ::std::default::Default::default(),
                    class_name: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for VisionClass {
            const NAME: &'static str = "vision_msgs/msg/VisionClass";
            const SCHEMA: &'static str = "# A key value pair that maps an integer class_id to a string class label\n#   in computer vision systems.\n\n# The int value that identifies the class.\n# Elements identified with 65535, the maximum uint16 value are assumed\n#   to belong to the \"UNLABELED\" class. For vision pipelines using less\n#   than 255 classes the \"UNLABELED\" is the maximum value in the uint8\n#   range.\nuint16 class_id\n\n# The name of the class represented by the class_id\nstring class_name\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct LabelInfo {
            pub header: crate::std_msgs::msg::Header,
            pub class_map: ::std::vec::Vec<crate::vision_msgs::msg::VisionClass>,
            pub threshold: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for LabelInfo {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    class_map: ::std::default::Default::default(),
                    threshold: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for LabelInfo {
            const NAME: &'static str = "vision_msgs/msg/LabelInfo";
            const SCHEMA: &'static str = "# Provides meta-information about a visual pipeline.\n#\n# This message serves a similar purpose to sensor_msgs/CameraInfo, but instead\n#   of being tied to hardware, it represents information about a specific\n#   computer vision pipeline. This information stays constant (or relatively\n#   constant) over time, and so it is wasteful to send it with each individual\n#   result. By listening to these messages, subscribers will receive\n#   the context in which published vision messages are to be interpreted.\n# Each vision pipeline should publish its LabelInfo messages to its own topic,\n#   in a manner similar to CameraInfo.\n# This message is meant to allow converting data from vision pipelines that\n#   return id based classifications back to human readable string class names.\n\n# Used for sequencing\nstd_msgs/Header header\n\n# An array of uint16 keys and string values containing the association\n#   between class identifiers and their names. According to the amount\n#   of classes and the datatype used to store their ids internally, the\n#   maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to\n#   the \"UNLABELED\" class.\nvision_msgs/VisionClass[] class_map \n\n# The value between 0-1 used as confidence threshold for the inference.\nfloat32 threshold\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: vision_msgs/VisionClass\n# A key value pair that maps an integer class_id to a string class label\n#   in computer vision systems.\n\n# The int value that identifies the class.\n# Elements identified with 65535, the maximum uint16 value are assumed\n#   to belong to the \"UNLABELED\" class. For vision pipelines using less\n#   than 255 classes the \"UNLABELED\" is the maximum value in the uint8\n#   range.\nuint16 class_id\n\n# The name of the class represented by the class_id\nstring class_name\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                for item in &self.class_map {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct VisionInfo {
            pub header: crate::std_msgs::msg::Header,
            pub method: ::std::string::String,
            pub database_location: ::std::string::String,
            pub database_version: i32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for VisionInfo {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    method: ::std::default::Default::default(),
                    database_location: ::std::default::Default::default(),
                    database_version: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for VisionInfo {
            const NAME: &'static str = "vision_msgs/msg/VisionInfo";
            const SCHEMA: &'static str = "# Provides meta-information about a visual pipeline.\n#\n# This message serves a similar purpose to sensor_msgs/CameraInfo, but instead\n#   of being tied to hardware, it represents information about a specific\n#   computer vision pipeline. This information stays constant (or relatively\n#   constant) over time, and so it is wasteful to send it with each individual\n#   result. By listening to these messages, subscribers will receive\n#   the context in which published vision messages are to be interpreted.\n# Each vision pipeline should publish its VisionInfo messages to its own topic,\n#   in a manner similar to CameraInfo.\n\n# Used for sequencing\nstd_msgs/Header header\n\n# Name of the vision pipeline. This should be a value that is meaningful to an\n#   outside user.\nstring method\n\n# Location where the metadata database is stored. The recommended location is\n#   as an XML string on the ROS parameter server, but the exact implementation\n#   and information is left up to the user.\n# The database should store information attached to class ids. Each\n#   class id should map to an atomic, visually recognizable element. This\n#   definition is intentionally vague to allow extreme flexibility. The\n#   elements could be classes in a pixel segmentation algorithm, object classes\n#   in a detector, different people's faces in a face detection algorithm, etc.\n#   Vision pipelines report results in terms of numeric IDs, which map into\n#   this  database.\n# The information stored in this database is, again, left up to the user. The\n#   database could be as simple as a map from ID to class name, or it could\n#   include information such as object meshes or colors to use for\n#   visualization.\nstring database_location\n\n# Metadata database version. This counter is incremented\n#   each time the pipeline begins using a new version of the database (useful\n#   in the case of online training or user modifications).\n#   The counter value can be monitored by listeners to ensure that the pipeline\n#   and the listener are using the same metadata.\nint32 database_version\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                Ok(())
            }
        }
    }
}
pub mod visualization_msgs {
    pub mod msg {
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct ImageMarker {
            pub header: crate::std_msgs::msg::Header,
            pub ns: ::std::string::String,
            pub id: i32,
            #[serde(rename = "type")]
            pub type_: i32,
            pub action: i32,
            pub position: crate::geometry_msgs::msg::Point,
            pub scale: f32,
            pub outline_color: crate::std_msgs::msg::ColorRGBA,
            pub filled: u8,
            pub fill_color: crate::std_msgs::msg::ColorRGBA,
            pub lifetime: crate::builtin_interfaces::msg::Duration,
            pub points: ::std::vec::Vec<crate::geometry_msgs::msg::Point>,
            pub outline_colors: ::std::vec::Vec<crate::std_msgs::msg::ColorRGBA>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for ImageMarker {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    ns: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                    type_: ::std::default::Default::default(),
                    action: ::std::default::Default::default(),
                    position: ::std::default::Default::default(),
                    scale: ::std::default::Default::default(),
                    outline_color: ::std::default::Default::default(),
                    filled: ::std::default::Default::default(),
                    fill_color: ::std::default::Default::default(),
                    lifetime: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                    outline_colors: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for ImageMarker {
            const NAME: &'static str = "visualization_msgs/msg/ImageMarker";
            const SCHEMA: &'static str = "int32 CIRCLE=0\nint32 LINE_STRIP=1\nint32 LINE_LIST=2\nint32 POLYGON=3\nint32 POINTS=4\n\nint32 ADD=0\nint32 REMOVE=1\n\nstd_msgs/Header header\n# Namespace which is used with the id to form a unique id.\nstring ns\n# Unique id within the namespace.\nint32 id\n# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.\nint32 type\n# Either ADD or REMOVE.\nint32 action\n# Two-dimensional coordinate position, in pixel-coordinates.\ngeometry_msgs/Point position\n# The scale of the object, e.g. the diameter for a CIRCLE.\nfloat32 scale\n# The outline color of the marker.\nstd_msgs/ColorRGBA outline_color\n# Whether or not to fill in the shape with color.\nuint8 filled\n# Fill color; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA fill_color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n\n# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.\ngeometry_msgs/Point[] points\n# The color for each line, point, etc. in the points field.\nstd_msgs/ColorRGBA[] outline_colors\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.position)?;
                crate::codec::Message::validate(&self.outline_color)?;
                crate::codec::Message::validate(&self.fill_color)?;
                crate::codec::Message::validate(&self.lifetime)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.outline_colors {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        impl ImageMarker {
            pub const CIRCLE: i32 = 0;
            pub const LINE_STRIP: i32 = 1;
            pub const LINE_LIST: i32 = 2;
            pub const POLYGON: i32 = 3;
            pub const POINTS: i32 = 4;
            pub const ADD: i32 = 0;
            pub const REMOVE: i32 = 1;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MeshFile {
            pub filename: ::std::string::String,
            pub data: ::std::vec::Vec<u8>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MeshFile {
            fn default() -> Self {
                Self {
                    filename: ::std::default::Default::default(),
                    data: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MeshFile {
            const NAME: &'static str = "visualization_msgs/msg/MeshFile";
            const SCHEMA: &'static str = "# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct UVCoordinate {
            pub u: f32,
            pub v: f32,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for UVCoordinate {
            fn default() -> Self {
                Self {
                    u: ::std::default::Default::default(),
                    v: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for UVCoordinate {
            const NAME: &'static str = "visualization_msgs/msg/UVCoordinate";
            const SCHEMA: &'static str = "# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct Marker {
            pub header: crate::std_msgs::msg::Header,
            pub ns: ::std::string::String,
            pub id: i32,
            #[serde(rename = "type")]
            pub type_: i32,
            pub action: i32,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub scale: crate::geometry_msgs::msg::Vector3,
            pub color: crate::std_msgs::msg::ColorRGBA,
            pub lifetime: crate::builtin_interfaces::msg::Duration,
            pub frame_locked: bool,
            pub points: ::std::vec::Vec<crate::geometry_msgs::msg::Point>,
            pub colors: ::std::vec::Vec<crate::std_msgs::msg::ColorRGBA>,
            pub texture_resource: ::std::string::String,
            pub texture: crate::sensor_msgs::msg::CompressedImage,
            pub uv_coordinates: ::std::vec::Vec<crate::visualization_msgs::msg::UVCoordinate>,
            pub text: ::std::string::String,
            pub mesh_resource: ::std::string::String,
            pub mesh_file: crate::visualization_msgs::msg::MeshFile,
            pub mesh_use_embedded_materials: bool,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for Marker {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    ns: ::std::default::Default::default(),
                    id: ::std::default::Default::default(),
                    type_: ::std::default::Default::default(),
                    action: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    scale: ::std::default::Default::default(),
                    color: ::std::default::Default::default(),
                    lifetime: ::std::default::Default::default(),
                    frame_locked: ::std::default::Default::default(),
                    points: ::std::default::Default::default(),
                    colors: ::std::default::Default::default(),
                    texture_resource: ::std::default::Default::default(),
                    texture: ::std::default::Default::default(),
                    uv_coordinates: ::std::default::Default::default(),
                    text: ::std::default::Default::default(),
                    mesh_resource: ::std::default::Default::default(),
                    mesh_file: ::std::default::Default::default(),
                    mesh_use_embedded_materials: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for Marker {
            const NAME: &'static str = "visualization_msgs/msg/Marker";
            const SCHEMA: &'static str = "# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                crate::codec::Message::validate(&self.scale)?;
                crate::codec::Message::validate(&self.color)?;
                crate::codec::Message::validate(&self.lifetime)?;
                for item in &self.points {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.colors {
                    crate::codec::Message::validate(item)?;
                }
                crate::codec::Message::validate(&self.texture)?;
                for item in &self.uv_coordinates {
                    crate::codec::Message::validate(item)?;
                }
                crate::codec::Message::validate(&self.mesh_file)?;
                Ok(())
            }
        }
        impl Marker {
            pub const ARROW: i32 = 0;
            pub const CUBE: i32 = 1;
            pub const SPHERE: i32 = 2;
            pub const CYLINDER: i32 = 3;
            pub const LINE_STRIP: i32 = 4;
            pub const LINE_LIST: i32 = 5;
            pub const CUBE_LIST: i32 = 6;
            pub const SPHERE_LIST: i32 = 7;
            pub const POINTS: i32 = 8;
            pub const TEXT_VIEW_FACING: i32 = 9;
            pub const MESH_RESOURCE: i32 = 10;
            pub const TRIANGLE_LIST: i32 = 11;
            pub const ARROW_STRIP: i32 = 12;
            pub const ADD: i32 = 0;
            pub const MODIFY: i32 = 0;
            pub const DELETE: i32 = 2;
            pub const DELETEALL: i32 = 3;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarkerControl {
            pub name: ::std::string::String,
            pub orientation: crate::geometry_msgs::msg::Quaternion,
            pub orientation_mode: u8,
            pub interaction_mode: u8,
            pub always_visible: bool,
            pub markers: ::std::vec::Vec<crate::visualization_msgs::msg::Marker>,
            pub independent_marker_orientation: bool,
            pub description: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarkerControl {
            fn default() -> Self {
                Self {
                    name: ::std::default::Default::default(),
                    orientation: ::std::default::Default::default(),
                    orientation_mode: ::std::default::Default::default(),
                    interaction_mode: ::std::default::Default::default(),
                    always_visible: ::std::default::Default::default(),
                    markers: ::std::default::Default::default(),
                    independent_marker_orientation: ::std::default::Default::default(),
                    description: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarkerControl {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarkerControl";
            const SCHEMA: &'static str = "# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0\nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n#\n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0\nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3\nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note:\n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as\n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\".\n# Default: A generic description based on the interaction mode\nstring description\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/Marker\n# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.orientation)?;
                for item in &self.markers {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        impl InteractiveMarkerControl {
            pub const INHERIT: u8 = 0;
            pub const FIXED: u8 = 1;
            pub const VIEW_FACING: u8 = 2;
            pub const NONE: u8 = 0;
            pub const MENU: u8 = 1;
            pub const BUTTON: u8 = 2;
            pub const MOVE_AXIS: u8 = 3;
            pub const MOVE_PLANE: u8 = 4;
            pub const ROTATE_AXIS: u8 = 5;
            pub const MOVE_ROTATE: u8 = 6;
            pub const MOVE_3D: u8 = 7;
            pub const ROTATE_3D: u8 = 8;
            pub const MOVE_ROTATE_3D: u8 = 9;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MenuEntry {
            pub id: u32,
            pub parent_id: u32,
            pub title: ::std::string::String,
            pub command: ::std::string::String,
            pub command_type: u8,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MenuEntry {
            fn default() -> Self {
                Self {
                    id: ::std::default::Default::default(),
                    parent_id: ::std::default::Default::default(),
                    title: ::std::default::Default::default(),
                    command: ::std::default::Default::default(),
                    command_type: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MenuEntry {
            const NAME: &'static str = "visualization_msgs/msg/MenuEntry";
            const SCHEMA: &'static str = "# MenuEntry message.\n#\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n#\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                Ok(())
            }
        }
        impl MenuEntry {
            pub const FEEDBACK: u8 = 0;
            pub const ROSRUN: u8 = 1;
            pub const ROSLAUNCH: u8 = 2;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarker {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub name: ::std::string::String,
            pub description: ::std::string::String,
            pub scale: f32,
            pub menu_entries: ::std::vec::Vec<crate::visualization_msgs::msg::MenuEntry>,
            pub controls: ::std::vec::Vec<crate::visualization_msgs::msg::InteractiveMarkerControl>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarker {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    name: ::std::default::Default::default(),
                    description: ::std::default::Default::default(),
                    scale: ::std::default::Default::default(),
                    menu_entries: ::std::default::Default::default(),
                    controls: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarker {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarker";
            const SCHEMA: &'static str = "# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/InteractiveMarkerControl\n# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0\nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n#\n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0\nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3\nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note:\n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as\n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\".\n# Default: A generic description based on the interaction mode\nstring description\n================================================================================\nMSG: visualization_msgs/Marker\n# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: visualization_msgs/MenuEntry\n# MenuEntry message.\n#\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n#\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                for item in &self.menu_entries {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.controls {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarkerFeedback {
            pub header: crate::std_msgs::msg::Header,
            pub client_id: ::std::string::String,
            pub marker_name: ::std::string::String,
            pub control_name: ::std::string::String,
            pub event_type: u8,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub menu_entry_id: u32,
            pub mouse_point: crate::geometry_msgs::msg::Point,
            pub mouse_point_valid: bool,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarkerFeedback {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    client_id: ::std::default::Default::default(),
                    marker_name: ::std::default::Default::default(),
                    control_name: ::std::default::Default::default(),
                    event_type: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    menu_entry_id: ::std::default::Default::default(),
                    mouse_point: ::std::default::Default::default(),
                    mouse_point_valid: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarkerFeedback {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarkerFeedback";
            const SCHEMA: &'static str = "# Time/frame info.\nstd_msgs/Header header\n\n# Identifying string. Must be unique in the topic namespace.\nstring client_id\n\n# Feedback message sent back from the GUI, e.g.\n# when the status of an interactive marker was modified by the user.\n\n# Specifies which interactive marker and control this message refers to\nstring marker_name\nstring control_name\n\n# Type of the event\n# KEEP_ALIVE: sent while dragging to keep up control of the marker\n# MENU_SELECT: a menu entry has been selected\n# BUTTON_CLICK: a button control has been clicked\n# POSE_UPDATE: the pose has been changed using one of the controls\nuint8 KEEP_ALIVE = 0\nuint8 POSE_UPDATE = 1\nuint8 MENU_SELECT = 2\nuint8 BUTTON_CLICK = 3\n\nuint8 MOUSE_DOWN = 4\nuint8 MOUSE_UP = 5\n\nuint8 event_type\n\n# Current pose of the marker\n# Note: Has to be valid for all feedback types.\ngeometry_msgs/Pose pose\n\n# Contains the ID of the selected menu entry\n# Only valid for MENU_SELECT events.\nuint32 menu_entry_id\n\n# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n# may contain the 3 dimensional position of the event on the\n# control.  If it does, mouse_point_valid will be true.  mouse_point\n# will be relative to the frame listed in the header.\ngeometry_msgs/Point mouse_point\nbool mouse_point_valid\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                crate::codec::Message::validate(&self.mouse_point)?;
                Ok(())
            }
        }
        impl InteractiveMarkerFeedback {
            pub const KEEP_ALIVE: u8 = 0;
            pub const POSE_UPDATE: u8 = 1;
            pub const MENU_SELECT: u8 = 2;
            pub const BUTTON_CLICK: u8 = 3;
            pub const MOUSE_DOWN: u8 = 4;
            pub const MOUSE_UP: u8 = 5;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarkerInit {
            pub server_id: ::std::string::String,
            pub seq_num: u64,
            pub markers: ::std::vec::Vec<crate::visualization_msgs::msg::InteractiveMarker>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarkerInit {
            fn default() -> Self {
                Self {
                    server_id: ::std::default::Default::default(),
                    seq_num: ::std::default::Default::default(),
                    markers: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarkerInit {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarkerInit";
            const SCHEMA: &'static str = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed a subsequent\n# update.  Every update message will have the same sequence number as\n# an init message.  Clients will likely want to unsubscribe from the\n# init topic after a successful initialization to avoid receiving\n# duplicate data.\nuint64 seq_num\n\n# All markers.\nInteractiveMarker[] markers\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/InteractiveMarker\n# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls\n================================================================================\nMSG: visualization_msgs/InteractiveMarkerControl\n# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0\nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n#\n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0\nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3\nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note:\n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as\n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\".\n# Default: A generic description based on the interaction mode\nstring description\n================================================================================\nMSG: visualization_msgs/Marker\n# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: visualization_msgs/MenuEntry\n# MenuEntry message.\n#\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n#\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.markers {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarkerPose {
            pub header: crate::std_msgs::msg::Header,
            pub pose: crate::geometry_msgs::msg::Pose,
            pub name: ::std::string::String,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarkerPose {
            fn default() -> Self {
                Self {
                    header: ::std::default::Default::default(),
                    pose: ::std::default::Default::default(),
                    name: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarkerPose {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarkerPose";
            const SCHEMA: &'static str = "\n# Time/frame info.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                crate::codec::Message::validate(&self.header)?;
                crate::codec::Message::validate(&self.pose)?;
                Ok(())
            }
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct InteractiveMarkerUpdate {
            pub server_id: ::std::string::String,
            pub seq_num: u64,
            #[serde(rename = "type")]
            pub type_: u8,
            pub markers: ::std::vec::Vec<crate::visualization_msgs::msg::InteractiveMarker>,
            pub poses: ::std::vec::Vec<crate::visualization_msgs::msg::InteractiveMarkerPose>,
            pub erases: ::std::vec::Vec<::std::string::String>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for InteractiveMarkerUpdate {
            fn default() -> Self {
                Self {
                    server_id: ::std::default::Default::default(),
                    seq_num: ::std::default::Default::default(),
                    type_: ::std::default::Default::default(),
                    markers: ::std::default::Default::default(),
                    poses: ::std::default::Default::default(),
                    erases: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for InteractiveMarkerUpdate {
            const NAME: &'static str = "visualization_msgs/msg/InteractiveMarkerUpdate";
            const SCHEMA: &'static str = "\n# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed an update.\nuint64 seq_num\n\n# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.\n# UPDATE: Incremental update to previous state.\n#         The sequence number must be 1 higher than for\n#         the previous update.\n# KEEP_ALIVE: Indicates the that the server is still living.\n#             The sequence number does not increase.\n#             No payload data should be filled out (markers, poses, or erases).\nuint8 KEEP_ALIVE = 0\nuint8 UPDATE = 1\n\nuint8 type\n\n# Note: No guarantees on the order of processing.\n#       Contents must be kept consistent by sender.\n\n# Markers to be added or updated\nInteractiveMarker[] markers\n\n# Poses of markers that should be moved\nInteractiveMarkerPose[] poses\n\n# Names of markers to be erased\nstring[] erases\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/InteractiveMarker\n# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls\n================================================================================\nMSG: visualization_msgs/InteractiveMarkerControl\n# Represents a control that is to be displayed together with an interactive marker\n\n# Identifying string for this control.\n# You need to assign a unique value to this to receive feedback from the GUI\n# on what actions the user performs on this control (e.g. a button click).\nstring name\n\n\n# Defines the local coordinate frame (relative to the pose of the parent\n# interactive marker) in which is being rotated and translated.\n# Default: Identity\ngeometry_msgs/Quaternion orientation\n\n\n# Orientation mode: controls how orientation changes.\n# INHERIT: Follow orientation of interactive marker\n# FIXED: Keep orientation fixed at initial state\n# VIEW_FACING: Align y-z plane with screen (x: forward, y:left, z:up).\nuint8 INHERIT = 0\nuint8 FIXED = 1\nuint8 VIEW_FACING = 2\n\nuint8 orientation_mode\n\n# Interaction mode for this control\n#\n# NONE: This control is only meant for visualization; no context menu.\n# MENU: Like NONE, but right-click menu is active.\n# BUTTON: Element can be left-clicked.\n# MOVE_AXIS: Translate along local x-axis.\n# MOVE_PLANE: Translate in local y-z plane.\n# ROTATE_AXIS: Rotate around local x-axis.\n# MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.\nuint8 NONE = 0\nuint8 MENU = 1\nuint8 BUTTON = 2\nuint8 MOVE_AXIS = 3\nuint8 MOVE_PLANE = 4\nuint8 ROTATE_AXIS = 5\nuint8 MOVE_ROTATE = 6\n# \"3D\" interaction modes work with the mouse+SHIFT+CTRL or with 3D cursors.\n# MOVE_3D: Translate freely in 3D space.\n# ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.\n# MOVE_ROTATE_3D: Full 6-DOF freedom of translation and rotation about the cursor origin.\nuint8 MOVE_3D = 7\nuint8 ROTATE_3D = 8\nuint8 MOVE_ROTATE_3D = 9\n\nuint8 interaction_mode\n\n\n# If true, the contained markers will also be visible\n# when the gui is not in interactive mode.\nbool always_visible\n\n\n# Markers to be displayed as custom visual representation.\n# Leave this empty to use the default control handles.\n#\n# Note:\n# - The markers can be defined in an arbitrary coordinate frame,\n#   but will be transformed into the local frame of the interactive marker.\n# - If the header of a marker is empty, its pose will be interpreted as\n#   relative to the pose of the parent interactive marker.\nMarker[] markers\n\n\n# In VIEW_FACING mode, set this to true if you don't want the markers\n# to be aligned with the camera view point. The markers will show up\n# as in INHERIT mode.\nbool independent_marker_orientation\n\n\n# Short description (< 40 characters) of what this control does,\n# e.g. \"Move the robot\".\n# Default: A generic description based on the interaction mode\nstring description\n================================================================================\nMSG: visualization_msgs/InteractiveMarkerPose\n\n# Time/frame info.\nstd_msgs/Header header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n================================================================================\nMSG: visualization_msgs/Marker\n# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: visualization_msgs/MenuEntry\n# MenuEntry message.\n#\n# Each InteractiveMarker message has an array of MenuEntry messages.\n# A collection of MenuEntries together describe a\n# menu/submenu/subsubmenu/etc tree, though they are stored in a flat\n# array.  The tree structure is represented by giving each menu entry\n# an ID number and a \"parent_id\" field.  Top-level entries are the\n# ones with parent_id = 0.  Menu entries are ordered within their\n# level the same way they are ordered in the containing array.  Parent\n# entries must appear before their children.\n#\n# Example:\n# - id = 3\n#   parent_id = 0\n#   title = \"fun\"\n# - id = 2\n#   parent_id = 0\n#   title = \"robot\"\n# - id = 4\n#   parent_id = 2\n#   title = \"pr2\"\n# - id = 5\n#   parent_id = 2\n#   title = \"turtle\"\n#\n# Gives a menu tree like this:\n#  - fun\n#  - robot\n#    - pr2\n#    - turtle\n\n# ID is a number for each menu entry.  Must be unique within the\n# control, and should never be 0.\nuint32 id\n\n# ID of the parent of this menu entry, if it is a submenu.  If this\n# menu entry is a top-level entry, set parent_id to 0.\nuint32 parent_id\n\n# menu / entry title\nstring title\n\n# Arguments to command indicated by command_type (below)\nstring command\n\n# Command_type stores the type of response desired when this menu\n# entry is clicked.\n# FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.\n# ROSRUN: execute \"rosrun\" with arguments given in the command field (above).\n# ROSLAUNCH: execute \"roslaunch\" with arguments given in the command field (above).\nuint8 FEEDBACK=0\nuint8 ROSRUN=1\nuint8 ROSLAUNCH=2\nuint8 command_type\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.markers {
                    crate::codec::Message::validate(item)?;
                }
                for item in &self.poses {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
        impl InteractiveMarkerUpdate {
            pub const KEEP_ALIVE: u8 = 0;
            pub const UPDATE: u8 = 1;
        }
        #[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        pub struct MarkerArray {
            pub markers: ::std::vec::Vec<crate::visualization_msgs::msg::Marker>,
        }
        // Explicit defaults also support .msg defaults and arrays longer than 32.
        #[allow(clippy::derivable_impls)]
        impl ::std::default::Default for MarkerArray {
            fn default() -> Self {
                Self {
                    markers: ::std::default::Default::default(),
                }
            }
        }
        impl crate::codec::Message for MarkerArray {
            const NAME: &'static str = "visualization_msgs/msg/MarkerArray";
            const SCHEMA: &'static str = "Marker[] markers\n================================================================================\nMSG: builtin_interfaces/Duration\n# Duration defines a period between two time points.\n# Messages of this datatype are of ROS Time following this design:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The duration -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The duration 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: builtin_interfaces/Time\n# This message communicates ROS Time defined here:\n# https://design.ros2.org/articles/clock_and_time.html\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. \n# e.g.\n# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}\n# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}\nuint32 nanosec\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation.\n\nPoint position\nQuaternion orientation\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space.\n\n# This is semantically different than a point.\n# A vector is always anchored at the origin.\n# When a transform is applied to a vector, only the rotational component is applied.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: sensor_msgs/CompressedImage\n# This message contains a compressed image.\n\nstd_msgs/Header header # Header timestamp should be acquisition time of image\n                             # Header frame_id should be optical frame of camera\n                             # origin of frame should be optical center of cameara\n                             # +x should point to the right in the image\n                             # +y should point down in the image\n                             # +z should point into to plane of the image\n\nstring format                # Specifies the format of the data\n                             # Acceptable values differ by the image transport used:\n                             # - compressed_image_transport:\n                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #   - CODEC is one of [jpeg, png, tiff]\n                             #   - COMPRESSED_PIXFMT is only appended for color images\n                             #     and is the pixel format used by the compression\n                             #     algorithm. Valid values for jpeg encoding are:\n                             #     [bgr8, rgb8]. Valid values for png encoding are:\n                             #     [bgr8, rgb8, bgr16, rgb16].\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as bgr8 or mono8\n                             #   jpeg image (depending on the number of channels).\n                             # - compressed_depth_image_transport:\n                             #     ORIG_PIXFMT; compressedDepth CODEC\n                             #   where:\n                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.\n                             #     the content of sensor_msgs/Image/encoding with\n                             #     values from include/sensor_msgs/image_encodings.h\n                             #     It is usually one of [16UC1, 32FC1].\n                             #   - CODEC is one of [png, rvl]\n                             #   If the field is empty or does not correspond to the\n                             #   above pattern, the image is treated as png image.\n                             # - Other image transports can store whatever values they\n                             #   need for successful decoding of the image. Refer to\n                             #   documentation of the other transports for details.\n\nuint8[] data                 # Compressed image buffer\n================================================================================\nMSG: std_msgs/ColorRGBA\nfloat32 r\nfloat32 g\nfloat32 b\nfloat32 a\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n================================================================================\nMSG: visualization_msgs/Marker\n# See:\n#  - http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n#  - http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes\n#\n# for more information on using this message with rviz.\n\nint32 ARROW=0\nint32 CUBE=1\nint32 SPHERE=2\nint32 CYLINDER=3\nint32 LINE_STRIP=4\nint32 LINE_LIST=5\nint32 CUBE_LIST=6\nint32 SPHERE_LIST=7\nint32 POINTS=8\nint32 TEXT_VIEW_FACING=9\nint32 MESH_RESOURCE=10\nint32 TRIANGLE_LIST=11\nint32 ARROW_STRIP=12\n\nint32 ADD=0\nint32 MODIFY=0\nint32 DELETE=2\nint32 DELETEALL=3\n\n# Header for timestamp and frame id.\nstd_msgs/Header header\n# Namespace in which to place the object.\n# Used in conjunction with id to create a unique name for the object.\nstring ns\n# Object ID used in conjunction with the namespace for manipulating and deleting the object later.\nint32 id\n# Type of object.\nint32 type\n# Action to take; one of:\n#  - 0 add/modify an object\n#  - 1 (deprecated)\n#  - 2 deletes an object (with the given ns and id)\n#  - 3 deletes all objects (or those with the given ns if any)\nint32 action\n# Pose of the object with respect the frame_id specified in the header.\ngeometry_msgs/Pose pose\n# Scale of the object; 1,1,1 means default (usually 1 meter square).\ngeometry_msgs/Vector3 scale\n# Color of the object; in the range: [0.0-1.0]\nstd_msgs/ColorRGBA color\n# How long the object should last before being automatically deleted.\n# 0 indicates forever.\nbuiltin_interfaces/Duration lifetime\n# If this marker should be frame-locked, i.e. retransformed into its frame every timestep.\nbool frame_locked\n\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ARROW_STRIP, etc.)\ngeometry_msgs/Point[] points\n# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, etc.)\n# The number of colors provided must either be 0 or equal to the number of points provided.\n# NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# Texture resource is a special URI that can either reference a texture file in\n# a format acceptable to (resource retriever)[https://docs.ros.org/en/rolling/p/resource_retriever/]\n# or an embedded texture via a string matching the format:\n#   \"embedded://texture_name\"\nstring texture_resource\n# An image to be loaded into the rendering engine as the texture for this marker.\n# This will be used iff texture_resource is set to embedded.\nsensor_msgs/CompressedImage texture\n# Location of each vertex within the texture; in the range: [0.0-1.0]\nUVCoordinate[] uv_coordinates\n\n# Only used for text markers\nstring text\n\n# Only used for MESH_RESOURCE markers.\n# Similar to texture_resource, mesh_resource uses resource retriever to load a mesh.\n# Optionally, a mesh file can be sent in-message via the mesh_file field. If doing so,\n# use the following format for mesh_resource:\n#   \"embedded://mesh_name\"\nstring mesh_resource\nMeshFile mesh_file\nbool mesh_use_embedded_materials\n================================================================================\nMSG: visualization_msgs/MeshFile\n# Used to send raw mesh files.\n\n# The filename is used for both debug purposes and to provide a file extension\n# for whatever parser is used.\nstring filename\n\n# This stores the raw text of the mesh file.\nuint8[] data\n================================================================================\nMSG: visualization_msgs/UVCoordinate\n# Location of the pixel as a ratio of the width of a 2D texture.\n# Values should be in range: [0.0-1.0].\nfloat32 u\nfloat32 v\n";
            fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {
                for item in &self.markers {
                    crate::codec::Message::validate(item)?;
                }
                Ok(())
            }
        }
    }
}
