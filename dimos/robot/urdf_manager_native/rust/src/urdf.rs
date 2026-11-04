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

//! A materialized URDF's joints and the files it names, parsed as `dimos.robot.assets.model` parses them.

use std::collections::{BTreeSet, HashMap};

use dimos_module::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use dimos_module::Transform;

/// A joint parsed from a materialized URDF description.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub name: String,
    pub joint_type: String,
    pub parent_link: String,
    pub child_link: String,
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    // URDF's default axis is x.
    pub axis: [f64; 3],
    // (leader joint, multiplier, offset): this joint sits at multiplier * leader + offset.
    pub mimic: Option<(String, f64, f64)>,
}

impl Joint {
    /// Floating and planar joints get their pose from tf (odometry, a planar base), so they are read from tf, never published to it.
    pub fn is_multi_dof(&self) -> bool {
        matches!(self.joint_type.as_str(), "floating" | "planar")
    }

    /// The parent -> child transform at the joint's zero (URDF rpy is extrinsic x, y, z).
    pub fn origin(&self) -> Isometry3<f64> {
        let [x, y, z] = self.origin_xyz;
        let [roll, pitch, yaw] = self.origin_rpy;
        Isometry3::from_parts(
            Translation3::new(x, y, z),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        )
    }

    /// The parent -> child transform of a fixed or single-DOF joint at `position`, None for any other type.
    pub fn parent_from_child(&self, position: f64) -> Option<Isometry3<f64>> {
        let axis = Vector3::from(self.axis).normalize();
        let motion = match self.joint_type.as_str() {
            "fixed" => return Some(self.origin()),
            "revolute" | "continuous" => Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_scaled_axis(axis * position),
            ),
            "prismatic" => Isometry3::from_parts(
                Translation3::from(axis * position),
                UnitQuaternion::identity(),
            ),
            _ => return None,
        };
        Some(self.origin() * motion)
    }
}

/// The joints of a URDF in document order.
pub fn parse_joints(xml: &str) -> Result<Vec<Joint>, String> {
    let document = roxmltree::Document::parse(xml).map_err(|e| format!("URDF is not XML: {e}"))?;
    document
        .root_element()
        .children()
        .filter(|node| node.has_tag_name("joint"))
        .map(|joint| {
            let child = |tag: &str| joint.children().find(|node| node.has_tag_name(tag));
            let link_of = |tag: &str| {
                child(tag)
                    .and_then(|node| node.attribute("link"))
                    .unwrap_or("")
                    .to_string()
            };
            let origin = child("origin");
            let mimic = match child("mimic") {
                Some(mimic) => Some((
                    mimic.attribute("joint").unwrap_or("").to_string(),
                    parse_float(mimic.attribute("multiplier").unwrap_or("1"))?,
                    parse_float(mimic.attribute("offset").unwrap_or("0"))?,
                )),
                None => None,
            };
            Ok(Joint {
                name: joint.attribute("name").unwrap_or("").to_string(),
                joint_type: joint.attribute("type").unwrap_or("fixed").to_string(),
                parent_link: link_of("parent"),
                child_link: link_of("child"),
                origin_xyz: parse_triple(origin.and_then(|node| node.attribute("xyz")))?,
                origin_rpy: parse_triple(origin.and_then(|node| node.attribute("rpy")))?,
                axis: match child("axis") {
                    Some(axis) => parse_triple(Some(axis.attribute("xyz").unwrap_or("1 0 0")))?,
                    None => [1.0, 0.0, 0.0],
                },
                mimic,
            })
        })
        .collect()
}

/// The full path of every file the URDF names (mesh, texture, splat, ...): any filename attribute.
pub fn files_named_by(xml: &str) -> Result<BTreeSet<String>, String> {
    let document = roxmltree::Document::parse(xml).map_err(|e| format!("URDF is not XML: {e}"))?;
    Ok(document
        .descendants()
        .filter_map(|node| node.attribute("filename"))
        .map(str::to_string)
        .collect())
}

/// One parent -> child transform per fixed or single-DOF joint at the state's positions; joints the state leaves out stay at zero.
pub fn joint_transforms(
    joints: &[Joint],
    names: &[String],
    positions: &[f64],
    ts: f64,
) -> Vec<Transform> {
    let mut by_name: HashMap<&str, f64> = names
        .iter()
        .map(String::as_str)
        .zip(positions.iter().copied())
        .collect();
    for joint in joints {
        if let Some((leader, multiplier, offset)) = &joint.mimic {
            if let Some(&leader_position) = by_name.get(leader.as_str()) {
                by_name.insert(joint.name.as_str(), multiplier * leader_position + offset);
            }
        }
    }
    // Fixed joints are republished with the moving ones: a chained lookup is only as fresh as its stalest edge.
    joints
        .iter()
        .filter(|joint| !joint.is_multi_dof())
        .filter_map(|joint| {
            let position = by_name.get(joint.name.as_str()).copied().unwrap_or(0.0);
            let iso = joint.parent_from_child(position)?;
            Some(Transform::new(
                joint.parent_link.clone(),
                joint.child_link.clone(),
                ts,
                iso,
            ))
        })
        .collect()
}

fn parse_float(text: &str) -> Result<f64, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("not a number: {text:?}"))
}

fn parse_triple(text: Option<&str>) -> Result<[f64; 3], String> {
    let Some(text) = text else {
        return Ok([0.0; 3]);
    };
    let parts = text
        .split_whitespace()
        .map(parse_float)
        .collect::<Result<Vec<_>, _>>()?;
    parts
        .try_into()
        .map_err(|_| format!("Expected 3 values, got {text:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const URDF: &str = r#"<?xml version="1.0"?>
<robot name="test">
  <link name="base"><visual><geometry><mesh filename="/meshes/base.stl"/></geometry></visual></link>
  <link name="upper"><visual><geometry><mesh filename="/meshes/upper.dae"/></geometry>
    <material name="skin"><texture filename="/textures/skin.png"/></material></visual></link>
  <link name="slider"/>
  <link name="finger"/>
  <link name="camera"/>
  <link name="world"/>
  <joint name="shoulder" type="revolute">
    <origin xyz="0 0 0.5" rpy="0 0 1.5707963267948966"/>
    <parent link="base"/><child link="upper"/>
    <axis xyz="0 2 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <origin xyz="1 0 0"/>
    <parent link="upper"/><child link="slider"/>
    <axis xyz="0 0 1"/>
    <limit acceleration="2"/>
  </joint>
  <joint name="finger_joint" type="prismatic">
    <parent link="slider"/><child link="finger"/>
    <mimic joint="slide" multiplier="-2" offset="0.1"/>
  </joint>
  <joint name="camera_joint">
    <origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
    <parent link="upper"/><child link="camera"/>
  </joint>
  <joint name="odom" type="floating">
    <parent link="world"/><child link="base"/>
  </joint>
  <gazebo><plugin filename="libgazebo_ros_control.so"/></gazebo>
</robot>
"#;

    fn assert_iso(actual: &Transform, translation: [f64; 3], rotation: UnitQuaternion<f64>) {
        assert!(
            (actual.translation() - Vector3::from(translation)).norm() < 1e-9,
            "{}->{} translation {:?} != {translation:?}",
            actual.parent,
            actual.child,
            actual.translation()
        );
        assert!(
            actual.rotation().angle_to(&rotation) < 1e-9,
            "{}->{} rotation {:?} != {rotation:?}",
            actual.parent,
            actual.child,
            actual.rotation()
        );
    }

    #[test]
    fn joints_parse_in_document_order_with_urdf_defaults() {
        let joints = parse_joints(URDF).unwrap();

        let names: Vec<_> = joints.iter().map(|joint| joint.name.as_str()).collect();
        assert_eq!(
            names,
            ["shoulder", "slide", "finger_joint", "camera_joint", "odom"]
        );
        assert_eq!(joints[0].axis, [0.0, 2.0, 0.0]);
        assert_eq!(joints[1].origin_rpy, [0.0; 3]);
        // no axis element is x, a missing type is fixed
        assert_eq!(joints[2].axis, [1.0, 0.0, 0.0]);
        assert_eq!(joints[2].mimic, Some(("slide".to_string(), -2.0, 0.1)));
        assert_eq!(joints[3].joint_type, "fixed");
        assert!(joints[4].is_multi_dof());
    }

    #[test]
    fn a_bad_origin_is_an_error() {
        let urdf =
            r#"<robot name="r"><joint name="j" type="fixed"><origin xyz="1 2"/></joint></robot>"#;

        assert!(parse_joints(urdf)
            .unwrap_err()
            .contains("Expected 3 values"));
    }

    #[test]
    fn joint_transforms_match_hand_computed_poses() {
        let joints = parse_joints(URDF).unwrap();
        let names = ["shoulder".to_string(), "slide".to_string()];

        let transforms = joint_transforms(&joints, &names, &[FRAC_PI_2, 0.25], 7.5);

        let edges: Vec<_> = transforms
            .iter()
            .map(|t| (t.parent.as_str(), t.child.as_str()))
            .collect();
        // the floating joint is not published
        assert_eq!(
            edges,
            [
                ("base", "upper"),
                ("upper", "slider"),
                ("slider", "finger"),
                ("upper", "camera")
            ]
        );
        assert!(transforms.iter().all(|t| t.ts == 7.5));
        // origin yaw 90 deg, then 90 deg about the (normalized) y axis
        let shoulder = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2);
        assert_iso(&transforms[0], [0.0, 0.0, 0.5], shoulder);
        assert_iso(&transforms[1], [1.0, 0.0, 0.25], UnitQuaternion::identity());
        // mimic: -2 * 0.25 + 0.1 along x
        assert_iso(&transforms[2], [-0.4, 0.0, 0.0], UnitQuaternion::identity());
        // extrinsic xyz: Rz(0.3) * Ry(0.2) * Rx(0.1)
        let camera = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        assert_iso(&transforms[3], [0.1, 0.2, 0.3], camera);
    }

    #[test]
    fn joints_the_state_leaves_out_stay_at_zero() {
        let joints = parse_joints(URDF).unwrap();

        let transforms = joint_transforms(&joints, &[], &[], 0.0);

        assert_iso(
            &transforms[0],
            [0.0, 0.0, 0.5],
            UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
        );
        assert_iso(&transforms[1], [1.0, 0.0, 0.0], UnitQuaternion::identity());
        // a mimic whose leader is absent stays at zero too, not at its offset
        assert_iso(&transforms[2], [0.0, 0.0, 0.0], UnitQuaternion::identity());
    }

    #[test]
    fn every_filename_attribute_is_listed() {
        let files = files_named_by(URDF).unwrap();

        let files: Vec<_> = files.iter().map(String::as_str).collect();
        assert_eq!(
            files,
            [
                "/meshes/base.stl",
                "/meshes/upper.dae",
                "/textures/skin.png",
                "libgazebo_ros_control.so"
            ]
        );
    }
}
