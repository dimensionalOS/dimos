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

//! Simulation-only C ABI over the unmodified private go2web policy crate.

use go2_policy::obs::{sdk_to_train, train_to_sdk};
use go2_policy::{BlindController, BlindPolicy, FreewalkController, FreewalkPolicy};

pub enum Controller {
    Free(FreewalkController),
    Blind(BlindController),
}

#[no_mangle]
pub unsafe extern "C" fn create(
    kind: u32,
    bytes: *const u8,
    len: usize,
    home: *mut f32,
) -> *mut Controller {
    let blob = std::slice::from_raw_parts(bytes, len);
    let (controller, pose) = match kind {
        0 => (
            Controller::Free(FreewalkController::from_bytes(blob).unwrap()),
            FreewalkPolicy::from_bytes(blob).unwrap().default_pose,
        ),
        1 => (
            Controller::Blind(BlindController::from_bytes(blob).unwrap()),
            BlindPolicy::from_bytes(blob).unwrap().default_pose,
        ),
        _ => return std::ptr::null_mut(),
    };
    std::slice::from_raw_parts_mut(home, 12).copy_from_slice(&pose);
    Box::into_raw(Box::new(controller))
}

#[no_mangle]
pub unsafe extern "C" fn tick(controller: *mut Controller, input: *const f32, output: *mut f32) {
    // Input: quaternion(wxyz), body gyro, train-order q/dq, velocity command.
    let x = std::slice::from_raw_parts(input, 34);
    let q = train_to_sdk(x[7..19].try_into().unwrap());
    let dq = train_to_sdk(x[19..31].try_into().unwrap());
    let low = match &mut *controller {
        Controller::Free(c) => c.tick(
            x[..4].try_into().unwrap(),
            x[4..7].try_into().unwrap(),
            &q,
            &dq,
            x[31..34].try_into().unwrap(),
            0.31,
        ),
        Controller::Blind(c) => c.tick(
            x[..4].try_into().unwrap(),
            x[4..7].try_into().unwrap(),
            &q,
            &dq,
            x[31..34].try_into().unwrap(),
        ),
    };
    let out = std::slice::from_raw_parts_mut(output, 36);
    for (offset, getter) in [(0, 0), (12, 1), (24, 2)] {
        let sdk = std::array::from_fn(|i| match getter {
            0 => low.motor_cmd[i].q,
            1 => low.motor_cmd[i].kp,
            _ => low.motor_cmd[i].kd,
        });
        out[offset..offset + 12].copy_from_slice(&sdk_to_train(&sdk));
    }
}

#[no_mangle]
pub unsafe extern "C" fn destroy(controller: *mut Controller) {
    drop(Box::from_raw(controller));
}
