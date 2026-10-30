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

//! Apple's System Management Controller, for the GPU temperatures M3 and later keep there (`Tg*` keys) rather than
//! in the IOHID sensors sysinfo reads. Unprivileged; the key list differs per chip, so it's discovered, not listed.

use std::ffi::{c_char, c_void};

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct KeyInfo {
    data_size: u32,
    data_type: u32,
    data_attributes: u8,
}

/// `SMCKeyData_t`, the struct AppleSMC's user client takes and returns (80 bytes).
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct KeyData {
    key: u32,
    vers: [u8; 6],
    p_limit_data: [u32; 4],
    key_info: KeyInfo,
    result: u8,
    status: u8,
    data8: u8,
    data32: u32,
    bytes: [u8; 32],
}

const HANDLE_YPC_EVENT: u32 = 2;
const READ_KEY: u8 = 5;
const GET_KEY_FROM_INDEX: u8 = 8;
const GET_KEY_INFO: u8 = 9;
const FLT: u32 = u32::from_be_bytes(*b"flt ");

#[link(name = "IOKit", kind = "framework")]
extern "C" {
    fn IOServiceMatching(name: *const c_char) -> *mut c_void;
    fn IOServiceGetMatchingService(main_port: u32, matching: *mut c_void) -> u32;
    fn IOServiceOpen(service: u32, owning_task: u32, kind: u32, connect: *mut u32) -> i32;
    fn IOServiceClose(connect: u32) -> i32;
    fn IOObjectRelease(object: u32) -> i32;
    fn IOConnectCallStructMethod(
        connect: u32,
        selector: u32,
        input: *const c_void,
        input_size: usize,
        output: *mut c_void,
        output_size: *mut usize,
    ) -> i32;
}

extern "C" {
    static mach_task_self_: u32;
}

pub struct Smc {
    connection: u32,
    /// the GPU temperature keys this chip has, with their info
    gpu_keys: Vec<(u32, KeyInfo)>,
}

impl Smc {
    /// None when there's no AppleSMC to open (not a Mac, or a sandbox).
    pub fn open() -> Option<Self> {
        // SAFETY: plain IOKit calls; the matching dictionary is consumed by IOServiceGetMatchingService.
        let connection = unsafe {
            let service = IOServiceGetMatchingService(0, IOServiceMatching(c"AppleSMC".as_ptr()));
            if service == 0 {
                return None;
            }
            let mut connection = 0;
            let opened = IOServiceOpen(service, mach_task_self_, 0, &mut connection);
            IOObjectRelease(service);
            if opened != 0 {
                return None;
            }
            connection
        };
        let mut smc = Self {
            connection,
            gpu_keys: Vec::new(),
        };
        let count = smc
            .read(u32::from_be_bytes(*b"#KEY"))
            .map(|(_, bytes)| u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            .unwrap_or(0);
        smc.gpu_keys = (0..count)
            .filter_map(|index| {
                let key = smc
                    .call(KeyData {
                        data8: GET_KEY_FROM_INDEX,
                        data32: index,
                        ..Default::default()
                    })?
                    .key;
                if key.to_be_bytes()[..2] != *b"Tg" {
                    return None;
                }
                Some((key, smc.key_info(key)?))
            })
            .filter(|(_, info)| info.data_type == FLT && info.data_size == 4)
            .collect();
        Some(smc)
    }

    fn call(&self, input: KeyData) -> Option<KeyData> {
        let mut output = KeyData::default();
        let mut size = std::mem::size_of::<KeyData>();
        // SAFETY: both buffers are a KeyData, the layout AppleSMC's selector 2 expects.
        let status = unsafe {
            IOConnectCallStructMethod(
                self.connection,
                HANDLE_YPC_EVENT,
                (&input as *const KeyData).cast(),
                std::mem::size_of::<KeyData>(),
                (&mut output as *mut KeyData).cast(),
                &mut size,
            )
        };
        (status == 0 && output.result == 0).then_some(output)
    }

    fn key_info(&self, key: u32) -> Option<KeyInfo> {
        Some(
            self.call(KeyData {
                key,
                data8: GET_KEY_INFO,
                ..Default::default()
            })?
            .key_info,
        )
    }

    fn read_with(&self, key: u32, info: KeyInfo) -> Option<[u8; 32]> {
        Some(
            self.call(KeyData {
                key,
                key_info: info,
                data8: READ_KEY,
                ..Default::default()
            })?
            .bytes,
        )
    }

    fn read(&self, key: u32) -> Option<(KeyInfo, [u8; 32])> {
        let info = self.key_info(key)?;
        Some((info, self.read_with(key, info)?))
    }

    /// Each GPU sensor's name (`Tg0U`, ...) and °C.
    pub fn gpu_temps(&self) -> Vec<(String, f32)> {
        self.gpu_keys
            .iter()
            .filter_map(|&(key, info)| {
                let bytes = self.read_with(key, info)?;
                // "flt " is a native-endian (little, on Apple silicon) IEEE float
                let celsius = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
                Some((
                    String::from_utf8_lossy(&key.to_be_bytes()).into_owned(),
                    celsius,
                ))
            })
            .collect()
    }
}

impl Drop for Smc {
    fn drop(&mut self) {
        // SAFETY: the connection was opened by IOServiceOpen and is closed once.
        unsafe { IOServiceClose(self.connection) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_data_matches_the_c_layout() {
        assert_eq!(std::mem::size_of::<KeyData>(), 80);
        assert_eq!(std::mem::offset_of!(KeyData, key_info), 28);
        assert_eq!(std::mem::offset_of!(KeyData, data8), 42);
        assert_eq!(std::mem::offset_of!(KeyData, bytes), 48);
    }

    #[test]
    fn reads_gpu_temperatures_where_there_is_an_smc() {
        let Some(smc) = Smc::open() else {
            return;
        };
        let start = std::time::Instant::now();
        let temps = smc.gpu_temps();
        println!("{} sensors in {:?}", temps.len(), start.elapsed());
        assert!(temps
            .iter()
            .all(|(_, celsius)| (-50.0..150.0).contains(celsius)));
    }
}
