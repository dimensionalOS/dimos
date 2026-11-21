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
//! Safe wrapper over the C capture shim (`csrc/capture.c`).

/// One captured frame, borrowed from the driver until the next `next` call.
pub struct Frame<'a> {
    /// JPEG when `jpeg` is set (hardware encoded), else the raw frame in the opened fourcc.
    pub data: &'a [u8],
    /// Bytes per row of a raw frame.
    pub stride: usize,
    pub jpeg: bool,
    /// The driver's capture timestamp, on whatever clock the driver stamps with.
    pub driver_stamp_s: f64,
    /// Time the hardware spent converting and encoding a `jpeg` frame.
    pub encode_s: f64,
}

#[cfg(v4l2_capture)]
mod ffi {
    use std::os::raw::{c_char, c_int};

    #[repr(C)]
    pub struct Capture {
        _private: [u8; 0],
    }

    #[repr(C)]
    pub struct CaptureFrame {
        pub data: *const u8,
        pub len: usize,
        pub stride: u32,
        pub jpeg: u32,
        pub stamp_s: f64,
        pub encode_s: f64,
    }

    extern "C" {
        #[allow(clippy::too_many_arguments)]
        pub fn capture_open(
            device: *const c_char,
            width: u32,
            height: u32,
            fourcc: u32,
            hardware: c_int,
            jpeg_quality: c_int,
            error: *mut c_char,
            error_len: usize,
        ) -> *mut Capture;
        pub fn capture_is_hardware(cap: *const Capture) -> c_int;
        pub fn capture_next(
            cap: *mut Capture,
            timeout_ms: c_int,
            frame: *mut CaptureFrame,
            error: *mut c_char,
            error_len: usize,
        ) -> c_int;
        pub fn capture_close(cap: *mut Capture);
    }
}

/// An open V4L2 capture device.
pub struct Capture {
    #[cfg(v4l2_capture)]
    raw: *mut ffi::Capture,
}

// The shim keeps no thread-local state; one owner drives it at a time.
unsafe impl Send for Capture {}

/// The four-character code V4L2 names a pixel format by.
pub fn fourcc(code: &str) -> Result<u32, String> {
    let bytes = code.as_bytes();
    if bytes.len() != 4 {
        return Err(format!("fourcc {code:?} is not four characters"));
    }
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

#[cfg(v4l2_capture)]
fn error_text(buffer: &[u8]) -> String {
    let end = buffer.iter().position(|&b| b == 0).unwrap_or(buffer.len());
    String::from_utf8_lossy(&buffer[..end]).into_owned()
}

impl Capture {
    /// Opens and starts streaming. With `hardware`, frames come back JPEG-encoded at `jpeg_quality` when the
    /// Jetson encoder is available; otherwise (or without it) they come back raw.
    #[cfg(v4l2_capture)]
    pub fn open(
        device: &str,
        width: u32,
        height: u32,
        fourcc: u32,
        hardware: bool,
        jpeg_quality: i32,
    ) -> Result<Self, String> {
        let path = std::ffi::CString::new(device).map_err(|e| e.to_string())?;
        let mut error = [0u8; 256];
        // SAFETY: path is NUL-terminated and error is a writable buffer of the given length.
        let raw = unsafe {
            ffi::capture_open(
                path.as_ptr(),
                width,
                height,
                fourcc,
                hardware as i32,
                jpeg_quality,
                error.as_mut_ptr().cast(),
                error.len(),
            )
        };
        if raw.is_null() {
            return Err(error_text(&error));
        }
        Ok(Self { raw })
    }

    #[cfg(not(v4l2_capture))]
    pub fn open(_: &str, _: u32, _: u32, _: u32, _: bool, _: i32) -> Result<Self, String> {
        Err("V4L2 capture needs Linux".into())
    }

    /// Whether frames come back JPEG-encoded by the hardware.
    pub fn is_hardware(&self) -> bool {
        #[cfg(v4l2_capture)]
        // SAFETY: raw is a live capture owned by self.
        return unsafe { ffi::capture_is_hardware(self.raw) } != 0;
        #[cfg(not(v4l2_capture))]
        false
    }

    /// The next frame, None on timeout. The frame borrows the driver's buffer until the next call.
    pub fn next(&mut self, timeout_ms: i32) -> Result<Option<Frame<'_>>, String> {
        #[cfg(v4l2_capture)]
        {
            let mut frame = ffi::CaptureFrame {
                data: std::ptr::null(),
                len: 0,
                stride: 0,
                jpeg: 0,
                stamp_s: 0.0,
                encode_s: 0.0,
            };
            let mut error = [0u8; 256];
            // SAFETY: raw is live; frame and error are writable for the call.
            let status = unsafe {
                ffi::capture_next(
                    self.raw,
                    timeout_ms,
                    &mut frame,
                    error.as_mut_ptr().cast(),
                    error.len(),
                )
            };
            match status {
                0 => Ok(None),
                1 => Ok(Some(Frame {
                    // SAFETY: the shim keeps this buffer queued-out until the next capture_next call, which
                    // needs &mut self and so cannot happen while this borrow lives.
                    data: unsafe { std::slice::from_raw_parts(frame.data, frame.len) },
                    stride: frame.stride as usize,
                    jpeg: frame.jpeg != 0,
                    driver_stamp_s: frame.stamp_s,
                    encode_s: frame.encode_s,
                })),
                _ => Err(error_text(&error)),
            }
        }
        #[cfg(not(v4l2_capture))]
        {
            let _ = timeout_ms;
            Err("V4L2 capture needs Linux".into())
        }
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        #[cfg(v4l2_capture)]
        // SAFETY: raw came from capture_open and is closed exactly once.
        unsafe {
            ffi::capture_close(self.raw)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_packs_little_endian_like_v4l2() {
        // V4L2_PIX_FMT_UYVY = v4l2_fourcc('U','Y','V','Y')
        assert_eq!(fourcc("UYVY").unwrap(), 0x5956_5955);
        assert!(fourcc("UYV").is_err());
    }
}
