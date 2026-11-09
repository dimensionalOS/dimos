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

//! The low-delay SVT-AV1 encoder, over `csrc/av1enc.c`. One frame in, one temporal unit out: under
//! `pred-struct=1` each frame predicts only from the one before it, so there is no reordering and no lookahead
//! to wait for.

/// How the encoder was asked to run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Settings {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub bitrate_bps: u32,
    /// Frames from one keyframe to the next.
    pub gop: u32,
    /// 0 slowest and best, 13 fastest.
    pub preset: i32,
    pub threads: u32,
}

/// One encoded temporal unit.
pub struct Unit<'a> {
    pub data: &'a [u8],
    pub keyframe: bool,
    /// The frame counter handed to `send`.
    pub pts: i64,
}

#[cfg(svt_av1)]
mod ffi {
    use std::os::raw::{c_char, c_int};

    #[repr(C)]
    pub struct Encoder {
        _private: [u8; 0],
    }

    extern "C" {
        #[allow(clippy::too_many_arguments)]
        pub fn dimos_av1_open(
            width: u32,
            height: u32,
            fps: u32,
            bitrate_bps: u32,
            gop: u32,
            preset: c_int,
            threads: u32,
            error: *mut c_char,
            error_len: usize,
        ) -> *mut Encoder;
        pub fn dimos_av1_stream_header(enc: *const Encoder, data: *mut *const u8, len: *mut usize);
        #[allow(clippy::too_many_arguments)]
        pub fn dimos_av1_send(
            enc: *mut Encoder,
            y: *const u8,
            u: *const u8,
            v: *const u8,
            y_stride: u32,
            uv_stride: u32,
            pts: i64,
            error: *mut c_char,
            error_len: usize,
        ) -> c_int;
        #[allow(clippy::too_many_arguments)]
        pub fn dimos_av1_next(
            enc: *mut Encoder,
            data: *mut *const u8,
            len: *mut usize,
            keyframe: *mut c_int,
            pts: *mut i64,
            error: *mut c_char,
            error_len: usize,
        ) -> c_int;
        pub fn dimos_av1_release(enc: *mut Encoder);
        pub fn dimos_av1_close(enc: *mut Encoder);
    }
}

/// An open encoder.
pub struct Encoder {
    #[cfg(svt_av1)]
    raw: *mut ffi::Encoder,
    settings: Settings,
}

// The shim keeps no thread-local state and one owner drives it at a time.
unsafe impl Send for Encoder {}

#[cfg(svt_av1)]
fn error_text(buffer: &[u8]) -> String {
    let end = buffer.iter().position(|&b| b == 0).unwrap_or(buffer.len());
    String::from_utf8_lossy(&buffer[..end]).into_owned()
}

/// SVT-AV1 refuses anything smaller in either direction (measured: 64x48 is `EB_ErrorBadParameter`,
/// 64x64 opens).
pub const MIN_SIDE: u32 = 64;

/// What SVT-AV1 accepts, checked here so the caller gets a sentence instead of an error code.
fn check(settings: &Settings) -> Result<(), String> {
    if settings.width < MIN_SIDE || settings.height < MIN_SIDE {
        return Err(format!(
            "SVT-AV1 will not encode {}x{}: both sides must be at least {MIN_SIDE}",
            settings.width, settings.height
        ));
    }
    // gop 1 means every frame is a keyframe, which SVT-AV1 calls ALL_INTRA, and it answers
    // "CBR Rate control is currently not supported for RANDOM_ACCESS/ALL_INTRA, use VBR mode".
    if settings.gop < 2 {
        return Err(
            "a gop of 1 is all-intra, which SVT-AV1 will not encode at a fixed bitrate".into(),
        );
    }
    Ok(())
}

impl Encoder {
    #[cfg(svt_av1)]
    pub fn open(settings: Settings) -> Result<Self, String> {
        check(&settings)?;
        let mut error = [0u8; 256];
        // SAFETY: error is writable for the length given; every other argument is a plain value.
        let raw = unsafe {
            ffi::dimos_av1_open(
                settings.width,
                settings.height,
                settings.fps,
                settings.bitrate_bps,
                settings.gop,
                settings.preset,
                settings.threads,
                error.as_mut_ptr().cast(),
                error.len(),
            )
        };
        if raw.is_null() {
            return Err(error_text(&error));
        }
        Ok(Self { raw, settings })
    }

    #[cfg(not(svt_av1))]
    pub fn open(settings: Settings) -> Result<Self, String> {
        check(&settings)?;
        Err("this build has no SVT-AV1".into())
    }

    pub fn settings(&self) -> Settings {
        self.settings
    }

    /// The sequence header a decoder needs before any frame. It goes in front of every keyframe, so a page that
    /// subscribes mid-stream can start at the next one.
    #[cfg(svt_av1)]
    pub fn stream_header(&self) -> Vec<u8> {
        let mut data: *const u8 = std::ptr::null();
        let mut len: usize = 0;
        // SAFETY: raw is a live encoder; both out-pointers are writable.
        unsafe { ffi::dimos_av1_stream_header(self.raw, &mut data, &mut len) };
        if data.is_null() || len == 0 {
            return Vec::new();
        }
        // SAFETY: the shim holds this buffer until close, which needs ownership of self.
        unsafe { std::slice::from_raw_parts(data, len) }.to_vec()
    }

    /// Without SVT-AV1 there is no stream and so no header.
    #[cfg(not(svt_av1))]
    pub fn stream_header(&self) -> Vec<u8> {
        Vec::new()
    }

    /// Feeds one 8-bit I420 frame. The planes are read during the call.
    pub fn send(
        &mut self,
        y: &[u8],
        u: &[u8],
        v: &[u8],
        y_stride: u32,
        uv_stride: u32,
        pts: i64,
    ) -> Result<(), String> {
        let (width, height) = (self.settings.width as usize, self.settings.height as usize);
        let (half_w, half_h) = (width.div_ceil(2), height.div_ceil(2));
        if (y_stride as usize) < width || (uv_stride as usize) < half_w {
            return Err(format!(
                "strides {y_stride}/{uv_stride} are too narrow for {width}x{height}"
            ));
        }
        if y.len() < y_stride as usize * height
            || u.len() < uv_stride as usize * half_h
            || v.len() < uv_stride as usize * half_h
        {
            return Err(format!(
                "planes of {}/{}/{} bytes do not hold {width}x{height}",
                y.len(),
                u.len(),
                v.len()
            ));
        }
        #[cfg(svt_av1)]
        {
            let mut error = [0u8; 256];
            // SAFETY: raw is live and the three planes are readable for the sizes checked above.
            let status = unsafe {
                ffi::dimos_av1_send(
                    self.raw,
                    y.as_ptr(),
                    u.as_ptr(),
                    v.as_ptr(),
                    y_stride,
                    uv_stride,
                    pts,
                    error.as_mut_ptr().cast(),
                    error.len(),
                )
            };
            if status != 0 {
                return Err(error_text(&error));
            }
            Ok(())
        }
        #[cfg(not(svt_av1))]
        {
            let _ = pts;
            Err("this build has no SVT-AV1".into())
        }
    }

    /// The temporal unit for the frame just sent, copied out. `None` when the encoder has nothing more.
    pub fn next_unit(&mut self) -> Result<Option<(Vec<u8>, bool, i64)>, String> {
        #[cfg(svt_av1)]
        {
            let mut data: *const u8 = std::ptr::null();
            let mut len: usize = 0;
            let mut keyframe: std::os::raw::c_int = 0;
            let mut pts: i64 = 0;
            let mut error = [0u8; 256];
            // SAFETY: raw is live; every out-pointer is writable for the call.
            let status = unsafe {
                ffi::dimos_av1_next(
                    self.raw,
                    &mut data,
                    &mut len,
                    &mut keyframe,
                    &mut pts,
                    error.as_mut_ptr().cast(),
                    error.len(),
                )
            };
            match status {
                0 => Ok(None),
                1 => {
                    // SAFETY: the packet is the encoder's until dimos_av1_release, called right after the copy.
                    let unit = unsafe { std::slice::from_raw_parts(data, len) }.to_vec();
                    // SAFETY: raw is live and holds exactly the packet just read.
                    unsafe { ffi::dimos_av1_release(self.raw) };
                    Ok(Some((unit, keyframe != 0, pts)))
                }
                _ => Err(error_text(&error)),
            }
        }
        #[cfg(not(svt_av1))]
        Err("this build has no SVT-AV1".into())
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        #[cfg(svt_av1)]
        // SAFETY: raw came from dimos_av1_open and is closed exactly once.
        unsafe {
            ffi::dimos_av1_close(self.raw)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings(width: u32, height: u32) -> Settings {
        Settings {
            width,
            height,
            fps: 30,
            bitrate_bps: 1_000_000,
            gop: 30,
            preset: 13,
            threads: 1,
        }
    }

    /// A moving pattern, so the encoder has something other than a flat field to predict.
    fn frame(width: usize, height: usize, step: usize) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let y = (0..width * height)
            .map(|i| ((i / width + i % width + step) % 255) as u8)
            .collect();
        let chroma = vec![128u8; width.div_ceil(2) * height.div_ceil(2)];
        (y, chroma.clone(), chroma)
    }

    #[test]
    fn a_frame_whose_planes_are_too_small_is_refused_before_the_encoder_sees_it() {
        // No encoder is opened here: the check is on the planes, so it must hold without SVT-AV1 too.
        let (width, height) = (64usize, 64usize);
        let (y, u, v) = frame(width, height, 0);
        let mut encoder = Encoder {
            #[cfg(svt_av1)]
            raw: std::ptr::null_mut(),
            settings: settings(width as u32, height as u32),
        };
        // A y plane one row short.
        let short = &y[..y.len() - width];
        let error = encoder
            .send(short, &u, &v, width as u32, width.div_ceil(2) as u32, 0)
            .expect_err("a short plane must be refused");
        assert!(error.contains("do not hold"), "{error}");
        // A stride narrower than the picture.
        let error = encoder
            .send(&y, &u, &v, (width - 1) as u32, width.div_ceil(2) as u32, 0)
            .expect_err("a narrow stride must be refused");
        assert!(error.contains("too narrow"), "{error}");
        // Nothing was sent, so dropping must not free an encoder that was never opened.
        std::mem::forget(encoder);
    }

    #[test]
    fn a_picture_below_svt_av1s_minimum_and_an_all_intra_gop_are_refused_with_a_sentence() {
        let mut small = settings(64, 48);
        let error = check(&small).expect_err("48 rows is under the minimum");
        assert!(error.contains("at least 64"), "{error}");
        small = settings(64, 64);
        small.gop = 1;
        let error = check(&small).expect_err("all-intra at a fixed bitrate");
        assert!(error.contains("all-intra"), "{error}");
    }

    #[cfg(svt_av1)]
    #[test]
    fn the_first_unit_is_a_keyframe_and_carries_a_sequence_header() {
        let (width, height) = (64usize, 64usize);
        let mut encoder = Encoder::open(settings(width as u32, height as u32)).expect("open");
        let header = encoder.stream_header();
        // An AV1 OBU's first byte has type in bits 3..6; 1 is OBU_SEQUENCE_HEADER.
        assert!(!header.is_empty(), "the stream header must not be empty");
        assert_eq!(
            (header[0] >> 3) & 0xf,
            1,
            "first OBU is the sequence header"
        );

        let (y, u, v) = frame(width, height, 0);
        encoder
            .send(&y, &u, &v, width as u32, width.div_ceil(2) as u32, 0)
            .expect("send");
        let (unit, keyframe, pts) = encoder
            .next_unit()
            .expect("get")
            .expect("low delay returns the frame just sent");
        assert!(keyframe, "the first frame is a keyframe");
        assert_eq!(pts, 0);
        assert!(!unit.is_empty());
    }

    #[cfg(svt_av1)]
    #[test]
    fn a_run_of_frames_comes_back_in_order_with_nothing_reordered() {
        // Low delay is one in, one out: `get_packet` blocks until the frame just sent is encoded.
        let (width, height) = (64usize, 64usize);
        let count = 16i64;
        let mut encoder = Encoder::open(settings(width as u32, height as u32)).expect("open");
        let mut stamps = Vec::new();
        for step in 0..count {
            let (y, u, v) = frame(width, height, step as usize * 3);
            encoder
                .send(&y, &u, &v, width as u32, width.div_ceil(2) as u32, step)
                .expect("send");
            // One get per send: in low delay get_packet blocks until this frame's unit is ready, so asking
            // twice would wait for a frame that has not been sent.
            if let Some((unit, _, pts)) = encoder.next_unit().expect("get") {
                assert!(!unit.is_empty(), "frame {pts} encoded to nothing");
                stamps.push(pts);
            }
        }
        assert!(
            stamps.len() as i64 == count,
            "low delay returns one unit per frame: {} for {count}",
            stamps.len()
        );
        assert!(
            stamps.windows(2).all(|pair| pair[0] < pair[1]),
            "low delay must not reorder: {stamps:?}"
        );
        assert_eq!(stamps[0], 0, "the first frame out is the first frame in");
    }
}
