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

//! The policies through ONNX Runtime's C API, loaded at run time (dlopen), configured as rl_deploy configures its
//! sessions (PolicyRunnerBase::PolicyRunnerBase): one intra-op thread, graph optimization level 99 (all). With the
//! library rl_deploy ships (AOS /opt/robot/share/rl_deploy/lib/onnxruntime/libonnxruntime.so.1.20.0) the network
//! outputs are rl_deploy's to the bit; tract's differ by up to ~5e-5. Nothing to link or ship: the library is the
//! board's own.

use std::ffi::{c_char, c_void, CStr, CString};
use std::path::Path;
use std::sync::{Mutex, OnceLock};

use anyhow::{anyhow, bail, Context, Result};

/// The C API version asked for (ORT 1.20's; any newer library serves it).
const API_VERSION: u32 = 20;

// OrtApi function table slots (onnxruntime_c_api.h order; the ones rl_deploy calls checked against its binary:
// CreateEnv 3, SetSessionGraphOptimizationLevel 23, SetIntraOpNumThreads 24, CreateCpuMemoryInfo 69, Release* 92-100)
const GET_ERROR_MESSAGE: usize = 2;
const CREATE_ENV: usize = 3;
const CREATE_SESSION: usize = 7;
const RUN: usize = 9;
const CREATE_SESSION_OPTIONS: usize = 10;
const SET_GRAPH_OPTIMIZATION_LEVEL: usize = 23;
const SET_INTRA_OP_NUM_THREADS: usize = 24;
const SESSION_GET_INPUT_COUNT: usize = 30;
const SESSION_GET_OUTPUT_COUNT: usize = 31;
const SESSION_GET_INPUT_NAME: usize = 36;
const SESSION_GET_OUTPUT_NAME: usize = 37;
const CREATE_TENSOR_WITH_DATA: usize = 49;
const GET_TENSOR_MUTABLE_DATA: usize = 51;
const GET_TENSOR_SHAPE_ELEMENT_COUNT: usize = 64;
const GET_TENSOR_TYPE_AND_SHAPE: usize = 65;
const CREATE_CPU_MEMORY_INFO: usize = 69;
const ALLOCATOR_FREE: usize = 76;
const GET_ALLOCATOR_WITH_DEFAULT_OPTIONS: usize = 78;
const RELEASE_STATUS: usize = 93;
const RELEASE_MEMORY_INFO: usize = 94;
const RELEASE_SESSION: usize = 95;
const RELEASE_VALUE: usize = 96;
const RELEASE_TENSOR_TYPE_AND_SHAPE_INFO: usize = 99;
const RELEASE_SESSION_OPTIONS: usize = 100;

const ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: i32 = 1;
const ORT_LOGGING_LEVEL_WARNING: i32 = 2;
const ORT_ENABLE_ALL: i32 = 99;
const ORT_ARENA_ALLOCATOR: i32 = 1;
const ORT_MEM_TYPE_DEFAULT: i32 = 0;

type Status = *mut c_void;

#[repr(C)]
struct ApiBase {
    get_api: unsafe extern "C" fn(u32) -> *const *const c_void,
    get_version_string: unsafe extern "C" fn() -> *const c_char,
}

/// One loaded library: its function table and an environment (process-wide, as ONNX Runtime wants).
struct Library {
    api: *const *const c_void,
    env: *mut c_void,
    version: String,
}

unsafe impl Send for Library {}
unsafe impl Sync for Library {}

static LIBRARIES: OnceLock<Mutex<Vec<(String, &'static Library)>>> = OnceLock::new();

impl Library {
    fn load(path: &str) -> Result<&'static Library> {
        let libs = LIBRARIES.get_or_init(|| Mutex::new(Vec::new()));
        let mut libs = libs
            .lock()
            .map_err(|_| anyhow!("onnxruntime library table poisoned"))?;
        if let Some((_, l)) = libs.iter().find(|(p, _)| p == path) {
            return Ok(l);
        }
        let c = CString::new(path)?;
        // SAFETY: dlopen / dlsym on a path the operator configured; the symbol is ONNX Runtime's documented entry
        let base = unsafe {
            let h = libc::dlopen(c.as_ptr(), libc::RTLD_NOW | libc::RTLD_LOCAL);
            if h.is_null() {
                let e = CStr::from_ptr(libc::dlerror())
                    .to_string_lossy()
                    .into_owned();
                bail!("cannot load {path}: {e}");
            }
            let sym = libc::dlsym(h, c"OrtGetApiBase".as_ptr());
            if sym.is_null() {
                bail!("{path} has no OrtGetApiBase");
            }
            let f: unsafe extern "C" fn() -> *const ApiBase = std::mem::transmute(sym);
            &*f()
        };
        // SAFETY: the API base's functions, as documented
        let (api, version) = unsafe {
            let version = CStr::from_ptr((base.get_version_string)())
                .to_string_lossy()
                .into_owned();
            ((base.get_api)(API_VERSION), version)
        };
        if api.is_null() {
            bail!("{path} (onnxruntime {version}) does not serve C API version {API_VERSION}");
        }
        let mut lib = Library {
            api,
            env: std::ptr::null_mut(),
            version,
        };
        let id = c"m20_controller";
        let mut env = std::ptr::null_mut();
        // SAFETY: CreateEnv(level, id, out)
        lib.check(unsafe {
            lib.f::<unsafe extern "C" fn(i32, *const c_char, *mut *mut c_void) -> Status>(
                CREATE_ENV,
            )(ORT_LOGGING_LEVEL_WARNING, id.as_ptr(), &mut env)
        })?;
        lib.env = env;
        let lib: &'static Library = Box::leak(Box::new(lib));
        libs.push((path.to_string(), lib));
        Ok(lib)
    }

    /// The function in slot `i` as type F.
    unsafe fn f<F: Copy>(&self, i: usize) -> F {
        let p = *self.api.add(i);
        std::mem::transmute_copy(&p)
    }

    fn check(&self, status: Status) -> Result<()> {
        if status.is_null() {
            return Ok(());
        }
        // SAFETY: GetErrorMessage / ReleaseStatus on the status returned
        unsafe {
            let msg = CStr::from_ptr(self
                .f::<unsafe extern "C" fn(Status) -> *const c_char>(GET_ERROR_MESSAGE)(
                status
            ))
            .to_string_lossy()
            .into_owned();
            self.f::<unsafe extern "C" fn(Status)>(RELEASE_STATUS)(status);
            bail!("onnxruntime: {msg}")
        }
    }
}

/// One model's session.
pub struct Session {
    lib: &'static Library,
    session: *mut c_void,
    memory: *mut c_void,
    inputs: Vec<CString>,
    outputs: Vec<CString>,
}

// the session is used by one thread at a time (the policy thread owns its runners)
unsafe impl Send for Session {}

impl Session {
    pub fn new(library: &str, model: &Path) -> Result<Session> {
        let lib = Library::load(library)?;
        let path = CString::new(model.to_str().context("model path is not UTF-8")?)?;
        let mut opts = std::ptr::null_mut();
        let mut session = std::ptr::null_mut();
        let mut memory = std::ptr::null_mut();
        // SAFETY: the C API's session setup, each status checked; options released once the session holds them
        unsafe {
            lib.check(lib.f::<unsafe extern "C" fn(*mut *mut c_void) -> Status>(
                CREATE_SESSION_OPTIONS,
            )(&mut opts))?;
            let configured = lib
                .check(lib.f::<unsafe extern "C" fn(*mut c_void, i32) -> Status>(
                    SET_INTRA_OP_NUM_THREADS,
                )(opts, 1))
                .and_then(|_| {
                    lib.check(lib.f::<unsafe extern "C" fn(*mut c_void, i32) -> Status>(
                        SET_GRAPH_OPTIMIZATION_LEVEL,
                    )(opts, ORT_ENABLE_ALL))
                })
                .and_then(|_| {
                    lib.check(lib.f::<unsafe extern "C" fn(
                        *mut c_void,
                        *const c_char,
                        *mut c_void,
                        *mut *mut c_void,
                    ) -> Status>(CREATE_SESSION)(
                        lib.env,
                        path.as_ptr(),
                        opts,
                        &mut session,
                    ))
                });
            lib.f::<unsafe extern "C" fn(*mut c_void)>(RELEASE_SESSION_OPTIONS)(opts);
            configured.with_context(|| format!("loading {}", model.display()))?;
            lib.check(lib.f::<unsafe extern "C" fn(
                i32,
                i32,
                *mut *mut c_void,
            ) -> Status>(CREATE_CPU_MEMORY_INFO)(
                ORT_ARENA_ALLOCATOR,
                ORT_MEM_TYPE_DEFAULT,
                &mut memory,
            ))?;
        }
        let mut s = Session {
            lib,
            session,
            memory,
            inputs: Vec::new(),
            outputs: Vec::new(),
        };
        s.inputs = s.names(SESSION_GET_INPUT_COUNT, SESSION_GET_INPUT_NAME)?;
        s.outputs = s.names(SESSION_GET_OUTPUT_COUNT, SESSION_GET_OUTPUT_NAME)?;
        Ok(s)
    }

    pub fn version(&self) -> &str {
        &self.lib.version
    }

    fn names(&self, count_fn: usize, name_fn: usize) -> Result<Vec<CString>> {
        let lib = self.lib;
        let mut n = 0usize;
        let mut alloc = std::ptr::null_mut();
        // SAFETY: the name queries, each name copied and freed with the default allocator
        unsafe {
            lib.check(lib
                .f::<unsafe extern "C" fn(*mut c_void, *mut usize) -> Status>(
                    count_fn,
                )(self.session, &mut n))?;
            lib.check(lib.f::<unsafe extern "C" fn(*mut *mut c_void) -> Status>(
                GET_ALLOCATOR_WITH_DEFAULT_OPTIONS,
            )(&mut alloc))?;
            (0..n)
                .map(|i| {
                    let mut p: *mut c_char = std::ptr::null_mut();
                    lib.check(lib.f::<unsafe extern "C" fn(
                        *mut c_void,
                        usize,
                        *mut c_void,
                        *mut *mut c_char,
                    ) -> Status>(name_fn)(
                        self.session, i, alloc, &mut p
                    ))?;
                    let name = CStr::from_ptr(p).to_owned();
                    lib.check(lib
                        .f::<unsafe extern "C" fn(*mut c_void, *mut c_void) -> Status>(
                            ALLOCATOR_FREE,
                        )(alloc, p as *mut c_void))?;
                    Ok(name)
                })
                .collect()
        }
    }

    /// Run on f32 inputs (data, shape) in the model's input order; every output's values.
    pub fn run(&self, inputs: &mut [(Vec<f32>, Vec<i64>)]) -> Result<Vec<Vec<f32>>> {
        let lib = self.lib;
        if inputs.len() != self.inputs.len() {
            bail!(
                "the model takes {} inputs, got {}",
                self.inputs.len(),
                inputs.len()
            );
        }
        let mut values: Vec<*mut c_void> = Vec::with_capacity(inputs.len());
        let mut outputs: Vec<*mut c_void> = vec![std::ptr::null_mut(); self.outputs.len()];
        // SAFETY: tensors over our buffers (alive for the call), the run, then every value released
        let result = unsafe {
            let made: Result<()> = inputs.iter_mut().try_for_each(|(data, shape)| {
                let mut v = std::ptr::null_mut();
                lib.check(lib.f::<unsafe extern "C" fn(
                    *mut c_void,
                    *mut c_void,
                    usize,
                    *const i64,
                    usize,
                    i32,
                    *mut *mut c_void,
                ) -> Status>(CREATE_TENSOR_WITH_DATA)(
                    self.memory,
                    data.as_mut_ptr() as *mut c_void,
                    data.len() * 4,
                    shape.as_ptr(),
                    shape.len(),
                    ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                    &mut v,
                ))?;
                values.push(v);
                Ok(())
            });
            let ran = made.and_then(|_| {
                let in_names: Vec<*const c_char> = self.inputs.iter().map(|n| n.as_ptr()).collect();
                let out_names: Vec<*const c_char> =
                    self.outputs.iter().map(|n| n.as_ptr()).collect();
                lib.check(lib.f::<unsafe extern "C" fn(
                    *mut c_void,
                    *const c_void,
                    *const *const c_char,
                    *const *const c_void,
                    usize,
                    *const *const c_char,
                    usize,
                    *mut *mut c_void,
                ) -> Status>(RUN)(
                    self.session,
                    std::ptr::null(),
                    in_names.as_ptr(),
                    values.as_ptr() as *const *const c_void,
                    values.len(),
                    out_names.as_ptr(),
                    out_names.len(),
                    outputs.as_mut_ptr(),
                ))
            });
            let read = ran.and_then(|_| outputs.iter().map(|&v| self.read(v)).collect());
            for v in values.iter().chain(outputs.iter()).filter(|v| !v.is_null()) {
                lib.f::<unsafe extern "C" fn(*mut c_void)>(RELEASE_VALUE)(*v);
            }
            read
        };
        result
    }

    unsafe fn read(&self, v: *mut c_void) -> Result<Vec<f32>> {
        let lib = self.lib;
        let mut info = std::ptr::null_mut();
        let mut n = 0usize;
        let mut data: *mut c_void = std::ptr::null_mut();
        lib.check(lib
            .f::<unsafe extern "C" fn(*mut c_void, *mut *mut c_void) -> Status>(
                GET_TENSOR_TYPE_AND_SHAPE,
            )(v, &mut info))?;
        let counted = lib.check(lib
            .f::<unsafe extern "C" fn(*mut c_void, *mut usize) -> Status>(
                GET_TENSOR_SHAPE_ELEMENT_COUNT,
            )(info, &mut n));
        lib.f::<unsafe extern "C" fn(*mut c_void)>(RELEASE_TENSOR_TYPE_AND_SHAPE_INFO)(info);
        counted?;
        lib.check(lib
            .f::<unsafe extern "C" fn(*mut c_void, *mut *mut c_void) -> Status>(
                GET_TENSOR_MUTABLE_DATA,
            )(v, &mut data))?;
        Ok(std::slice::from_raw_parts(data as *const f32, n).to_vec())
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        // SAFETY: releasing what new() created
        unsafe {
            self.lib
                .f::<unsafe extern "C" fn(*mut c_void)>(RELEASE_SESSION)(self.session);
            self.lib
                .f::<unsafe extern "C" fn(*mut c_void)>(RELEASE_MEMORY_INFO)(
                self.memory
            );
        }
    }
}
