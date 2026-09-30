// Sojan James
// build.rs for cyclonedds-sys

/*
    Copyright 2020 Sojan James

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

use cc;
use std::env;
use std::process::Command;

fn main() {
    // Don't try to re-generate in docs.rs' build runner.
    if let Ok(val) = env::var("DOCS_RS") {
        if val == "1" {
            return;
        }
    }
    build::main();
}

macro_rules! ok(($expression:expr) => ($expression.unwrap()));
macro_rules! log {
    ($fmt:expr) => (println!(concat!("cyclonedds-sys/build.rs:{}: ", $fmt), line!()));
    ($fmt:expr, $($arg:tt)*) => (println!(concat!("cyclonedds-sys/build.rs:{}: ", $fmt),
    line!(), $($arg)*));
}

fn run<F>(name: &str, mut configure: F)
where
    F: FnMut(&mut Command) -> &mut Command,
{
    let mut command = Command::new(name);
    let configured = configure(&mut command);
    log!("Executing {:?}", configured);
    if !ok!(configured.status()).success() {
        panic!("failed to execute {:?}", configured);
    }
    log!("Command {:?} finished successfully", configured);
}

mod build {

    extern crate bindgen;

    use std::path::Path;
    use std::path::PathBuf;
    //use walkdir::{DirEntry, WalkDir};
    use super::*;
    use glob::glob;

    static ENV_PREFIX: &str = "CYCLONEDDS";
    static LINKLIB: &str = "ddsc";
    static GIT_COMMIT: &str = "9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd";

    pub enum HeaderLocation {
        FromCMakeEnvironment(std::vec::Vec<String>, String),
        FromYoctoSDKBuild(std::vec::Vec<String>, String),
        FromEnvironment(std::vec::Vec<String>),
        FromLocalBuild(std::vec::Vec<String>),
    }

    impl HeaderLocation {
        fn add_paths(&mut self, mut path: Vec<String>) {
            match self {
                HeaderLocation::FromCMakeEnvironment(paths, _) => paths.append(&mut path),
                HeaderLocation::FromYoctoSDKBuild(paths, _) => paths.append(&mut path),
                HeaderLocation::FromEnvironment(paths) => paths.append(&mut path),
                HeaderLocation::FromLocalBuild(paths) => paths.append(&mut path),
            }
        }

        fn get_paths(&self) -> Vec<String> {
            match self {
                HeaderLocation::FromCMakeEnvironment(paths, _)
                | HeaderLocation::FromYoctoSDKBuild(paths, _)
                | HeaderLocation::FromEnvironment(paths)
                | HeaderLocation::FromLocalBuild(paths) => paths.clone(),
            }
        }
    }

    /// download cyclone dds from github
    fn download() {
        // get head of master for now. We can change to a specific version when
        // needed

        let outdir = env::var("OUT_DIR").expect("OUT_DIR is not set");
        let srcpath = format!("{}/cyclonedds", &outdir);
        let cyclonedds_src_path = Path::new(srcpath.as_str());

        if !cyclonedds_src_path.exists() {
            log!("Cloning cyclonedds from github");
            run("git", |command| {
                command
                    .arg("clone")
                    .arg("https://github.com/eclipse-cyclonedds/cyclonedds.git")
                    .current_dir(env::var("OUT_DIR").expect("OUT_DIR is not set").as_str())
            });
        }
        log!("running git checkout to get the right version of cyclonedds");
        run("git", |command| {
            command
                .arg("checkout")
                .arg(GIT_COMMIT)
                .current_dir(cyclonedds_src_path.to_str().unwrap())
        });
    }

    fn configure_and_build() {
        let outdir = env::var("OUT_DIR").expect("OUT_DIR is not set");
        let srcpath = format!("{}/cyclonedds", &outdir);
        let cyclonedds_src_path = Path::new(srcpath.as_str());

        run("mkdir", |command| {
            command
                .arg("-p")
                .arg("build")
                .current_dir(cyclonedds_src_path.to_str().unwrap())
        });

        run("cmake", |command| {
            command
                .env("CFLAGS", "-w")
                .arg("-DWERROR=OFF")
                .arg("-DBUILD_IDLC=OFF")
                .arg("-DBUILD_DDSPERF=OFF")
                .arg("-DBUILD_TESTING=OFF")
                .arg("-DBUILD_DDSPERF=OFF")
                .arg("-DENABLE_TYPE_DISCOVERY=YES")
                .arg("-DENABLE_TOPIC_DISCOVERY=YES")
                .arg(format!("-DCMAKE_INSTALL_PREFIX={}/install", outdir))
                .arg("..")
                .current_dir(format!("{}/build", cyclonedds_src_path.to_str().unwrap()));

            #[cfg(feature = "shm")]
            command.arg("-DENABLE_SHM=YES");

            #[cfg(not(feature = "shm"))]
            command.arg("-DENABLE_SHM=NO");

            command
        });

        run("make", |command| {
            command
                .env("MAKEFLAGS", env::var("CARGO_MAKEFLAGS").unwrap())
                .current_dir(format!("{}/build", cyclonedds_src_path.to_str().unwrap()))
        });

        run("make", |command| {
            command
                .arg("install")
                .current_dir(format!("{}/build", cyclonedds_src_path.to_str().unwrap()))
        });

        println!("cargo:rustc-link-search=native={}", outdir);
        println!("cargo:rustc-link-lib=dylib=ddsc");
        //cargo:rustc-link-lib=LIB
    }

    fn find_iceoryx(iceoryx_version: &str) -> Option<HeaderLocation> {
        // Check if we are building with an OE SDK and the OECORE_TARGET_SYSROOT is set
        let iceoryx_header_path = format!(
            "usr/include/iceoryx/{}/iceoryx_binding_c/api.h",
            iceoryx_version
        );
        if let Ok(sysroot) = env::var("OECORE_TARGET_SYSROOT") {
            let header = PathBuf::from(&sysroot).join(&iceoryx_header_path);
            if header.exists() {
                let iceoryx_include_path =
                    header.parent().unwrap().parent().unwrap().to_str().unwrap();
                let paths = vec![iceoryx_include_path.into()];
                //println!("cargo:warning=Found Iceoryx headers in OECORE_TARGET_SYSROOT");

                return Some(HeaderLocation::FromYoctoSDKBuild(paths, sysroot));
            }
        }

        // now look in local paths - nothing fancy here for now, just using the paths where iceoryx gets installed on my Ubuntu machine.
        let iceoryx_header_path = format!(
            "/usr/local/include/iceoryx/{}/iceoryx_binding_c/api.h",
            iceoryx_version
        );
        let header = PathBuf::from(&iceoryx_header_path);
        if header.exists() {
            //println!("cargo:warning=Found Iceoryx headers in {}",iceoryx_header_path);
            let iceoryx_include_path = header.parent().unwrap().parent().unwrap().to_str().unwrap();
            return Some(HeaderLocation::FromLocalBuild(vec![
                iceoryx_include_path.into()
            ]));
        }

        println!("cargo:warning=Iceoryx headers not found");
        None
    }

    fn find_cyclonedds() -> Option<HeaderLocation> {
        // The library name does not change. Print that out right away.
        // CYCLONEDDS_STATIC=1 forces static linking (re-added local patch for the
        // fully-static aarch64-musl build that ships one self-contained binary).
        if env::var("CYCLONEDDS_STATIC").is_ok() {
            println!("cargo:rustc-link-lib=static={}", LINKLIB);
        } else {
            println!("cargo:rustc-link-lib={}", LINKLIB);
        }

        let outdir = env::var("OUT_DIR").expect("OUT_DIR is not set");

        // Check if we are building with an OE SDK and the OECORE_TARGET_SYSROOT is set
        if let Ok(sysroot) = env::var("OECORE_TARGET_SYSROOT") {
            let header = PathBuf::from(&sysroot).join("usr/include/dds/dds.h");
            if header.exists() {
                let paths = vec![sysroot.clone()];
                println!("Found OECORE_TARGET_SYSROOT");
                return Some(HeaderLocation::FromYoctoSDKBuild(paths, sysroot));
            }
        }

        //first priority is environment variable.
        if let Ok(dir) = env::var(format!("{}_LIB_DIR", ENV_PREFIX)) {
            println!("cargo:rustc-link-search={}", dir);

            // Now find the include path
            if let Ok(dir) = env::var(format!("{}_INCLUDE_DIR", ENV_PREFIX)) {
                let path = format!("{}/dds/dds.h", &dir);
                let path = Path::new(&path);
                if path.exists() {
                    println!("Found {}", &path.to_str().unwrap());
                    let paths = vec![dir];
                    Some(HeaderLocation::FromEnvironment(paths))
                } else {
                    println!("Cannot find dds/dds.h");
                    None
                }
            } else {
                println!("LIB_DIR set but INCLUDE_DIR is unset");
                None
            }
        }
        // now check if building using CMAKE. CycloneDDS has a cmake
        // build environment. When building within CMake, the cyclonedds need not
        // be "installed", so multiple include paths are required.
        else if let Ok(dir) = env::var("CMAKE_BINARY_DIR") {
            let cmake_bin_dir = &dir;
            let lib_dir = Path::new(&dir).join("lib");
            println!("cargo:rustc-link-search={:}", lib_dir.display());

            if let Ok(dir) = env::var("CMAKE_SOURCE_DIR") {
                println!(
                    "CMAKE_SOURCE_DIR is set to {}, searching for include path",
                    &dir
                );
                let cmake_src_dir = Path::new(&dir);
                let glob_pattern = format!("{}/**/dds/dds.h", cmake_src_dir.display());
                println!("Glob pattern: {}", &glob_pattern);
                let mut paths = std::vec::Vec::new();
                for entry in glob(&glob_pattern).expect("Glob pattern error") {
                    match entry {
                        Ok(path) => {
                            println!("{:?}", path.display());
                            let cyclone_src = path
                                .to_str()
                                .unwrap()
                                .split("cyclonedds")
                                .collect::<Vec<&str>>();
                            let mut cyclone_src = String::from(cyclone_src[0]);
                            cyclone_src.push_str("cyclonedds");

                            paths.push(format!("{}/src/core/ddsc/include", cyclone_src));
                            paths.push(format!("{}/src/core/include", cyclone_src));

                            //
                            paths.push(format!(
                                "{}/src/core/include",
                                find_cyclone_bin_dir(cmake_bin_dir).unwrap()
                            ));

                            println!("{:?}", paths);
                            break;
                        }
                        Err(e) => println!("{:?}", e),
                    }
                }
                // now get the sysroot
                if let Ok(toolchain_sysroot) = env::var("TOOLCHAIN_SYSROOT") {
                    Some(HeaderLocation::FromCMakeEnvironment(
                        paths,
                        toolchain_sysroot,
                    ))
                } else {
                    println!("Unable to get TOOLCHAIN_SYSROOT");
                    Some(HeaderLocation::FromCMakeEnvironment(paths, "/".to_string()))
                }
            } else {
                None
            }
        } else {
            println!("No CMAKE environment or CYCLONEDDS_[LIB|INCLUDE]_DIR found");
            //try some defaults
            println!("cargo:rustc-link-search=/usr/local/lib");

            let path = format!("{}/dds/dds.h", "/usr/local/include");
            let path = Path::new(&path);
            if path.exists() {
                println!("Found {}", &path.to_str().unwrap());
                let paths = vec![String::from("/usr/local/include")];
                Some(HeaderLocation::FromEnvironment(paths))
            } else {
                println!("Cannot find dds/dds.h attempting to build");
                download();
                configure_and_build();
                let local_build_libpath = format!("{}/install/lib/libddsc.so", &outdir);
                let local_build_so = Path::new(local_build_libpath.as_str());

                if local_build_so.exists() {
                    println!("cargo:rustc-link-search={}/install/lib", &outdir);
                    let include_dir = String::from(format!("{}/install/include", &outdir));
                    let path = format!("{}/dds/dds.h", &include_dir);
                    let path = Path::new(&path);

                    if path.exists() {
                        println!("Found {}", &path.to_str().unwrap());
                        let paths = vec![include_dir];
                        Some(HeaderLocation::FromLocalBuild(paths))
                    } else {
                        println!("Cannot find dds/dds.h");
                        None
                    }
                } else {
                    None
                }
            }
        }
    }

    fn find_cyclone_bin_dir(cmake_bin_dir: &str) -> Option<String> {
        Some(format!(
            "{}/sys/cyclonedds/src/ddsrt/include",
            cmake_bin_dir
        ))
    }

    fn add_whitelist(builder: bindgen::Builder) -> bindgen::Builder {
        builder
        .allowlist_function("dds_enable")
        .allowlist_function("dds_delete")
        .allowlist_function("dds_get_publisher")
        .allowlist_function("dds_get_subscriber")
        .allowlist_function("dds_get_datareader")
        .allowlist_function("dds_get_mask")
        .allowlist_function("dds_get_instance_handle")
        .allowlist_function("dds_read_status")
        .allowlist_function("dds_take_status")
        .allowlist_function("dds_get_status_changes")
        .allowlist_function("dds_get_status_mask")
        .allowlist_function("dds_get_enabled_status")
        .allowlist_function("dds_set_status_mask")
        .allowlist_function("dds_set_enabled_status")
        .allowlist_function("dds_get_qos")
        .allowlist_function("dds_set_qos")
        .allowlist_function("dds_get_listener")
        .allowlist_function("dds_set_listener")
        .allowlist_function("dds_read_status")
        .allowlist_function("dds_create_participant")
        .allowlist_function("dds_create_domain")
        .allowlist_function("dds_get_parent")
        .allowlist_function("dds_get_participant")
        .allowlist_function("dds_get_children")
        .allowlist_function("dds_get_domainid")
        .allowlist_function("dds_lookup_participant")
        .allowlist_function("dds_create_topic")
        .allowlist_function("dds_create_topic_arbitrary")
        .allowlist_function("dds_find_topic")
        .allowlist_function("dds_get_name")
        .allowlist_function("dds_get_type_name")
        .allowlist_function("dds_set_topic_filter")
        .allowlist_function("dds_get_topic_filter")
        .allowlist_function("dds_create_subscriber")
        .allowlist_function("dds_create_publisher")
        .allowlist_function("dds_suspend")
        .allowlist_function("dds_resume")
        .allowlist_function("dds_wait_for_acks")
        .allowlist_function("dds_create_reader")
        .allowlist_function("dds_create_reader_rhc")
        .allowlist_function("dds_reader_wait_for_historical_data")
        .allowlist_function("dds_create_writer")
        .allowlist_function("dds_register_instance")
        .allowlist_function("dds_unregister_instance")
        .allowlist_function("dds_unregister_instance_ih")
        .allowlist_function("dds_unregister_instance_ts")
        .allowlist_function("dds_unregister_instance_ih_ts")
        .allowlist_function("dds_writedispose")
        .allowlist_function("dds_writedispose_ts")
        .allowlist_function("dds_dispose")
        .allowlist_function("dds_dispose_ts")
        .allowlist_function("dds_dispose_ih")
        .allowlist_function("dds_dispose_ih_ts")
        .allowlist_function("dds_write")
        .allowlist_function("dds_write_flush")
        .allowlist_function("dds_writecdr")
        .allowlist_function("dds_write_ts")
        .allowlist_function("dds_create_readcondition")
        .allowlist_function("dds_create_querycondition")
        .allowlist_function("dds_create_guardcondition")
        .allowlist_function("dds_set_guardcondition")
        .allowlist_function("dds_read_guardcondition")
        .allowlist_function("dds_take_guardcondition")
        .allowlist_function("dds_create_waitset")
        .allowlist_function("dds_waitset_get_entities")
        .allowlist_function("dds_waitset_attach")
        .allowlist_function("dds_waitset_detach")
        .allowlist_function("dds_waitset_set_trigger")
        .allowlist_function("dds_waitset_wait")
        .allowlist_function("dds_waitset_wait_until")
        .allowlist_function("dds_read")
        .allowlist_function("dds_read_wl")
        .allowlist_function("dds_read_mask")
        .allowlist_function("dds_read_mask_wl")
        .allowlist_function("dds_read_instance")
        .allowlist_function("dds_read_instance_wl")
        .allowlist_function("dds_read_mask_wl")
        .allowlist_function("dds_read_instance_mask")
        .allowlist_function("dds_read_instance_mask_wl")
        .allowlist_function("dds_take")
        .allowlist_function("dds_take_wl")
        .allowlist_function("dds_take_mask")
        .allowlist_function("dds_take_mask_wl")
        .allowlist_function("dds_take_cdr")
        .allowlist_function("dds_take_instance")
        .allowlist_function("dds_take_instance_wl")
        .allowlist_function("dds_take_instance_mask")
        .allowlist_function("dds_take_instance_mask_wl")
        .allowlist_function("dds_take_next")
        .allowlist_function("dds_take_next_wl")
        .allowlist_function("dds_read_next")
        .allowlist_function("dds_read_next_wl")
        .allowlist_function("dds_return_loan")
        .allowlist_function("dds_lookup_instance")
        .allowlist_function("dds_instance_get_key")
        .allowlist_function("dds_begin_coherent")
        .allowlist_function("dds_end_coherent")
        .allowlist_function("dds_notify_readers")
        .allowlist_function("dds_triggered")
        .allowlist_function("dds_get_topic")
        .allowlist_function("dds_get_matched_subscriptions")
        .allowlist_function("dds_get_matched_subscription_data")
        .allowlist_function("dds_get_matched_publications")
        .allowlist_function("dds_get_matched_publication_data")
        .allowlist_function("dds_assert_liveliness")   /* DDS Public Listener API Follows */
        .allowlist_function("dds_create_listener")
        .allowlist_function("dds_delete_listener")
        .allowlist_function("dds_reset_listener")
        .allowlist_function("dds_copy_listener")
        .allowlist_function("dds_merge_listener")
        .allowlist_function("dds_lset_inconsistent_topic")
        .allowlist_function("dds_lset_liveliness_lost")
        .allowlist_function("dds_lset_offered_deadline_missed")
        .allowlist_function("dds_lset_offered_incompatible_qos")
        .allowlist_function("dds_lset_data_on_readers")
        .allowlist_function("dds_lset_sample_lost")
        .allowlist_function("dds_lset_data_available")
        .allowlist_function("dds_lset_sample_rejected")
        .allowlist_function("dds_lset_liveliness_changed")
        .allowlist_function("dds_lset_requested_deadline_missed")
        .allowlist_function("dds_lset_requested_incompatible_qos")
        .allowlist_function("dds_lset_offered_incompatible_qos")
        .allowlist_function("dds_lset_publication_matched")
        .allowlist_function("dds_lset_subscription_matched")
        .allowlist_function("dds_lget_inconsistent_topic")
        .allowlist_function("dds_lget_liveliness_lost")
        .allowlist_function("dds_lget_offered_deadline_missed")
        .allowlist_function("dds_lget_offered_incompatible_qos")
        .allowlist_function("dds_lget_data_on_readers")
        .allowlist_function("dds_lget_sample_lost")
        .allowlist_function("dds_lget_data_available")
        .allowlist_function("dds_lget_sample_rejected")
        .allowlist_function("dds_lget_liveliness_changed")
        .allowlist_function("dds_lget_requested_deadline_missed")
        .allowlist_function("dds_lget_requested_incompatible_qos")
        .allowlist_function("dds_lget_publication_matched")
        .allowlist_function("dds_lget_subscription_matched")  /* DDS Public Alloc APIs follow */
        .allowlist_function("dds_alloc")
        .allowlist_function("dds_realloc")
        .allowlist_function("dds_realloc_zero")
        .allowlist_function("dds_free")
        .allowlist_function("dds_string_alloc")
        .allowlist_function("dds_string_dup")
        .allowlist_function("dds_string_free")
        .allowlist_function("dds_sample_free")   /* DDS Public Status APIs follow */
        .allowlist_function("dds_get_inconsistent_topic_status")
        .allowlist_function("dds_get_publication_matched_status")
        .allowlist_function("dds_get_liveliness_lost_status")
        .allowlist_function("dds_get_offered_deadline_missed_status")
        .allowlist_function("dds_get_inconsistent_topic_status")
        .allowlist_function("dds_get_offered_incompatible_qos_status")
        .allowlist_function("dds_get_subscription_matched_status")
        .allowlist_function("dds_get_liveliness_changed_status")
        .allowlist_function("dds_get_sample_rejected_status")
        .allowlist_function("dds_get_sample_lost_status")
        .allowlist_function("dds_get_requested_deadline_missed_status")
        .allowlist_function("dds_get_requested_incompatible_qos_status")
        .allowlist_function("dds_get_inconsistent_topic_status")  /* DDS Public QOS APIs follow */
        .allowlist_function("dds_create_qos")
        .allowlist_function("dds_delete_qos")
        .allowlist_function("dds_reset_qos")
        .allowlist_function("dds_copy_qos")
        .allowlist_function("dds_merge_qos")
        .allowlist_function("dds_qos_equal")
        .allowlist_function("dds_qset_userdata")
        .allowlist_function("dds_qset_topicdata")
        .allowlist_function("dds_qset_groupdata")
        .allowlist_function("dds_qset_durability")
        .allowlist_function("dds_qset_history")
        .allowlist_function("dds_qset_resource_limits")
        .allowlist_function("dds_qset_presentation")
        .allowlist_function("dds_qset_lifespan")
        .allowlist_function("dds_qset_deadline")
        .allowlist_function("dds_qset_latency_budget")
        .allowlist_function("dds_qset_ownership")
        .allowlist_function("dds_qset_ownership_strength")
        .allowlist_function("dds_qset_liveliness")
        .allowlist_function("dds_qset_time_based_filter")
        .allowlist_function("dds_qset_partition")
        .allowlist_function("dds_qset_partition1")
        .allowlist_function("dds_qset_reliability")
        .allowlist_function("dds_qset_transport_priority")
        .allowlist_function("dds_qset_destination_order")
        .allowlist_function("dds_qset_writer_data_lifecycle")
        .allowlist_function("dds_qset_reader_data_lifecycle")
        .allowlist_function("dds_qset_durability_service")
        .allowlist_function("dds_qset_ignorelocal")
        .allowlist_function("dds_qget_userdata")
        .allowlist_function("dds_qget_topicdata")
        .allowlist_function("dds_qget_groupdata")
        .allowlist_function("dds_qget_durability")
        .allowlist_function("dds_qget_history")
        .allowlist_function("dds_qget_resource_limits")
        .allowlist_function("dds_qget_presentation")
        .allowlist_function("dds_qget_lifespan")
        .allowlist_function("dds_qget_deadline")
        .allowlist_function("dds_qget_latency_budget")
        .allowlist_function("dds_qget_ownership")
        .allowlist_function("dds_qget_ownership_strength")
        .allowlist_function("dds_qget_liveliness")
        .allowlist_function("dds_qget_time_based_filter")
        .allowlist_function("dds_qget_partition")
        .allowlist_function("dds_qget_reliability")
        .allowlist_function("dds_qget_transport_priority")
        .allowlist_function("dds_qget_destination_order")
        .allowlist_function("dds_qget_writer_data_lifecycle")
        .allowlist_function("dds_qget_reader_data_lifecyele")
        .allowlist_function("dds_qget_durability_service")
        .allowlist_function("dds_qget_history")
        .allowlist_function("dds_qget_ignorelocal")
        .allowlist_function("dds_qget_history")
        .allowlist_function("dds_create_topic_sertype")
        .allowlist_function("ddsi_sertype_init")
        .allowlist_function("ddsi_sertype_fini")
        .allowlist_function("ddsi_sertype_v0")
        .allowlist_function("ddsi_serdata_init")
        .allowlist_function("ddsi_serdata_addref")
        .allowlist_function("ddsi_serdata_removeref")
        .allowlist_function("ddsrt_md5_init")
        .allowlist_function("ddsrt_md5_append")
        .allowlist_function("ddsrt_md5_finish")
        .allowlist_function("iceoryx_header_from_chunk")
        .allowlist_function("shm_lock_iox_sub")
        .allowlist_function("shm_unlock_iox_sub")
        .allowlist_function("free_iox_chunk")
        .allowlist_function("shm_set_loglevel")
        .allowlist_function("shm_create_chunk")
        .allowlist_function("shm_set_data_state")
        .allowlist_function("shm_get_data_state")
        .allowlist_function("dds_is_loan_available")
        .allowlist_function("dds_is_shared_memory_available")
        .allowlist_function("dds_loan_shared_memory_buffer")
        .allowlist_function("dds_return_writer_loan")
        .allowlist_function("dds_loan_sample")
        .allowlist_function("_dummy")
        .allowlist_type("dds_stream_opcode")
        .allowlist_type("dds_stream_typecode")
        .allowlist_type("dds_stream_typecode_primary")
        .allowlist_type("dds_stream_typecode_subtype")
        .allowlist_type("dds_sequence_t")
        .allowlist_type("dds_duration_t")
        .allowlist_type("ddsi_sertype_ops")
        .allowlist_type("ddsi_serdata_ops")
        .allowlist_type("ddsi_sertype_init")
        .allowlist_type("nn_rdata")
        .allowlist_function("ddsrt_iovec_t")
        .allowlist_function("ddsrt_md5_state_t")
        .allowlist_var("DDS_DOMAIN_DEFAULT")
        .rustified_enum("dds_durability_kind")
        .rustified_enum("dds_history_kind")
        .rustified_enum("dds_ownership_kind")
        .rustified_enum("dds_liveliness_kind")
        .rustified_enum("dds_reliability_kind")
        .rustified_enum("dds_destination_order_kind")
        .rustified_enum("dds_presentation_access_scope_kind")
        .rustified_enum("dds_ignorelocal_kind")
        .allowlist_var("BUILTIN_TOPIC_DCPSPARTICIPANT")
        .allowlist_var("BUILTIN_TOPIC_DCPSTOPIC")
        .allowlist_var("BUILTIN_TOPIC_DCPSPUBLICATION")
        .allowlist_var("BUILTIN_TOPIC_DCPSSUBSCRIPTION")
 	.derive_default(true)
        .constified_enum("dds_status_id")
    }

    pub fn generate(include_paths: &std::vec::Vec<String>, maybe_sysroot: Option<&String>) {
        // cyclonedds-rs names `size_t` and reads it as u64, as bindgen 0.55 emitted it.
        let mut bindings = bindgen::Builder::default()
            .header("wrapper.h")
            .size_t_is_usize(false);

        for path in include_paths {
            bindings = bindings.clang_arg(format!("-I{}", path));
        }

        if let Some(sysroot) = maybe_sysroot {
            bindings = bindings.clang_arg(format!("--sysroot={}", sysroot));
        }

        let gen = add_whitelist(bindings)
            .generate()
            .expect("Unable to generate bindings");

        if let Ok(path) = env::var("OUT_DIR") {
            let out_path = PathBuf::from(path);
            let bindings_path = out_path.join("generated.rs");
            gen.write_to_file(bindings_path)
                .expect("Couldn't write bindings");
        } else {
            println!("OUT_DIR not set, not generating bindings");
        }
    }

    pub fn main() {
        for (key, value) in env::vars() {
            println!("{}: {}", key, value);
        }
        let mut headerloc = find_cyclonedds().unwrap();

        if let Some(iceoryx_headers) = find_iceoryx("v2.0.2") {
            headerloc.add_paths(iceoryx_headers.get_paths());
        } else if let Some(iceoryx_headers) = find_iceoryx("v2.0.0") {
            headerloc.add_paths(iceoryx_headers.get_paths());
        }

        match &headerloc {
            HeaderLocation::FromCMakeEnvironment(paths, sysroot) => generate(&paths, Some(sysroot)),
            HeaderLocation::FromEnvironment(paths) | HeaderLocation::FromLocalBuild(paths) => {
                generate(&paths, None)
            }
            HeaderLocation::FromYoctoSDKBuild(paths, sysroot) => generate(&paths, Some(sysroot)),
        }

        match &headerloc {
            HeaderLocation::FromCMakeEnvironment(paths, sysroot) => {
                compile_inlines(&paths, Some(sysroot))
            }
            HeaderLocation::FromEnvironment(paths) | HeaderLocation::FromLocalBuild(paths) => {
                compile_inlines(&paths, None)
            }
            HeaderLocation::FromYoctoSDKBuild(paths, sysroot) => {
                compile_inlines(&paths, Some(sysroot))
            }
        }
    }

    fn compile_inlines(include_paths: &Vec<String>, _maybe_sysroot: Option<&String>) {
        let mut cc = cc::Build::new();

        cc.file("inline_functions.c");

        for dir in include_paths {
            cc.include(dir);
        }
        cc.compile("libinline_functions.a");

        //if let Some(sysroot) = maybe_sysroot {
        //    cc.s
        //}
    }
}
