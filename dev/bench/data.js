window.BENCHMARK_DATA = {
  "lastUpdate": 1793408182743,
  "repoUrl": "https://github.com/dimensionalOS/dimos",
  "entries": {
    "go2 replay realtime (arm64)": [
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51e97981bf339c029b8c1e179c845f714782185e",
          "message": "Use Otava for benchmark analysis (#4298)",
          "timestamp": "2026-10-02T16:57:32+01:00",
          "tree_id": "e7189f416bef7dd6b0820649feab088e70b6afb6",
          "url": "https://github.com/dimensionalOS/dimos/commit/51e97981bf339c029b8c1e179c845f714782185e"
        },
        "date": 1790956831103,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.047,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.352,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.259,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d05c743aa98c7ebfec14988c7f0fec216e908215",
          "message": "fix mem concurrency (#4405)",
          "timestamp": "2026-10-02T16:22:10Z",
          "tree_id": "9437fca6ac8ba9597fa9ba6632a322a52d772f65",
          "url": "https://github.com/dimensionalOS/dimos/commit/d05c743aa98c7ebfec14988c7f0fec216e908215"
        },
        "date": 1790958305535,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.203,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.559,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.191,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.326,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe",
          "message": "fix: bump zenoh crate to 1.10.1 (#4386)",
          "timestamp": "2026-10-02T16:53:44Z",
          "tree_id": "b861fc4bb0ea22884dd9d370e7eeb9076e10efa0",
          "url": "https://github.com/dimensionalOS/dimos/commit/d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe"
        },
        "date": 1790960208283,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2000.301,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.952,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.176,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.23,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d97554f54977b80ff0304f4e68a1eb8e5d79d",
          "message": "feat(agent): add planner-based go_to(x,y) skill (#4380)",
          "timestamp": "2026-10-02T10:08:08-07:00",
          "tree_id": "ffafcf0f77777fb795961dd3a4c0201cc30729bc",
          "url": "https://github.com/dimensionalOS/dimos/commit/e06d97554f54977b80ff0304f4e68a1eb8e5d79d"
        },
        "date": 1790961069377,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2025.395,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5677b6d46c1e3c15a445793586afa1da39c88892",
          "message": "feat(control): operator hold task (#4402)",
          "timestamp": "2026-10-02T19:22:48Z",
          "tree_id": "d20d0cf28fb0c1b75ee67b18398a8f1f54b275db",
          "url": "https://github.com/dimensionalOS/dimos/commit/5677b6d46c1e3c15a445793586afa1da39c88892"
        },
        "date": 1790969138260,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2033.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 404,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.586,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.32,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "943ce13c67e679e75452557f9457f1ba58541091",
          "message": "feat(memory): record explicit JSON streams with source timestamps (#4392)",
          "timestamp": "2026-10-02T15:12:04-07:00",
          "tree_id": "5a1d5b3f6095c1b6ceb917e361f488e4cb42a640",
          "url": "https://github.com/dimensionalOS/dimos/commit/943ce13c67e679e75452557f9457f1ba58541091"
        },
        "date": 1790979300466,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.031,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 397,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.585,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.136,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e9a1ec0498a203187c6ea3e782fba83cb5779535",
          "message": "refactor(manipulation): skills report facts instead of error codes (#4401)",
          "timestamp": "2026-10-03T00:16:58-07:00",
          "tree_id": "258cd2a97575f3575fa0e5a9bda1dca0d5af4937",
          "url": "https://github.com/dimensionalOS/dimos/commit/e9a1ec0498a203187c6ea3e782fba83cb5779535"
        },
        "date": 1791011987648,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 400,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2739.751,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "60aee3c05e86ae15c9e38df960918414cf1904c1",
          "message": "r1pro: head cameras straight off V4L2, intrinsics from the factory calibration (#4388)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-03T00:45:21-07:00",
          "tree_id": "5406c0e83bffd810cece15c9bba0687315ae65b0",
          "url": "https://github.com/dimensionalOS/dimos/commit/60aee3c05e86ae15c9e38df960918414cf1904c1"
        },
        "date": 1791013699144,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2024.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.595,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.151,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "db3d0ca9f31d726bfea3a78e8a57662292372d50",
          "message": "fix(sim): reject orphaned MuJoCo shared-memory buffers (#4337)",
          "timestamp": "2026-10-03T17:51:09-07:00",
          "tree_id": "91e640f27eba6853ab057c8c66ad6a529bf5263a",
          "url": "https://github.com/dimensionalOS/dimos/commit/db3d0ca9f31d726bfea3a78e8a57662292372d50"
        },
        "date": 1791075237313,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2016.594,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 392,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.46,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.141,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.056,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c2189a6ea5f431bf6c9bd2111acc0914d375121d",
          "message": "r1pro: hardware-sync the two head cameras (FSYNC) (#4383)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-05T12:27:10-07:00",
          "tree_id": "ce77c018b749a7b395221b4a3e0c2a7aa0c9a497",
          "url": "https://github.com/dimensionalOS/dimos/commit/c2189a6ea5f431bf6c9bd2111acc0914d375121d"
        },
        "date": 1791229214708,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2038.367,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.633,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 148.971,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a79ad67c3f1a9f2a1c9ee73e2a1caf89f67f10e7",
          "message": "perf(sim): faster MuJoCo simulator start-up (#4281)",
          "timestamp": "2026-10-05T22:01:59Z",
          "tree_id": "0965d78653ea1b82a6aee35346f006ad06d7917e",
          "url": "https://github.com/dimensionalOS/dimos/commit/a79ad67c3f1a9f2a1c9ee73e2a1caf89f67f10e7"
        },
        "date": 1791237909318,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2012.555,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 400,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.583,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.202,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "875e64d284b9bd272f30762e07ccf6e51bc93c10",
          "message": "ci: reject AI co-authors in incoming commits (#4346)",
          "timestamp": "2026-10-05T22:38:50Z",
          "tree_id": "e938dd2ad418f3c2d12a0a32853f7aaf2e3ba2c7",
          "url": "https://github.com/dimensionalOS/dimos/commit/875e64d284b9bd272f30762e07ccf6e51bc93c10"
        },
        "date": 1791240107437,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2025.445,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.537,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b18930752de93bbf71c7741c5a1eb8a2d7e05c84",
          "message": "Make Mac tests required (#4321)",
          "timestamp": "2026-10-06T02:01:29+03:00",
          "tree_id": "04f066ab304939e550fcf8e28f748cb9167b81f9",
          "url": "https://github.com/dimensionalOS/dimos/commit/b18930752de93bbf71c7741c5a1eb8a2d7e05c84"
        },
        "date": 1791241468632,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2023.961,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 395,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.823,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.227,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ee0e99f0d5995da8e3a9f2b98355f6bf2ac9c17c",
          "message": "Drop cooldown from internal docker (#4309)",
          "timestamp": "2026-10-06T02:05:25+03:00",
          "tree_id": "9289cbd0bb1f5f06eed8b50355580928979dc118",
          "url": "https://github.com/dimensionalOS/dimos/commit/ee0e99f0d5995da8e3a9f2b98355f6bf2ac9c17c"
        },
        "date": 1791241700044,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2056.336,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 404,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.079,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.164,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.202,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f10c2f3823d485cc9e44ed7c631e0ef75360773d",
          "message": "fix: deskewed pointlio point clouds (#4423)",
          "timestamp": "2026-10-06T02:07:57+03:00",
          "tree_id": "53bd1c32d2569b2210d942bdbf07a21916713c4d",
          "url": "https://github.com/dimensionalOS/dimos/commit/f10c2f3823d485cc9e44ed7c631e0ef75360773d"
        },
        "date": 1791241846917,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2020.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.964,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.52,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c419f9933087ce5e11140ca3763eff562e20b39a",
          "message": "fix: remove default lidar ip (#4416)",
          "timestamp": "2026-10-06T02:23:05+03:00",
          "tree_id": "995d2db4bd495ec5c467d266969d73aa3dddf01a",
          "url": "https://github.com/dimensionalOS/dimos/commit/c419f9933087ce5e11140ca3763eff562e20b39a"
        },
        "date": 1791242760867,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2011.684,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.338,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.263,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49974392+Nabla7@users.noreply.github.com",
            "name": "Pim Van den Bosch",
            "username": "Nabla7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7554a447c1afbe1df1f825971db0b36ebf518190",
          "message": "Fix RPC shell startup with older IPython versions (#4306)",
          "timestamp": "2026-10-05T23:48:32Z",
          "tree_id": "6cade672a15de7cdb2231e4849e05ebe5c6c2774",
          "url": "https://github.com/dimensionalOS/dimos/commit/7554a447c1afbe1df1f825971db0b36ebf518190"
        },
        "date": 1791244281195,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2023.395,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.96,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.199,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.2,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b8cfd0ff177dc51aaa8926c9c9cbb8ef4b09a69d",
          "message": "ci: require policy from the event base (#4443)",
          "timestamp": "2026-10-06T04:12:32Z",
          "tree_id": "74d0218b27c21508c2b9663d430ed0fadc4cca79",
          "url": "https://github.com/dimensionalOS/dimos/commit/b8cfd0ff177dc51aaa8926c9c9cbb8ef4b09a69d"
        },
        "date": 1791260119979,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2030.07,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 401,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.029,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.188,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.164,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0",
          "message": "perf: overlap model loads and heavy imports with the blueprint deploy (#4285)",
          "timestamp": "2026-10-05T22:34:13-07:00",
          "tree_id": "bb16775419f95c779a882f6f37dbf8915f1c0aca",
          "url": "https://github.com/dimensionalOS/dimos/commit/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0"
        },
        "date": 1791265039117,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2018.945,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 401,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.967,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.156,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.242,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7b1d8ab5dfddafd3a84c39fe58e2ea6c5e873915",
          "message": "Remove skipif_macos_bug from tests that pass on Darwin. (#4204)\n\nCo-authored-by: bogwi <bogdan@dimensional.org>",
          "timestamp": "2026-10-06T23:29:27+08:00",
          "tree_id": "f37afd7e6bb20e5f1d2af6cfeadb0150863dac4c",
          "url": "https://github.com/dimensionalOS/dimos/commit/7b1d8ab5dfddafd3a84c39fe58e2ea6c5e873915"
        },
        "date": 1791301020484,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1785.668,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.009,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.34,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.301,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d1f599c5ceb45dedbf89f4722fabb13cf01b4399",
          "message": "chore(codeowners): Andrew/Ivan to codeowners for nav (#4447)",
          "timestamp": "2026-10-06T19:03:44+03:00",
          "tree_id": "4c9b4c35b1217809a7848e48b3b15dd2bd094132",
          "url": "https://github.com/dimensionalOS/dimos/commit/d1f599c5ceb45dedbf89f4722fabb13cf01b4399"
        },
        "date": 1791302797642,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2024.062,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 395,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.666,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.273,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "07edae3181158fd76f1aac67d56cab2102886427",
          "message": "perf(perception): CLIP embeddings without transformers or torch (#4286)",
          "timestamp": "2026-10-06T20:42:32+03:00",
          "tree_id": "a216723c0b03f90a4356185fd4dd124feb679490",
          "url": "https://github.com/dimensionalOS/dimos/commit/07edae3181158fd76f1aac67d56cab2102886427"
        },
        "date": 1791308740814,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2038.902,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2732.625,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.348,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.518,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5b37e609adb5736e0eebb9d6bf2b9b1aa43d5571",
          "message": "refactor(agents): SkillResult reports facts (#4404)",
          "timestamp": "2026-10-06T22:41:18+03:00",
          "tree_id": "00c3fc70a15873421253b30b729755f556bf66fd",
          "url": "https://github.com/dimensionalOS/dimos/commit/5b37e609adb5736e0eebb9d6bf2b9b1aa43d5571"
        },
        "date": 1791315847854,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2054.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 392,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.523,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.301,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.228,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "99a2a4f8a32d2856ee09a2914f6154f2e262d75d",
          "message": "feat(web): add WebRTC video signaling to the protocol and the relay (#4264)",
          "timestamp": "2026-10-06T23:39:01+03:00",
          "tree_id": "cb1e69227c3840c2d2989d0803fbcadec8235e40",
          "url": "https://github.com/dimensionalOS/dimos/commit/99a2a4f8a32d2856ee09a2914f6154f2e262d75d"
        },
        "date": 1791319330696,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1997.613,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.036,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.189,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4048c568a4e634eeab6f4655f0d94f12db950f57",
          "message": "feat(evals): environments and cases specify which artifacts are given to the evaluated agent (#4384)",
          "timestamp": "2026-10-06T16:39:38-07:00",
          "tree_id": "0adaf0048015b8e5fe3dc16856c8ae534892c80f",
          "url": "https://github.com/dimensionalOS/dimos/commit/4048c568a4e634eeab6f4655f0d94f12db950f57"
        },
        "date": 1791330163310,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2021.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.852,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.156,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.214,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4acbaddb43cefe5bd028bdecf97d75d0c6fd7227",
          "message": "perf: stop loading heavy libraries in processes that never use them (#4282)",
          "timestamp": "2026-10-06T17:51:29-07:00",
          "tree_id": "6aa3b934b2ca0f8f58ca0ce46c2cdecfe6cf0c0a",
          "url": "https://github.com/dimensionalOS/dimos/commit/4acbaddb43cefe5bd028bdecf97d75d0c6fd7227"
        },
        "date": 1791334452122,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1601.258,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2733.152,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.617,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "faf1bed3a5c07b1023f484b7cb544cee5b7908cd",
          "message": "docs: ray tracer (#4411)",
          "timestamp": "2026-10-06T18:24:47-07:00",
          "tree_id": "42dbb668e223683b640b312935e7a23a90774f1e",
          "url": "https://github.com/dimensionalOS/dimos/commit/faf1bed3a5c07b1023f484b7cb544cee5b7908cd"
        },
        "date": 1791336455587,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1616.949,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.458,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.598,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.651,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f22803d194135b86435893d2c2395bdbdf76c0d0",
          "message": "fix: resolve native sources for pip installations (#3970)",
          "timestamp": "2026-10-07T09:18:11+03:00",
          "tree_id": "ce94b943392660053c9e74d55833c437ef3bf4ff",
          "url": "https://github.com/dimensionalOS/dimos/commit/f22803d194135b86435893d2c2395bdbdf76c0d0"
        },
        "date": 1791354068479,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1597.902,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 389,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.806,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.895,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.455,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e7e289830d8e7d79426d4892ecd70f8997ed7922",
          "message": "Ivan/feat/go2 nav fixes (#4449)",
          "timestamp": "2026-10-07T20:54:22+03:00",
          "tree_id": "d72ac943e5cfdf99d4fd672f47e509cbee6857ff",
          "url": "https://github.com/dimensionalOS/dimos/commit/e7e289830d8e7d79426d4892ecd70f8997ed7922"
        },
        "date": 1791395839716,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1515.895,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.344,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.375,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.22,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a017963b9372711763e705dfea6acd6542008075",
          "message": "depth2depth: Depth Anything cloud calibrated per pixel to the lidar (#4330)",
          "timestamp": "2026-10-07T20:14:45Z",
          "tree_id": "484e6f084ec4655bd485426b92a884456f15e84f",
          "url": "https://github.com/dimensionalOS/dimos/commit/a017963b9372711763e705dfea6acd6542008075"
        },
        "date": 1791404298704,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1484.125,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 344,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.954,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.363,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.307,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d0e2332679bbc9376304d091863c40c696556933",
          "message": "fix: lazy voxel normal calculations and voxel chunking (#4324)\n\nCo-authored-by: Jeff Hykin <jeff.hykin@gmail.com>",
          "timestamp": "2026-10-07T23:55:20+03:00",
          "tree_id": "ce2d9ee14026daf916e3b314c48b1dae4501b2c0",
          "url": "https://github.com/dimensionalOS/dimos/commit/d0e2332679bbc9376304d091863c40c696556933"
        },
        "date": 1791406706277,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1515.949,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.985,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.187,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aea28cfce4f7bd02afc4deb1cc9715e5d15fc8b5",
          "message": "feat(imitation): publish validated episode JSON and HUD status (#4393)",
          "timestamp": "2026-10-07T14:10:27-07:00",
          "tree_id": "140c4f8d1c60c8cca3bcbc776e995ffa1f4b9150",
          "url": "https://github.com/dimensionalOS/dimos/commit/aea28cfce4f7bd02afc4deb1cc9715e5d15fc8b5"
        },
        "date": 1791407592139,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1529.824,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.733,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.277,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.227,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dbb630f8e298218e9862e03dc24d6ba47080d20c",
          "message": "feat(evals): add six robosuite scenes (#4413)",
          "timestamp": "2026-10-08T01:35:29+03:00",
          "tree_id": "3f26421a2470c9e4a9904862ee1b615589d92df3",
          "url": "https://github.com/dimensionalOS/dimos/commit/dbb630f8e298218e9862e03dc24d6ba47080d20c"
        },
        "date": 1791412703881,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1524.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.093,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.543,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.257,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "504e582062f1d528366d2e9592cc2a0489d5485d",
          "message": "feat(evals): run evals in Docker containers with dimos evals run --docker (#4225)\n\nCo-authored-by: stash <pomichterstash@gmail.com>",
          "timestamp": "2026-10-07T18:36:14-07:00",
          "tree_id": "be3a192bd2e20368734669dec54a14e189c0e18f",
          "url": "https://github.com/dimensionalOS/dimos/commit/504e582062f1d528366d2e9592cc2a0489d5485d"
        },
        "date": 1791423532440,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1529.922,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.535,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.235,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "90bf6a1343656069355ddeda64af044d44b733c6",
          "message": "feat(control): publish measured end-effector poses on tf (#4450)",
          "timestamp": "2026-10-08T01:54:57Z",
          "tree_id": "e6078d9e35284860bbf39d87cb8dfdd33f1c0fbc",
          "url": "https://github.com/dimensionalOS/dimos/commit/90bf6a1343656069355ddeda64af044d44b733c6"
        },
        "date": 1791424670168,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.275,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5b0d592517597b01a8c8dcf4883f7caacda83164",
          "message": "r1: enable config  (#4358)",
          "timestamp": "2026-10-08T03:05:47Z",
          "tree_id": "f5b9df8c96047ac0b827975db1a2ce7cafcdcce6",
          "url": "https://github.com/dimensionalOS/dimos/commit/5b0d592517597b01a8c8dcf4883f7caacda83164"
        },
        "date": 1791428930913,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.852,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.714,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.342,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "64019369eb536af3cb6690679462d577eac238bc",
          "message": "feat(evals): one raw robot bridge for navigation and manipulation (#4417)",
          "timestamp": "2026-10-08T07:23:56+03:00",
          "tree_id": "9741b6eabfef1d553088b6d117cad29d05b7fc3f",
          "url": "https://github.com/dimensionalOS/dimos/commit/64019369eb536af3cb6690679462d577eac238bc"
        },
        "date": 1791433605155,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.828,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.968,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.59,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.343,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "55f1061cad657fa4d2bf56563007969c6d85f7c4",
          "message": "feat(dataprep): align recordings with explicit feature schemas (#4394)",
          "timestamp": "2026-10-07T21:28:08-07:00",
          "tree_id": "946894f8e7ca74d88b943b4ef88b346ff13f71c5",
          "url": "https://github.com/dimensionalOS/dimos/commit/55f1061cad657fa4d2bf56563007969c6d85f7c4"
        },
        "date": 1791433849816,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1542.641,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2726.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.461,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.211,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dc80d89b89558e9ef3bcccc7949b20aa3e2b4d3b",
          "message": "feat(go2): teleop-only raw recording blueprint, KeyboardTeleop key presses as Joy (#4478)\n\nCo-authored-by: Krishna_Hundekari <krishna@dimensionalos.com>",
          "timestamp": "2026-10-08T00:15:23-07:00",
          "tree_id": "97f388897a3924fbf5ffdadd3887d99593596bc1",
          "url": "https://github.com/dimensionalOS/dimos/commit/dc80d89b89558e9ef3bcccc7949b20aa3e2b4d3b"
        },
        "date": 1791443909222,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1522.645,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.864,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.566,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.21,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "293b92735544a8b8d14972b8271bdd92668f1e87",
          "message": "cloud: dimos data upload sends a spatial preview for the console (#4458)",
          "timestamp": "2026-10-08T02:39:27-07:00",
          "tree_id": "b9e92bcfaa40fe5dad1f9aa60e26779409d612e6",
          "url": "https://github.com/dimensionalOS/dimos/commit/293b92735544a8b8d14972b8271bdd92668f1e87"
        },
        "date": 1791452543003,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1517.016,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.871,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.24,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "feb30ea34a6392c985d384d1b6d510029376f6a9",
          "message": "chore: clean debug logging (#4460)\n\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-08T10:21:45Z",
          "tree_id": "6bfcd75cc70d7224e9a5cc9eedcfc74841e8dada",
          "url": "https://github.com/dimensionalOS/dimos/commit/feb30ea34a6392c985d384d1b6d510029376f6a9"
        },
        "date": 1791455089422,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1508.242,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.875,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.235,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4828b504f8fc0aa64968b79f75608f7ca67f092e",
          "message": "fix(cloud): preview picks the right streams; a failed timelapse keeps the preview (#4480)",
          "timestamp": "2026-10-08T03:43:42-07:00",
          "tree_id": "7a72ea21bb06eb04b73455cbc652e0eb4f8a369a",
          "url": "https://github.com/dimensionalOS/dimos/commit/4828b504f8fc0aa64968b79f75608f7ca67f092e"
        },
        "date": 1791456393405,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1517.84,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 350,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.728,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.316,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.281,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dad5ad7cb4277ee4b266f415067bda6e352ab419",
          "message": "feat: go2 and mid360 sim (#4441)",
          "timestamp": "2026-10-08T17:29:36Z",
          "tree_id": "7ce800f6d86d46fbf13acf1a4da40de4ae5091be",
          "url": "https://github.com/dimensionalOS/dimos/commit/dad5ad7cb4277ee4b266f415067bda6e352ab419"
        },
        "date": 1791480753033,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1514.387,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.717,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.332,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.274,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0761d4ee215821f4116ff53e88465df4149507a4",
          "message": "Updated code ownership for CC to all code (#4444)",
          "timestamp": "2026-10-08T15:13:55-07:00",
          "tree_id": "9c92f7ddc229b60dc86ac6c5b9c53abdbf1ee7c2",
          "url": "https://github.com/dimensionalOS/dimos/commit/0761d4ee215821f4116ff53e88465df4149507a4"
        },
        "date": 1791497797461,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1520.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.366,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.324,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.28,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6ed445f0f45898f8466c0c5ae2850ceb4b0d5ad3",
          "message": "feat(control): add control contract package (#4263)",
          "timestamp": "2026-10-08T15:14:44-07:00",
          "tree_id": "b0d7271c8eeeeea8c7496293f8049cedd6668315",
          "url": "https://github.com/dimensionalOS/dimos/commit/6ed445f0f45898f8466c0c5ae2850ceb4b0d5ad3"
        },
        "date": 1791497846087,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1524.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 358,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.216,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.257,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9197ea5b1dca07c6c10f4b1141fd627754c146c7",
          "message": "perf(core): warm up stream types and skills right after a module is constructed (#4284)",
          "timestamp": "2026-10-09T01:22:46+03:00",
          "tree_id": "d65e539e9f3553629d5d8c833e42ae2372e184bb",
          "url": "https://github.com/dimensionalOS/dimos/commit/9197ea5b1dca07c6c10f4b1141fd627754c146c7"
        },
        "date": 1791498322828,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1576.41,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 345,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.658,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.776,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c53724a060988f279c347e954beb0e8916a3ecdb",
          "message": "fix: remove blank g1 image (#4486)",
          "timestamp": "2026-10-09T01:09:09Z",
          "tree_id": "e4e9ecbfb80ced608e9322163d045b3696b02f3e",
          "url": "https://github.com/dimensionalOS/dimos/commit/c53724a060988f279c347e954beb0e8916a3ecdb"
        },
        "date": 1791508319015,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.797,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.111,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.742,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.517,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c25d33135d422464b0d6c09135c8b51176f4ed61",
          "message": "feat(web): receive WebRTC video tracks in the SDK and the cockpit (#4265)",
          "timestamp": "2026-10-09T07:18:20+03:00",
          "tree_id": "8efc4e5b2c1e439105b381d4bdaa66d18e14cb57",
          "url": "https://github.com/dimensionalOS/dimos/commit/c25d33135d422464b0d6c09135c8b51176f4ed61"
        },
        "date": 1791519656988,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1577.66,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.102,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.604,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4d7496165a5bb3771967bcbf1c5ee82a70611a0c",
          "message": "cloud: console preview carries joystick input (#4491)",
          "timestamp": "2026-10-08T21:33:44-07:00",
          "tree_id": "39818ff87d92c4a3d5844a423f91c399af065b41",
          "url": "https://github.com/dimensionalOS/dimos/commit/4d7496165a5bb3771967bcbf1c5ee82a70611a0c"
        },
        "date": 1791520605630,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1557.562,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.094,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.441,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.724,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0147381f1a28a3dd9df8bce93de409baf1f85ca5",
          "message": "fix relay e2e timeout mac test (#4488)",
          "timestamp": "2026-10-09T17:02:02+03:00",
          "tree_id": "c88f9043857d34fba2698e267b16ca5c18267730",
          "url": "https://github.com/dimensionalOS/dimos/commit/0147381f1a28a3dd9df8bce93de409baf1f85ca5"
        },
        "date": 1791554689446,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1577.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 342,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.58,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.554,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e6e7a6d684294434d7523cfc7309a2169e3621c5",
          "message": "feat: go2 nav sim (#4481)",
          "timestamp": "2026-10-09T19:29:58Z",
          "tree_id": "228235b9f59609f56339e3be563e5202e9c7e905",
          "url": "https://github.com/dimensionalOS/dimos/commit/e6e7a6d684294434d7523cfc7309a2169e3621c5"
        },
        "date": 1791574368850,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.776,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4493f5ca86faf78f7e7ad681ee9bba3c88345892",
          "message": "Add dimsim ready state to avoid race condition (#4495)",
          "timestamp": "2026-10-09T19:59:55Z",
          "tree_id": "a760abada8b4ed224df551927bc98ec0b2a241dc",
          "url": "https://github.com/dimensionalOS/dimos/commit/4493f5ca86faf78f7e7ad681ee9bba3c88345892"
        },
        "date": 1791576167028,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.539,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 350,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.449,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "647a9b1ceb42df774333588e4d2a6e90206ca652",
          "message": "fix: test_walk_forward (#4501)",
          "timestamp": "2026-10-09T18:10:08-07:00",
          "tree_id": "60368b1ec1cc704349c60471cbf9336c6e65305a",
          "url": "https://github.com/dimensionalOS/dimos/commit/647a9b1ceb42df774333588e4d2a6e90206ca652"
        },
        "date": 1791594767216,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1580.527,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.001,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.548,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "622d924eceb7c5b4dcfed12d000c42f5268e4cc2",
          "message": "refactor(dannav): rework DanHolonomicTC tracking (#4493)",
          "timestamp": "2026-10-10T11:10:05+09:00",
          "tree_id": "d5912c1fc6fcd7f19e6995b3215f2f830746eabd",
          "url": "https://github.com/dimensionalOS/dimos/commit/622d924eceb7c5b4dcfed12d000c42f5268e4cc2"
        },
        "date": 1791598362182,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1575.375,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 356,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.999,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.904,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "649b045296ef5d059dfcdfe8adfb13dd72e9d8aa",
          "message": "Move a750-control out of our extras (#4494)",
          "timestamp": "2026-10-09T20:59:32-07:00",
          "tree_id": "08403c2f50ad0d19e0440025e03043244c269f32",
          "url": "https://github.com/dimensionalOS/dimos/commit/649b045296ef5d059dfcdfe8adfb13dd72e9d8aa"
        },
        "date": 1791604938381,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.781,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 345,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.499,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.669,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8b17f2ff38979a79112ad8c38c9a5e88bcde4c77",
          "message": "feat(control): add description presets and limits_from_urdf (#4273)",
          "timestamp": "2026-10-09T21:27:27-07:00",
          "tree_id": "1c372b362014292b672cfa0b7c6f32d661bd90f4",
          "url": "https://github.com/dimensionalOS/dimos/commit/8b17f2ff38979a79112ad8c38c9a5e88bcde4c77"
        },
        "date": 1791606610727,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1582.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.697,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.617,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.471,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "066d3f3fc21a0785b7fe1c5f40e572c60cd2e4d6",
          "message": "Migrate planning and point-cloud filtering to RoboPlan 0.7 (#4420)",
          "timestamp": "2026-10-10T11:47:07-07:00",
          "tree_id": "8acdf47d07bda48b8ae978cabe6ad2a6e52f2697",
          "url": "https://github.com/dimensionalOS/dimos/commit/066d3f3fc21a0785b7fe1c5f40e572c60cd2e4d6"
        },
        "date": 1791658192819,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.152,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.138,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.648,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1e7b4918bc626de4fa6592fb023d4b3aee932754",
          "message": "test: give the wrong-layout policy blob cases readable ids (#4516)",
          "timestamp": "2026-10-11T02:37:32Z",
          "tree_id": "02d1a6455c8197bc0dbec630dcce472fafd7c082",
          "url": "https://github.com/dimensionalOS/dimos/commit/1e7b4918bc626de4fa6592fb023d4b3aee932754"
        },
        "date": 1791686414489,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.52,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.016,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.634,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fd989ff70f0c86a1e5c53f9414944c9fc61dd963",
          "message": "Update ROS image (#4496)",
          "timestamp": "2026-10-12T17:46:28+01:00",
          "tree_id": "e92a9bffc74a22e9b20b69f5a4b3abd4429f7fe0",
          "url": "https://github.com/dimensionalOS/dimos/commit/fd989ff70f0c86a1e5c53f9414944c9fc61dd963"
        },
        "date": 1791823758912,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1578.188,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.046,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.658,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "475fad598448ab125fb8ce19fb8496b759b7d58b",
          "message": "feat(web): send video as WebRTC tracks from the relay bridge (#4267)",
          "timestamp": "2026-10-12T19:57:29+03:00",
          "tree_id": "74fc49bc2ff5427b472f56aef685e333cfd5df7b",
          "url": "https://github.com/dimensionalOS/dimos/commit/475fad598448ab125fb8ce19fb8496b759b7d58b"
        },
        "date": 1791824425045,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1597.328,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 339,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.136,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.562,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c0d14726f2f2b88ba4d901d4fd3fc83261a1fd8b",
          "message": "refactor: deprecate nav eval tool (#4438)",
          "timestamp": "2026-10-12T19:25:40Z",
          "tree_id": "2079233757d5c6a1146bc57d3eca59d7cc4325cc",
          "url": "https://github.com/dimensionalOS/dimos/commit/c0d14726f2f2b88ba4d901d4fd3fc83261a1fd8b"
        },
        "date": 1791833305704,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1573.219,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.427,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "908fc40a07056d58453b6a219d0c82e9b684dead",
          "message": "feat(evals): find all cones in building (#4414)",
          "timestamp": "2026-10-12T15:27:03-07:00",
          "tree_id": "2456f169c1300a4c29b370d28a2546ab843d59ab",
          "url": "https://github.com/dimensionalOS/dimos/commit/908fc40a07056d58453b6a219d0c82e9b684dead"
        },
        "date": 1791844192682,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.934,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.663,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.415,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "288875102ff85a0994e2c6f4b5d63d7f0734dfd0",
          "message": "Move why above what in PR descriptions (#4514)",
          "timestamp": "2026-10-13T06:19:18+03:00",
          "tree_id": "2f7599093fe8aedf4b63f7ea1f11d8364c98383d",
          "url": "https://github.com/dimensionalOS/dimos/commit/288875102ff85a0994e2c6f4b5d63d7f0734dfd0"
        },
        "date": 1791861739978,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1549.234,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.636,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.642,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26ccb5d88e8e1bd674e8339e821a8a8bc5880b48",
          "message": "depth2depth: optimize cloud calculations (#4549)",
          "timestamp": "2026-10-12T21:27:18-07:00",
          "tree_id": "fc6b5566a7b6dcf76269d6576589bc6a37959db4",
          "url": "https://github.com/dimensionalOS/dimos/commit/26ccb5d88e8e1bd674e8339e821a8a8bc5880b48"
        },
        "date": 1791865817493,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1559.621,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.735,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.449,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.618,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3b6d5eafe66bcf8378b50b86c422a48a05916321",
          "message": "chore: remove shm transports (#4550)",
          "timestamp": "2026-10-13T08:11:17+03:00",
          "tree_id": "7658beeeb2eeda79d3bf59c16fc9dd910d88041e",
          "url": "https://github.com/dimensionalOS/dimos/commit/3b6d5eafe66bcf8378b50b86c422a48a05916321"
        },
        "date": 1791868463828,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1558.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.048,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.506,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "wuy691128@gmail.com",
            "name": "poorwym",
            "username": "poorwym"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bd82b3dd15c2c7b0ebf59bfb158aa4626b6570ac",
          "message": "Yimo/feat/host fragment (#3967)\n\nCo-authored-by: yimo <yimo@dimensional.org>\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-13T12:17:18Z",
          "tree_id": "c6ae6671b266eb495aa60a20b8893428ecd483d8",
          "url": "https://github.com/dimensionalOS/dimos/commit/bd82b3dd15c2c7b0ebf59bfb158aa4626b6570ac"
        },
        "date": 1791894009351,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1562.855,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 356,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.479,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.529,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "19e9c0a198af61be1a1901b4027c00af2a2acdf2",
          "message": "feat(models): add SigLIP2 embedding wrapper (#4539)\n\nCo-authored-by: bogwi <bogdan@dimensional.org>",
          "timestamp": "2026-10-13T19:13:45+03:00",
          "tree_id": "9d121bcb3a2cc2096fc34e68a313dd293d22c6ea",
          "url": "https://github.com/dimensionalOS/dimos/commit/19e9c0a198af61be1a1901b4027c00af2a2acdf2"
        },
        "date": 1791908195456,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.781,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.693,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.805,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.585,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a84700b089b9948831bb489d643a29f5d3d47cda",
          "message": "feat(mapping): live mesh for go2 (#4497)",
          "timestamp": "2026-10-13T17:13:22Z",
          "tree_id": "d1f9a3b9da42363b0568a3fc455ea27772421cc6",
          "url": "https://github.com/dimensionalOS/dimos/commit/a84700b089b9948831bb489d643a29f5d3d47cda"
        },
        "date": 1791911773333,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1565.832,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.717,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e132bc5290574eb22355afe5b1ca580c5a2036f6",
          "message": "refactor(habitat): extract asset paths to constants (#4459)",
          "timestamp": "2026-10-13T17:38:01Z",
          "tree_id": "504fec46e75ddc8df0785c56ab8d2e81a0e54184",
          "url": "https://github.com/dimensionalOS/dimos/commit/e132bc5290574eb22355afe5b1ca580c5a2036f6"
        },
        "date": 1791913264590,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1553.996,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.066,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.448,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1c5e5addce8619e42f1536f2385a700262547d8a",
          "message": "feat(sim): the 555 office as the default MuJoCo",
          "timestamp": "2026-10-13T13:05:36-07:00",
          "tree_id": "7efb6a3b371f66d7be1ed9ecfa2ef09e7127559f",
          "url": "https://github.com/dimensionalOS/dimos/commit/1c5e5addce8619e42f1536f2385a700262547d8a"
        },
        "date": 1791922112290,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1566.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.395,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "49fa9f8c33e4994ebac8ee266303732e1d7c7983",
          "message": "docs: remove hosted architecture draft (#4556)",
          "timestamp": "2026-10-13T20:16:06Z",
          "tree_id": "e23068f5f7610b1a12eac5bd69c4681bc3e94e13",
          "url": "https://github.com/dimensionalOS/dimos/commit/49fa9f8c33e4994ebac8ee266303732e1d7c7983"
        },
        "date": 1791922734378,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.039,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 340,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.98,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.603,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "263b0591c9980d9e11c605a86811216bbdb8018f",
          "message": "feat(porcelain): add remote stream discovery and subscription  (#4492)\n\nCo-authored-by: Paul Nechifor <paul@nechifor.net>",
          "timestamp": "2026-10-14T00:19:35+03:00",
          "tree_id": "925b77c81287cfe93fa65ae8a8e71b7988f1fb63",
          "url": "https://github.com/dimensionalOS/dimos/commit/263b0591c9980d9e11c605a86811216bbdb8018f"
        },
        "date": 1791926540790,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1588.074,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.045,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.456,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5d97134cc1fda66e9f26127f59b3b78687648e31",
          "message": "r1pro: boot the vendor stack with one setup script (#4558)",
          "timestamp": "2026-10-13T20:12:22-07:00",
          "tree_id": "49282bc854c2cd512c2524f8a16a7909e524d32b",
          "url": "https://github.com/dimensionalOS/dimos/commit/5d97134cc1fda66e9f26127f59b3b78687648e31"
        },
        "date": 1791947718592,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.098,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.364,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "alxbouchard@gmail.com",
            "name": "alxbouchard",
            "username": "alxbouchard"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0f65afc34c6012e2d7924e620fb24fcb9f171510",
          "message": "fix(memory): record depth images losslessly instead of as JPEG (#4554)\n\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-14T03:31:34Z",
          "tree_id": "e92c0f31d14486b530b83d79cfc64d96cb53d93c",
          "url": "https://github.com/dimensionalOS/dimos/commit/0f65afc34c6012e2d7924e620fb24fcb9f171510"
        },
        "date": 1791948867752,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1557.73,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.332,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.461,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.606,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "moshi@dimensional.org",
            "name": "moshiii-wei",
            "username": "moshiii-wei"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "886af5be9b612e6f479075da01b5c500fd25c7dd",
          "message": "feat(evals): manifest-driven LIBERO-PRO/RoboCasa MuJoCo smoke (#4517)\n\nCo-authored-by: Moshi Wei <wmswms938@gmail.com>",
          "timestamp": "2026-10-13T23:14:01-07:00",
          "tree_id": "69cdcfe362ef8a336dd7d063c83fdb227b370b0b",
          "url": "https://github.com/dimensionalOS/dimos/commit/886af5be9b612e6f479075da01b5c500fd25c7dd"
        },
        "date": 1791958609369,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1586.348,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2718.603,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.488,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.035,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5a694d6dcbcbce868b14eccbe5be60c4c623c4c9",
          "message": "refactor perception stack to modular design (#4552)",
          "timestamp": "2026-10-14T16:02:28+09:00",
          "tree_id": "0b75da272bbecf11f3551ae344972eddb6dca9cc",
          "url": "https://github.com/dimensionalOS/dimos/commit/5a694d6dcbcbce868b14eccbe5be60c4c623c4c9"
        },
        "date": 1791961500995,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.531,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.212,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.491,
            "unit": "G",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3faae1f93ed44e1119ec8bd5916132bed34b2219",
          "message": "ci: reject relative Python imports with Ruff (#4563)",
          "timestamp": "2026-10-14T15:44:59+03:00",
          "tree_id": "903c6d32f72d57a678d4709fab16cb3c1d19aff4",
          "url": "https://github.com/dimensionalOS/dimos/commit/3faae1f93ed44e1119ec8bd5916132bed34b2219"
        },
        "date": 1791982081148,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1589.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.636,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.588,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5a7149c401eabbba8e2e676bff54ef5859b3b69c",
          "message": "dimOS Desktop contract: dimos.yaml and install.sh fixes (#4428)",
          "timestamp": "2026-10-15T01:44:37+03:00",
          "tree_id": "607d756b144bd061e3315456326a7c6f526013da",
          "url": "https://github.com/dimensionalOS/dimos/commit/5a7149c401eabbba8e2e676bff54ef5859b3b69c"
        },
        "date": 1792018051171,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1564.09,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.107,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.57,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "19c7db2e9056dd6454e82fcfad341f343b5aa8f7",
          "message": "fix(lcm): allow shutdown from dispatch callbacks (#4515)",
          "timestamp": "2026-10-15T01:27:34Z",
          "tree_id": "7874f9e01368ea7e88648928373e82679c23d26f",
          "url": "https://github.com/dimensionalOS/dimos/commit/19c7db2e9056dd6454e82fcfad341f343b5aa8f7"
        },
        "date": 1792027823695,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1559.703,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.727,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.463,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f21694937364cf756f7bc2917bbf2191cd04d86b",
          "message": "test: delete 37 tests that never run in CI (#4569)",
          "timestamp": "2026-10-15T04:35:20+03:00",
          "tree_id": "59e9900773b166679dba141ed4907cab738ff276",
          "url": "https://github.com/dimensionalOS/dimos/commit/f21694937364cf756f7bc2917bbf2191cd04d86b"
        },
        "date": 1792028288478,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.246,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.59,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5478eb724312b7793a3837d06dc5e4ebc35f6a98",
          "message": "r1pro: Point-LIO odometry on the chassis Mid-360, as r1pro-pointlio (#4360)",
          "timestamp": "2026-10-15T02:14:31Z",
          "tree_id": "e55b47e1382bea3e7c0090eb93c4c455b9f82cea",
          "url": "https://github.com/dimensionalOS/dimos/commit/5478eb724312b7793a3837d06dc5e4ebc35f6a98"
        },
        "date": 1792030636560,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1598.121,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.436,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.438,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.434,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "moshi@dimensional.org",
            "name": "moshiii-wei",
            "username": "moshiii-wei"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1340eb21785c1c74bade40123ba8e3d7bcd480b0",
          "message": "feat(evals): optional QualityWindow frame selection for VQA generate (#4566)\n\nCo-authored-by: Moshi Wei <wmswms938@gmail.com>",
          "timestamp": "2026-10-15T11:58:37-07:00",
          "tree_id": "1cb4a170ed9150b59faf7c61374f76a1338b2dfa",
          "url": "https://github.com/dimensionalOS/dimos/commit/1340eb21785c1c74bade40123ba8e3d7bcd480b0"
        },
        "date": 1792090880098,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1575.801,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.696,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.618,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0bcf47eed57fe00042d737b060693c2e78554453",
          "message": "desktop: installer script and dimos.yaml (#4598)",
          "timestamp": "2026-10-16T02:56:05+03:00",
          "tree_id": "1cb4a170ed9150b59faf7c61374f76a1338b2dfa",
          "url": "https://github.com/dimensionalOS/dimos/commit/0bcf47eed57fe00042d737b060693c2e78554453"
        },
        "date": 1792108746535,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1560.066,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.002,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.441,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.702,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a8fa4bb01726b7856fe6e98e9d7759f6dc74b5b7",
          "message": "Fix all the nix (#4187)\n\nCo-authored-by: autofix-ci[bot] <114827586+autofix-ci[bot]@users.noreply.github.com>\nCo-authored-by: Sam Bull <git@sambull.org>",
          "timestamp": "2026-10-16T03:12:40+03:00",
          "tree_id": "f8ac51e61e345420af4f4a36c01022d19c39389c",
          "url": "https://github.com/dimensionalOS/dimos/commit/a8fa4bb01726b7856fe6e98e9d7759f6dc74b5b7"
        },
        "date": 1792109734798,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.344,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.07,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.454,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e45b7e131bfe8d6941aeba54f009fb71ba59529b",
          "message": "chore: delete dead code that imports undeclared packages (#4576)",
          "timestamp": "2026-10-16T04:03:19+03:00",
          "tree_id": "360ec5d30afe281194b2cfd4ee9f7482e27a261b",
          "url": "https://github.com/dimensionalOS/dimos/commit/e45b7e131bfe8d6941aeba54f009fb71ba59529b"
        },
        "date": 1792112770778,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.895,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.451,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.48,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "afc6a475a76f2360e0ad837075da5fcc2ca22948",
          "message": "chore: remove Python implementation of local planner (#4608)",
          "timestamp": "2026-10-16T07:57:02Z",
          "tree_id": "0832e0d69cbe8d4ce30e42947ab43302ad4a0d9a",
          "url": "https://github.com/dimensionalOS/dimos/commit/afc6a475a76f2360e0ad837075da5fcc2ca22948"
        },
        "date": 1792137605003,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.027,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 363,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.099,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.727,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.659,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "509d0b57ac175a282e47362f4a7485603ef03d89",
          "message": "Fix zombie processes on Mac tests (#4567)",
          "timestamp": "2026-10-16T23:48:32+03:00",
          "tree_id": "6a89ddc3893d587dc3a7dee74e78d93caabbf6b3",
          "url": "https://github.com/dimensionalOS/dimos/commit/509d0b57ac175a282e47362f4a7485603ef03d89"
        },
        "date": 1792183879180,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.391,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.283,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.629,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.384,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b7b4e787dc300753fa749c4f2b17dee9f66554b1",
          "message": "build: check declared dependencies with deptry (#4584)",
          "timestamp": "2026-10-16T23:16:51Z",
          "tree_id": "844133a0cb4c4a6e835ab7413b733b4174bfb0a1",
          "url": "https://github.com/dimensionalOS/dimos/commit/b7b4e787dc300753fa749c4f2b17dee9f66554b1"
        },
        "date": 1792192781152,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.705,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.649,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "08945017b523e7365a76c6255039ac035b2cba09",
          "message": "Enable some tests in CI (#4611)",
          "timestamp": "2026-10-16T18:40:04-07:00",
          "tree_id": "cc369243c6f294f42a2747b6057ae84aba649f87",
          "url": "https://github.com/dimensionalOS/dimos/commit/08945017b523e7365a76c6255039ac035b2cba09"
        },
        "date": 1792201378999,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1563.141,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.449,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.447,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e71566551a2638886e5b62e84ee43b18f9ee0dea",
          "message": "Remove .github/ from CODEOWNERS (#4617)",
          "timestamp": "2026-10-17T07:10:19+03:00",
          "tree_id": "0e170acd5435082d6c8be1762db78e10ab216d75",
          "url": "https://github.com/dimensionalOS/dimos/commit/e71566551a2638886e5b62e84ee43b18f9ee0dea"
        },
        "date": 1792210394920,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1571.188,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.261,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.484,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4bee180187d4e4bf84aac0422264998d39b5681f",
          "message": "perf: prioritize live clouds over premap tiles (#4454)",
          "timestamp": "2026-10-17T14:17:22+03:00",
          "tree_id": "d1d2d6ab5d12f2d374d9f004efb0b22c8af7fd0e",
          "url": "https://github.com/dimensionalOS/dimos/commit/4bee180187d4e4bf84aac0422264998d39b5681f"
        },
        "date": 1792236002609,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1566.871,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.605,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.449,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.402,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "21321b2a9e45a25a099623e12041765f02a1c440",
          "message": "feat: soft cost for low objects (#4609)\n\nCo-authored-by: autofix-ci[bot] <114827586+autofix-ci[bot]@users.noreply.github.com>",
          "timestamp": "2026-10-19T12:41:15-07:00",
          "tree_id": "d82eba39d2eb28da53ce18652b679e133acff86a",
          "url": "https://github.com/dimensionalOS/dimos/commit/21321b2a9e45a25a099623e12041765f02a1c440"
        },
        "date": 1792439062923,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1565.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.288,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.709,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "17fdb6b945da617acc37038297f0caa8b5297fac",
          "message": "dimos-pointlio: re-export pointlio_core (#4648)",
          "timestamp": "2026-10-20T01:03:54Z",
          "tree_id": "8747e7e6e5f796cb260f02d322d53a0db1cea219",
          "url": "https://github.com/dimensionalOS/dimos/commit/17fdb6b945da617acc37038297f0caa8b5297fac"
        },
        "date": 1792458419129,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1589.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 350,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.079,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.707,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.497,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5c36356cfcae1dced6094608138421f283a99d4e",
          "message": "refactor: move module settings out of global config (#4610)",
          "timestamp": "2026-10-19T20:31:33-07:00",
          "tree_id": "cfaf35ba2be6f4a02dcdbe13347081c5aad7e6bc",
          "url": "https://github.com/dimensionalOS/dimos/commit/5c36356cfcae1dced6094608138421f283a99d4e"
        },
        "date": 1792467266638,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1568.418,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.248,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.908,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "97344c8fe91700f6e2360d9cc11678803cb63775",
          "message": "go2/dds: L1 lidar + joint data as ROS types, robot timestamps (#4596)",
          "timestamp": "2026-10-19T20:35:04-07:00",
          "tree_id": "dc74ab6b36d3c37efe0f12e7678aa04ac24dcf29",
          "url": "https://github.com/dimensionalOS/dimos/commit/97344c8fe91700f6e2360d9cc11678803cb63775"
        },
        "date": 1792467469481,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1602.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 356,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.755,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.742,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.909,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4b4f0ce3d7a7356476071a199f0f6db9485ae59e",
          "message": "docs: add robot safety pages (general + R1 Pro) (#4562)",
          "timestamp": "2026-10-19T21:01:27-07:00",
          "tree_id": "e921bc5c6b79701464b060ad4ee15e85eb706358",
          "url": "https://github.com/dimensionalOS/dimos/commit/4b4f0ce3d7a7356476071a199f0f6db9485ae59e"
        },
        "date": 1792469067389,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1562.223,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.918,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.952,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fbe126e741c44e14ee1c23bb17a7e68e2b6a00d6",
          "message": "feat: go2 dds obstacle avoidance config (#4638)",
          "timestamp": "2026-10-20T04:05:43Z",
          "tree_id": "64f85a7afdc678269da7176aed40d61401cf82d4",
          "url": "https://github.com/dimensionalOS/dimos/commit/fbe126e741c44e14ee1c23bb17a7e68e2b6a00d6"
        },
        "date": 1792469309249,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1564.824,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.328,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.488,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.929,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f0f20083e38af5b53d0e5561a24c0717646ad8a6",
          "message": "feat(control): publish intended position commands as JointState (#4646)",
          "timestamp": "2026-10-20T06:19:45Z",
          "tree_id": "8d1d15afdf574ac680b2640b806bcb548f7a269e",
          "url": "https://github.com/dimensionalOS/dimos/commit/f0f20083e38af5b53d0e5561a24c0717646ad8a6"
        },
        "date": 1792477363299,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1562.102,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 353,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.053,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.75,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.955,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "ari@dimensionalos.com",
            "name": "aa-MONTS",
            "username": "aa-MONTS"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b71a775ef1c52830facb25a5b6bc4e2a0195c9f0",
          "message": "fix: avoid macOS daemon startup crash after fork (#4422)\n\nCo-authored-by: Ari Ameri <302640929+aa-MONTS@users.noreply.github.com>",
          "timestamp": "2026-10-20T19:40:02+03:00",
          "tree_id": "845b573a9376e020daf75ed1069c00511e15c29a",
          "url": "https://github.com/dimensionalOS/dimos/commit/b71a775ef1c52830facb25a5b6bc4e2a0195c9f0"
        },
        "date": 1792514618976,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1579.137,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.792,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.82,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.146,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e9f17bb20cf16be8a9648817b8c0dcf34c560231",
          "message": "build: integrate published CDR generator (1/8) (#4247)\n\nCo-authored-by: autofix-ci[bot] <114827586+autofix-ci[bot]@users.noreply.github.com>",
          "timestamp": "2026-10-20T10:02:59-07:00",
          "tree_id": "8d725fbb05a3a225cc0fb32a5f64791e8fce148c",
          "url": "https://github.com/dimensionalOS/dimos/commit/e9f17bb20cf16be8a9648817b8c0dcf34c560231"
        },
        "date": 1792516059634,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1586.828,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 372,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2726.381,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.28,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cb50a9d240f54093718e0a789f20095a83eb4009",
          "message": "feat(porcelain): add app.unload() to remove one running module (#4572)",
          "timestamp": "2026-10-20T11:53:45-07:00",
          "tree_id": "50c9b3ce66a8320fd95881b6bcc04ccb733509d2",
          "url": "https://github.com/dimensionalOS/dimos/commit/cb50a9d240f54093718e0a789f20095a83eb4009"
        },
        "date": 1792522689093,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1541.547,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.791,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.949,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6d93c269c6ef95dfb88524e407c1a57d0125ddf8",
          "message": "fix(manipulation): keep picks planning with the voxel map on (#4519)",
          "timestamp": "2026-10-20T14:53:00-07:00",
          "tree_id": "f84f18157e250ca89533df83c717d7fab7b8f63a",
          "url": "https://github.com/dimensionalOS/dimos/commit/6d93c269c6ef95dfb88524e407c1a57d0125ddf8"
        },
        "date": 1792533349491,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1606.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.534,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.251,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8cecb3c094a2cd42370771d46cd4396b46750ccd",
          "message": "Quiet zenoh (#4746)",
          "timestamp": "2026-10-21T02:39:01Z",
          "tree_id": "fb1c5945d1c07825d4fbee1630b761180f8c788d",
          "url": "https://github.com/dimensionalOS/dimos/commit/8cecb3c094a2cd42370771d46cd4396b46750ccd"
        },
        "date": 1792550509137,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1577.98,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.052,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.633,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.286,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "49ef9291e2510794a63611aad5537f44a3971e66",
          "message": "feat(manipulation): GraspGenX samples 500 grasps and checks every one (#4525)",
          "timestamp": "2026-10-21T03:02:03Z",
          "tree_id": "d5ec27368e60bb6e0084e1fd07bc8e94fc1faf71",
          "url": "https://github.com/dimensionalOS/dimos/commit/49ef9291e2510794a63611aad5537f44a3971e66"
        },
        "date": 1792551887577,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.027,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.979,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.68,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.205,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b3ffc18409c137537ff760383b615c70485b6883",
          "message": "feat(control): add ConnectionModule (#4301)",
          "timestamp": "2026-10-20T20:19:17-07:00",
          "tree_id": "214fc20d661a777d8cde541cb04bd27bbae798d4",
          "url": "https://github.com/dimensionalOS/dimos/commit/b3ffc18409c137537ff760383b615c70485b6883"
        },
        "date": 1792552931299,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1595.535,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.054,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.913,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cb132e1c474f2a063ac2badf93063b92939b7357",
          "message": "perf(web): sampling gate, liveness, RLE zlib, cockpit leak and render fixes (1/9) (#4620)",
          "timestamp": "2026-10-20T20:37:12-07:00",
          "tree_id": "6256ef3d4dec543e09d91c84d1bfd473ef667cb2",
          "url": "https://github.com/dimensionalOS/dimos/commit/cb132e1c474f2a063ac2badf93063b92939b7357"
        },
        "date": 1792553998447,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1594.969,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 361,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.564,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.777,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.09,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "36f54ee2c103471c30d05a12c34780ae57847148",
          "message": "feat(recorder): record new streams after dimos start (#4594)",
          "timestamp": "2026-10-21T07:05:37+03:00",
          "tree_id": "adfe76980b50863b3f2646dc1605e7fc1114a9aa",
          "url": "https://github.com/dimensionalOS/dimos/commit/36f54ee2c103471c30d05a12c34780ae57847148"
        },
        "date": 1792555708163,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1581.121,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 361,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.129,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.75,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.168,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7647368e28b6d3c1d9094888ee361e7e8173933a",
          "message": "feat: alfred nav with local planner and relocalization (#4744)",
          "timestamp": "2026-10-21T01:49:53-07:00",
          "tree_id": "b8770299f541efb3ccac2e32943cee26dec21f4d",
          "url": "https://github.com/dimensionalOS/dimos/commit/7647368e28b6d3c1d9094888ee361e7e8173933a"
        },
        "date": 1792572761249,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1607.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.067,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.963,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0978e4b7373e535b08e59d625eeb16a05d1484ed",
          "message": "test(control): make shared deadman timing deterministic (#4755)",
          "timestamp": "2026-10-21T09:47:36-07:00",
          "tree_id": "4d3f49bfa6b199b1b1d7042479d5622ef6643a5c",
          "url": "https://github.com/dimensionalOS/dimos/commit/0978e4b7373e535b08e59d625eeb16a05d1484ed"
        },
        "date": 1792601424078,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1593.637,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 373,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.109,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.758,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.169,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "31eeb6baa30257fda5fc92cebccd99983feb6348",
          "message": "feat: pull carrot back to farthest point robot fits (#4741)\n\nCo-authored-by: autofix-ci[bot] <114827586+autofix-ci[bot]@users.noreply.github.com>",
          "timestamp": "2026-10-21T21:10:37+03:00",
          "tree_id": "e1bcc88ce8066da4f5cd270ca88d128c277897f9",
          "url": "https://github.com/dimensionalOS/dimos/commit/31eeb6baa30257fda5fc92cebccd99983feb6348"
        },
        "date": 1792606403772,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1627.57,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.827,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.727,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.254,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "eb73dadee54e4a958ea3740b3af117b01b1aaa10",
          "message": "perception: feat/split localize stages (#4582)\n\nCo-authored-by: bogwi <bogdan@dimensional.org>",
          "timestamp": "2026-10-22T03:27:03+09:00",
          "tree_id": "ecab3d6e2877375c9c776ce453d072d020bfd81e",
          "url": "https://github.com/dimensionalOS/dimos/commit/eb73dadee54e4a958ea3740b3af117b01b1aaa10"
        },
        "date": 1792607395922,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1595.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.7,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.52,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.15,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ecc2b8f62f34d39a109643b6fcaff97ef9c733ab",
          "message": "fix: teleop cancels current nav goal (#4748)",
          "timestamp": "2026-10-21T18:32:55Z",
          "tree_id": "7892e6f0d69d3fc0c02f51d9226ee70966c039a2",
          "url": "https://github.com/dimensionalOS/dimos/commit/ecc2b8f62f34d39a109643b6fcaff97ef9c733ab"
        },
        "date": 1792607744196,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.996,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.387,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3972c006d6fe78ff16cc0bc6883968c4b8dd0723",
          "message": "feat(go2tool): connect-wifi --ap puts the go2 in hotspot mode (#4634)",
          "timestamp": "2026-10-21T19:47:08Z",
          "tree_id": "04d6233caaaad7e02e8ea9d539889f95b77e1133",
          "url": "https://github.com/dimensionalOS/dimos/commit/3972c006d6fe78ff16cc0bc6883968c4b8dd0723"
        },
        "date": 1792612210760,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1609.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.961,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0ed5c312203f970940a784215102ab4fa1db29e1",
          "message": "go2 permanent map: unitree-go2-map (#4754)",
          "timestamp": "2026-10-21T20:03:21Z",
          "tree_id": "04e7ba06be51c74661b9cf5ce00921ba139c09f5",
          "url": "https://github.com/dimensionalOS/dimos/commit/0ed5c312203f970940a784215102ab4fa1db29e1"
        },
        "date": 1792613192579,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1741.348,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.759,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.906,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "70c922824d17761f6f79826e2ecde02ca4586734",
          "message": "fix: local planner shortcuts respect turns in the search (#4745)",
          "timestamp": "2026-10-21T20:14:30Z",
          "tree_id": "c3bde9a7608dbb92fe6ba23228a7f9a0596a59e6",
          "url": "https://github.com/dimensionalOS/dimos/commit/70c922824d17761f6f79826e2ecde02ca4586734"
        },
        "date": 1792613856148,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1810.359,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.198,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.487,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a16e03af7a2303c31eb5f458d2b0624029f284ce",
          "message": "fix(web): retry ephemeral port when QUIC UDP is occupied (#4758)",
          "timestamp": "2026-10-21T20:55:29Z",
          "tree_id": "a9886241ba2885d8e06ad3801c2602a4b9171a87",
          "url": "https://github.com/dimensionalOS/dimos/commit/a16e03af7a2303c31eb5f458d2b0624029f284ce"
        },
        "date": 1792616295283,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1832.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.039,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.742,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "82cd02af3b33b7dc7db3ba319523101bf5d6c01e",
          "message": "Dimos distributed tooling, example blueprint (#4457)\n\nCo-authored-by: Paul Nechifor <paul@nechifor.net>",
          "timestamp": "2026-10-22T06:51:46+03:00",
          "tree_id": "880e21d548562586ee4a78c521c7aaf02477c250",
          "url": "https://github.com/dimensionalOS/dimos/commit/82cd02af3b33b7dc7db3ba319523101bf5d6c01e"
        },
        "date": 1792641273099,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1824.523,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.385,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.486,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1a7d1e54c6508af9972c44809e61b22671034d4d",
          "message": "Desktop: dimos gateway (#4429)",
          "timestamp": "2026-10-22T17:42:58+03:00",
          "tree_id": "cfa831c5a4203492129807e6d8fdac84d23beed0",
          "url": "https://github.com/dimensionalOS/dimos/commit/1a7d1e54c6508af9972c44809e61b22671034d4d"
        },
        "date": 1792680352957,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1717.98,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 361,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.367,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.727,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.328,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c240b5467f3b2217c1c7a038d3d9d7ee4c35c3b8",
          "message": "v0.0.15 (#4771)",
          "timestamp": "2026-10-22T19:58:06+03:00",
          "tree_id": "edc16ab019bd1a3d2775b1cfe994bd8f11a64d64",
          "url": "https://github.com/dimensionalOS/dimos/commit/c240b5467f3b2217c1c7a038d3d9d7ee4c35c3b8"
        },
        "date": 1792688461082,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1827.461,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2726.743,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.809,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.831,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d667fad25b05e8072e4a17e2d3bfd826f3336602",
          "message": "refactor(control): joints and limits come only from the robot model (#4760)",
          "timestamp": "2026-10-22T19:07:25Z",
          "tree_id": "d298cb772efbf1f916b5b6c9edf79b437c98726a",
          "url": "https://github.com/dimensionalOS/dimos/commit/d667fad25b05e8072e4a17e2d3bfd826f3336602"
        },
        "date": 1792696224958,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1718.254,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.13,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.009,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23d3ef2dbd3b4c68f23eb625889af10cae77fca2",
          "message": "refactor(control): a connection module wires its own ports (#4761)",
          "timestamp": "2026-10-22T21:37:03Z",
          "tree_id": "41641ea93dc6c20d0ba117db13d16faad4867576",
          "url": "https://github.com/dimensionalOS/dimos/commit/23d3ef2dbd3b4c68f23eb625889af10cae77fca2"
        },
        "date": 1792705194845,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1783.574,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.916,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 138.022,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "42ca50dfef92b6e8192bb9169b4bb03895ddc854",
          "message": "fix: ignore generated flake locks in native module test (#4780)",
          "timestamp": "2026-10-22T21:55:40Z",
          "tree_id": "3b4768d6200de4d2ab94176753c5c3747b8eb95f",
          "url": "https://github.com/dimensionalOS/dimos/commit/42ca50dfef92b6e8192bb9169b4bb03895ddc854"
        },
        "date": 1792706320246,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1779.824,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.015,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.812,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0ea05f66c968ab844665c7d5733eea1f360b9776",
          "message": "feat(web): add 3D map navigation goals and path overlay (#4637)",
          "timestamp": "2026-10-22T22:59:28Z",
          "tree_id": "bee2323acd02c23f7028608c7a5edb66b6878601",
          "url": "https://github.com/dimensionalOS/dimos/commit/0ea05f66c968ab844665c7d5733eea1f360b9776"
        },
        "date": 1792710139903,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1699.445,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 357,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.914,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.469,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.571,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e4b62ac88c47453110f6ca6038230ffc978d8894",
          "message": "fix(ci): drop the setup-deno cache from the lint and tests jobs (#4792)",
          "timestamp": "2026-10-23T10:58:13-07:00",
          "tree_id": "a64caf9baf8b59dbe85b9811315e5f3d4b83a77e",
          "url": "https://github.com/dimensionalOS/dimos/commit/e4b62ac88c47453110f6ca6038230ffc978d8894"
        },
        "date": 1792778477172,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1840.531,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.976,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.761,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f9c5a5cf7c79a5d9654f7950e0ca267533b8c4c1",
          "message": "Fix navigation CostMapper configuration validation (#4787)",
          "timestamp": "2026-10-24T02:38:06+03:00",
          "tree_id": "399320a2e300d8f53083b0b4521947307dd1d59a",
          "url": "https://github.com/dimensionalOS/dimos/commit/f9c5a5cf7c79a5d9654f7950e0ca267533b8c4c1"
        },
        "date": 1792798857800,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1795.23,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.945,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.801,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c3f0addcb6b4c9397fa3dd327c05da0967b9747a",
          "message": "Fix SQLite errors when stopping replay early (#4810)",
          "timestamp": "2026-10-24T03:46:51+03:00",
          "tree_id": "057300b288e343d3d65f573bb4f5a5ff835d2a37",
          "url": "https://github.com/dimensionalOS/dimos/commit/c3f0addcb6b4c9397fa3dd327c05da0967b9747a"
        },
        "date": 1792802991398,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1762.234,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.794,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.654,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1b5e5692da4f66d43f57302deb71a083ee797eb8",
          "message": "fix(control): make frame pose rate test deterministic (#4811)",
          "timestamp": "2026-10-24T05:33:54+03:00",
          "tree_id": "6f71f1079d6fb565815403c0a8aeb67062fd4df8",
          "url": "https://github.com/dimensionalOS/dimos/commit/1b5e5692da4f66d43f57302deb71a083ee797eb8"
        },
        "date": 1792809411511,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1863.574,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.289,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.68,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.683,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "alxbouchard@gmail.com",
            "name": "alxbouchard",
            "username": "alxbouchard"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5daef81a62353c513e8253912bc4a5df37199423",
          "message": "fix(reactive): deliver backpressure messages on one resident worker thread (#4796)",
          "timestamp": "2026-10-24T06:04:27+03:00",
          "tree_id": "768937fc79d1229bb8e9cf23369bff7f0b1b23ac",
          "url": "https://github.com/dimensionalOS/dimos/commit/5daef81a62353c513e8253912bc4a5df37199423"
        },
        "date": 1792811254291,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1686.902,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 361,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2732.025,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.239,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d9793f176194d91d12110a47636217b1eb61112c",
          "message": "Configure Google docstrings with Ruff and MkDocs (#4798)",
          "timestamp": "2026-10-24T06:05:51Z",
          "tree_id": "f9cee1d23063ce0dab9a5dfd279afb4c35b942f2",
          "url": "https://github.com/dimensionalOS/dimos/commit/d9793f176194d91d12110a47636217b1eb61112c"
        },
        "date": 1792822142264,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1711.082,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.994,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.511,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bedd25bd111ecec38475c31282b7b39618243a81",
          "message": "fix(openyam): wire the planner keyboard twist input (#4794)",
          "timestamp": "2026-10-25T06:06:26+02:00",
          "tree_id": "8d2e358350f1eea211768fdec8de3dc4bc920afd",
          "url": "https://github.com/dimensionalOS/dimos/commit/bedd25bd111ecec38475c31282b7b39618243a81"
        },
        "date": 1792901361030,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1715.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 376,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.384,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.518,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "miguel.villafloran@gmail.com",
            "name": "Miguel Villa Floran",
            "username": "Kaweees"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1adeec63d3633ad8cd8a056708da315071c2ec57",
          "message": "feat(dual-openyam): engage each arm from its own WebXR grip (#4631)",
          "timestamp": "2026-10-25T07:59:00Z",
          "tree_id": "a20312313ff463b46fa6b103ce39a8aa3c73bc68",
          "url": "https://github.com/dimensionalOS/dimos/commit/1adeec63d3633ad8cd8a056708da315071c2ec57"
        },
        "date": 1792915318203,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1692.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.092,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.05,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "miguel.villafloran@gmail.com",
            "name": "Miguel Villa Floran",
            "username": "Kaweees"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6de7182e693f2c7f5118fe2197675fef81270fbc",
          "message": "feat(openyam): reset arms to the zero joint pose from a Quest button (#4633)",
          "timestamp": "2026-10-25T08:22:25Z",
          "tree_id": "8f6609e882ea8b0846bab66316f462a53da88f8d",
          "url": "https://github.com/dimensionalOS/dimos/commit/6de7182e693f2c7f5118fe2197675fef81270fbc"
        },
        "date": 1792916712569,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1692.82,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 373,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.305,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.493,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "113392023+KrishnaH96@users.noreply.github.com",
            "name": "Krishna_Hundekari",
            "username": "KrishnaH96"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3e39e54148be0a927253165b863aec0859839c88",
          "message": "fix(dataprep): make the example config build the xarm7 WebXR recording (#4830)\n\nCo-authored-by: Krishna Hundekari <krishna@dimensional.org>",
          "timestamp": "2026-10-25T20:05:44-07:00",
          "tree_id": "960e54234114f76541775e7b3298b485ed7c4d43",
          "url": "https://github.com/dimensionalOS/dimos/commit/3e39e54148be0a927253165b863aec0859839c88"
        },
        "date": 1792984129929,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1653.609,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.319,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.777,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 134.955,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "99c5afa6aabb3b3d0643d4fec8ef7700490882f9",
          "message": "fix(reactive): tie backpressure workers to subscription lifetime (#4819)",
          "timestamp": "2026-10-26T18:01:02+02:00",
          "tree_id": "a8bf63f7ee7a2a003c593482b32f8d623ed03b7c",
          "url": "https://github.com/dimensionalOS/dimos/commit/99c5afa6aabb3b3d0643d4fec8ef7700490882f9"
        },
        "date": 1793030650092,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1700.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.761,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.75,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.99,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aafa6107db181413d7b59b00a8a7b55cca1334d7",
          "message": "Fix SDK peeks for externally published odometry (#4816)",
          "timestamp": "2026-10-26T18:04:42+02:00",
          "tree_id": "3b8a65adfddb2f015ddaaa5f057a4a9322a83eb3",
          "url": "https://github.com/dimensionalOS/dimos/commit/aafa6107db181413d7b59b00a8a7b55cca1334d7"
        },
        "date": 1793030866963,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1699.094,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.053,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.5,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.512,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f9530bbee47d7bb128c619bdb7dc3bb87bd1fc54",
          "message": "Fix shutdown hangs and cleanup tracebacks (#4820)",
          "timestamp": "2026-10-26T20:03:26+02:00",
          "tree_id": "fa2678556eb5cf72222e95156cb9444e9428d5bf",
          "url": "https://github.com/dimensionalOS/dimos/commit/f9530bbee47d7bb128c619bdb7dc3bb87bd1fc54"
        },
        "date": 1793037995044,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1702.926,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2733.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.531,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.013,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f41f9e384d9448e52faf0fca18cbeb8f60e27abf",
          "message": "fix(evals): exit 1 when a case errors or nothing is selected (#4824)",
          "timestamp": "2026-10-26T18:53:49Z",
          "tree_id": "f23669e305876d19a83a5250f9fa9ac0ca59d8fc",
          "url": "https://github.com/dimensionalOS/dimos/commit/f41f9e384d9448e52faf0fca18cbeb8f60e27abf"
        },
        "date": 1793041008748,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1687.758,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.34,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.509,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3a1c8931f46e243320047dc6286950dc89fdc436",
          "message": "fix(agents): show tool image artefacts as tool output, not typed input (#4822)",
          "timestamp": "2026-10-26T22:33:02+02:00",
          "tree_id": "9a2554bca08531ee626bab7c98867cd571585400",
          "url": "https://github.com/dimensionalOS/dimos/commit/3a1c8931f46e243320047dc6286950dc89fdc436"
        },
        "date": 1793046954578,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1703.516,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.89,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.508,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "699c51b259d83d496f61e11032bc63c130c029df",
          "message": "R1: v4l2: Jetson wrapper supplies libEGL and libstdc++ so the head cameras start (#4616)",
          "timestamp": "2026-10-26T13:52:12-07:00",
          "tree_id": "94c6c93d11f31b0f1071141d18973451faf906f8",
          "url": "https://github.com/dimensionalOS/dimos/commit/699c51b259d83d496f61e11032bc63c130c029df"
        },
        "date": 1793048108120,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1695.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.636,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.668,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.554,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cc9666a245f6807738e7f8adb24151c5f4d1294a",
          "message": "UrdfManager: URDF self-masking and joint tf (#4612)",
          "timestamp": "2026-10-26T20:57:06Z",
          "tree_id": "717f7c8cf19156552094f9f0272ffd944f704101",
          "url": "https://github.com/dimensionalOS/dimos/commit/cc9666a245f6807738e7f8adb24151c5f4d1294a"
        },
        "date": 1793048385729,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1690.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 374,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.938,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.539,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.54,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f3103a5d4750cb3dca7c9fdb712acc1273b2017f",
          "message": "feat(manipulation): check grasps against the gripper's allowed contact surfaces (#4526)",
          "timestamp": "2026-10-26T16:07:16-07:00",
          "tree_id": "c5bf1075afe01fb55990e21325c7bf92c2398f2d",
          "url": "https://github.com/dimensionalOS/dimos/commit/f3103a5d4750cb3dca7c9fdb712acc1273b2017f"
        },
        "date": 1793056217627,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1727.062,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.978,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.56,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cd5d23ebef1d942ca863656425172fb6568585f9",
          "message": "feat(manipulation): heuristic grasps aim the closed finger pads, not the tool tip (#4527)",
          "timestamp": "2026-10-27T04:07:16Z",
          "tree_id": "c53a6e1010f6927f6bfaa4dc83f9bcd4c11cb3b1",
          "url": "https://github.com/dimensionalOS/dimos/commit/cd5d23ebef1d942ca863656425172fb6568585f9"
        },
        "date": 1793074199367,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1728.086,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.046,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.758,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.736,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61fe69d69843d67d7fdc610449b4b0874706125e",
          "message": "feat(control): coordinator drives connection modules over streams (#4425)",
          "timestamp": "2026-10-27T04:14:15Z",
          "tree_id": "28553b3a16a2ae9e968a60684c795d1493eb2e47",
          "url": "https://github.com/dimensionalOS/dimos/commit/61fe69d69843d67d7fdc610449b4b0874706125e"
        },
        "date": 1793074619402,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1705.809,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 358,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.981,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.711,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.592,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "73aa3269ed94c4a7028bd7c56eef9dd385186482",
          "message": "feat(control): estop and status through connection modules (#4426)",
          "timestamp": "2026-10-27T05:10:27Z",
          "tree_id": "b42529fa91da3a53ff031a24c930b86ffbc33a81",
          "url": "https://github.com/dimensionalOS/dimos/commit/73aa3269ed94c4a7028bd7c56eef9dd385186482"
        },
        "date": 1793077993099,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1719.289,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 377,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2733.216,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.75,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.571,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8c414bf6875d553c8e18707016a6e231746d362a",
          "message": "feat(manipulation): contact check rejects grasps that close on nothing (#4528)",
          "timestamp": "2026-10-27T05:12:27Z",
          "tree_id": "228c7afc90cc75a6fb92da1518ef250b2c2d9640",
          "url": "https://github.com/dimensionalOS/dimos/commit/8c414bf6875d553c8e18707016a6e231746d362a"
        },
        "date": 1793078120892,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1729.656,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.191,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.803,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2db18582da0e19ea5066f859cdd0b77f37e8ebd6",
          "message": "Add manual checks. (#4764)",
          "timestamp": "2026-10-27T07:15:39+02:00",
          "tree_id": "731bd43644352e5c09f54f14d0c20b79e7241c1a",
          "url": "https://github.com/dimensionalOS/dimos/commit/2db18582da0e19ea5066f859cdd0b77f37e8ebd6"
        },
        "date": 1793078305825,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1686.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.165,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bc6ec06c19e6550813943b0449702fc7a093a4f5",
          "message": "feat(xarm): GraspGenX returns all its samples so the contact check sees every one (#4529)",
          "timestamp": "2026-10-27T06:20:29Z",
          "tree_id": "e36d341f7a5da89ccff04436a369b08b451e37be",
          "url": "https://github.com/dimensionalOS/dimos/commit/bc6ec06c19e6550813943b0449702fc7a093a4f5"
        },
        "date": 1793082205972,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1695.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.797,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.959,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "ari@dimensionalos.com",
            "name": "aa-MONTS",
            "username": "aa-MONTS"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ff18e9094026372ab5466adfddec75b4420b4be9",
          "message": "feat(manipulation): add Zero preset to Viser planner (#4632)\n\nCo-authored-by: aa-monts <302640929+aa-MONTS@users.noreply.github.com>",
          "timestamp": "2026-10-27T00:33:28-07:00",
          "tree_id": "a3115bd78119edf32ca1715e9c392aacc0b1f8f9",
          "url": "https://github.com/dimensionalOS/dimos/commit/ff18e9094026372ab5466adfddec75b4420b4be9"
        },
        "date": 1793086581568,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1688.344,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.169,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.781,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.874,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4143859+dimos-release-bot[bot]@users.noreply.github.com",
            "name": "dimos-release-bot[bot]"
          },
          "committer": {
            "email": "4143859+dimos-release-bot[bot]@users.noreply.github.com",
            "name": "dimos-release-bot[bot]"
          },
          "distinct": true,
          "id": "3493be2adad052e0d7fcd4f2340fc7d91eedff62",
          "message": "Merge release/0.0.15 back to main",
          "timestamp": "2026-10-27T16:41:18Z",
          "tree_id": "a3115bd78119edf32ca1715e9c392aacc0b1f8f9",
          "url": "https://github.com/dimensionalOS/dimos/commit/3493be2adad052e0d7fcd4f2340fc7d91eedff62"
        },
        "date": 1793119479222,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1715.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.397,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.57,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.547,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "76ffe6ead7be4387b91c3d89c795faf51089d31f",
          "message": "feat(manipulation): contact check can slide grasps to the nearest depth that passes (#4530)",
          "timestamp": "2026-10-27T12:10:32-07:00",
          "tree_id": "5d4df7062ccc8448df101db5c1fab595dc36326d",
          "url": "https://github.com/dimensionalOS/dimos/commit/76ffe6ead7be4387b91c3d89c795faf51089d31f"
        },
        "date": 1793128418019,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1715.621,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 371,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.296,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.573,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6b3643de0db46b0e644f546317fbf387c0c5d15a",
          "message": "fix(gateway): relaunch waits for the stopped run to be reaped (#4854)\n\nCo-authored-by: Jeff Hykin <jeff.hykin@gmail.com>",
          "timestamp": "2026-10-28T00:26:46Z",
          "tree_id": "3fc0aa6cf96d97c3c83f6f69b3076f890d521cd5",
          "url": "https://github.com/dimensionalOS/dimos/commit/6b3643de0db46b0e644f546317fbf387c0c5d15a"
        },
        "date": 1793147382788,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1709.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.071,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.576,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cdfeb524088c0af287d00131d313a9378bb34161",
          "message": "build(audio): drop the ffmpeg system dependency (#4848)",
          "timestamp": "2026-10-28T04:55:07+02:00",
          "tree_id": "d67857732a1a578bf2fcb18a95f4433197b10687",
          "url": "https://github.com/dimensionalOS/dimos/commit/cdfeb524088c0af287d00131d313a9378bb34161"
        },
        "date": 1793156279927,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1695.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.762,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.163,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "68f6c98da1a2ff5093c774cf19d3bae0b7e4ef96",
          "message": "chore: remove unused stream code (#4883)",
          "timestamp": "2026-10-27T20:47:48-07:00",
          "tree_id": "a53fe2969789454d3a7a07a8577c062d4ba0402d",
          "url": "https://github.com/dimensionalOS/dimos/commit/68f6c98da1a2ff5093c774cf19d3bae0b7e4ef96"
        },
        "date": 1793159443531,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1697.848,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.049,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.523,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "65bcf689e0936e242dca4b01397264683500ef14",
          "message": "Fix shutdown race condition (#4769)",
          "timestamp": "2026-10-28T06:24:33+02:00",
          "tree_id": "fb21054cefe5670bc3252425424144db242a6b60",
          "url": "https://github.com/dimensionalOS/dimos/commit/65bcf689e0936e242dca4b01397264683500ef14"
        },
        "date": 1793161638987,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1724.742,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.955,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.5,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c18e2fbfd3b591098962c0007f2cddf58f7c68d9",
          "message": "fix(ci): skip branch name check for fork PRs (#4884)",
          "timestamp": "2026-10-28T06:25:03+02:00",
          "tree_id": "e28a66340c3e5ae419d3b4482b1365ba260da9af",
          "url": "https://github.com/dimensionalOS/dimos/commit/c18e2fbfd3b591098962c0007f2cddf58f7c68d9"
        },
        "date": 1793161660985,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1737.852,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.091,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.388,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "777b76ba3c7c463e548e5e97421ff653bd069f0f",
          "message": "fix(agents): navigate_with_text reports when the planner stops (#4880)",
          "timestamp": "2026-10-28T00:11:13-07:00",
          "tree_id": "453ae05062461013a1c370cc09686a1ee4b5d6ca",
          "url": "https://github.com/dimensionalOS/dimos/commit/777b76ba3c7c463e548e5e97421ff653bd069f0f"
        },
        "date": 1793171658909,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1699.379,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.911,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.75,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.044,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1d71f0137cda31df840a1b3f35abaaf4590a63e4",
          "message": "refactor(evals): rename the Agent base to EvalSubject (#4432)",
          "timestamp": "2026-10-28T03:44:51-07:00",
          "tree_id": "0809dce7ee100c5fe08f970e7a63038cb11180a4",
          "url": "https://github.com/dimensionalOS/dimos/commit/1d71f0137cda31df840a1b3f35abaaf4590a63e4"
        },
        "date": 1793184482956,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1696.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.745,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.925,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3bec585f30943ee7cc5d38aeffb4cad2cdb035c3",
          "message": "feat: go2 sim eval framework (#4502)",
          "timestamp": "2026-10-28T18:05:03+02:00",
          "tree_id": "dc08363150e4aa332bf76d2cf928512c84da6b81",
          "url": "https://github.com/dimensionalOS/dimos/commit/3bec585f30943ee7cc5d38aeffb4cad2cdb035c3"
        },
        "date": 1793203689765,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1728.82,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.818,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.997,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "78c71f33f6cea71564aefe060b0e5cae94fe6986",
          "message": "Fix Zenoh hanging after a failed test (#4856)\n\nCo-authored-by: Jeff Hykin <jeff.hykin@gmail.com>",
          "timestamp": "2026-10-28T19:03:07+02:00",
          "tree_id": "e08d6f44d05941f6f9e93351dfaaffeeece232fa",
          "url": "https://github.com/dimensionalOS/dimos/commit/78c71f33f6cea71564aefe060b0e5cae94fe6986"
        },
        "date": 1793207170225,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1709.215,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 371,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.02,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.793,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e57083eb9e872ee3b7c9a3ecdcb1ddac22312e09",
          "message": "test(e2e): go-to-bed accepts any spot within 3 m of the bed (#4886)",
          "timestamp": "2026-10-28T17:33:00Z",
          "tree_id": "939901ce2d34e0b585357cf45d11700eca4dbcb4",
          "url": "https://github.com/dimensionalOS/dimos/commit/e57083eb9e872ee3b7c9a3ecdcb1ddac22312e09"
        },
        "date": 1793208953126,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1682.891,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.139,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.03,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "094d5d033b9a5df010954b5f0254db2e55abd72e",
          "message": "chore: remove Python trajectory follower (#4850)\n\nCo-authored-by: autofix-ci[bot] <114827586+autofix-ci[bot]@users.noreply.github.com>",
          "timestamp": "2026-10-28T17:56:22Z",
          "tree_id": "9ea6f8e0481f3095678fa3cb8eb37c4a91caf101",
          "url": "https://github.com/dimensionalOS/dimos/commit/094d5d033b9a5df010954b5f0254db2e55abd72e"
        },
        "date": 1793210382541,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1760.785,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.09,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.587,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "ari@dimensionalos.com",
            "name": "aa-MONTS",
            "username": "aa-MONTS"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d793e80a1732fe84b51e95cebbf046c4efa05d8d",
          "message": "feat(data): print Console link after upload (#4505)\n\nCo-authored-by: Ari Ameri <302640929+aa-MONTS@users.noreply.github.com>\nCo-authored-by: stash <pomichterstash@gmail.com>",
          "timestamp": "2026-10-28T11:58:37-07:00",
          "tree_id": "f027e80249decffcc472d0c898eb6334bf2f910a",
          "url": "https://github.com/dimensionalOS/dimos/commit/d793e80a1732fe84b51e95cebbf046c4efa05d8d"
        },
        "date": 1793214085579,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1716.062,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.058,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.102,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7532a4114cc178f2480d4e44cd956bfd50467bce",
          "message": "test(msgs): no new messages in dimos-lcm's ROS packages (#4832)",
          "timestamp": "2026-10-28T22:33:49Z",
          "tree_id": "95e7091e8273820cd2cfc7d72dd949840a394055",
          "url": "https://github.com/dimensionalOS/dimos/commit/7532a4114cc178f2480d4e44cd956bfd50467bce"
        },
        "date": 1793226997177,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1735.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.679,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.634,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cea65150017b8019f4e64bc849e592233036fc37",
          "message": "feat: go2 unit sim environments (#4548)",
          "timestamp": "2026-10-28T15:37:12-07:00",
          "tree_id": "ed3cb93ba4d5172d4b932ed46590120b09a48e69",
          "url": "https://github.com/dimensionalOS/dimos/commit/cea65150017b8019f4e64bc849e592233036fc37"
        },
        "date": 1793227193065,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1741.09,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.537,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.244,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e3f7c0aba09354ce5971b2b74ee47ec3f2b3c664",
          "message": "fix(sim): publish tracked-body ground truth on body_poses, not tf (#4871)",
          "timestamp": "2026-10-28T15:47:04-07:00",
          "tree_id": "43adc6cf22949aa2fe2b497fff9fb5492deefeaa",
          "url": "https://github.com/dimensionalOS/dimos/commit/e3f7c0aba09354ce5971b2b74ee47ec3f2b3c664"
        },
        "date": 1793227787246,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1732.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.327,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.653,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c7c00f5e4089f312c563c651cf2545a189089385",
          "message": "feat(agent): python runtime for agent harness (#4814)\n\nCo-authored-by: stash <pomichterstash@gmail.com>",
          "timestamp": "2026-10-28T16:01:48-07:00",
          "tree_id": "98e4cba48bca08c03b3cc39ce7194d8b93b32093",
          "url": "https://github.com/dimensionalOS/dimos/commit/c7c00f5e4089f312c563c651cf2545a189089385"
        },
        "date": 1793228687017,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1674.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.366,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.116,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1216a557531bc39632c181df843d96c2d667b52c",
          "message": "feat(manipulation): Arm.reset() in the SDK (#4866)",
          "timestamp": "2026-10-28T18:06:59-07:00",
          "tree_id": "df1e1ba64f348b7d4141e167b7545710ddd9913d",
          "url": "https://github.com/dimensionalOS/dimos/commit/1216a557531bc39632c181df843d96c2d667b52c"
        },
        "date": 1793236186733,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1705.691,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.831,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.978,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6ccf78177805ea82e869d849d65fc7ac97095a1c",
          "message": "fix: use mj ray and remove some unecessary tests (#4899)",
          "timestamp": "2026-10-29T06:06:39+02:00",
          "tree_id": "0f26eadd143293a1d9766e0bf7664158944b5f7f",
          "url": "https://github.com/dimensionalOS/dimos/commit/6ccf78177805ea82e869d849d65fc7ac97095a1c"
        },
        "date": 1793246969277,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1716.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 371,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.433,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 135.976,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d39375df29ccb1c76c84b98e481a085b4cf9eb70",
          "message": "feat(dataprep): export LeRobot datasets in the isolated runtime (#4395)",
          "timestamp": "2026-10-28T23:55:16-07:00",
          "tree_id": "44e25b879c569bcc863ec0e5366ceac4d42fb060",
          "url": "https://github.com/dimensionalOS/dimos/commit/d39375df29ccb1c76c84b98e481a085b4cf9eb70"
        },
        "date": 1793257090928,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1711.961,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.028,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.73,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.565,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "017231c4819c148982533dcdc4304cf4d8714d68",
          "message": "Alert on failing runners (#4806)",
          "timestamp": "2026-10-29T16:55:53Z",
          "tree_id": "758ad55af177cc8b7c8d3ac4b1e54c36e912c18e",
          "url": "https://github.com/dimensionalOS/dimos/commit/017231c4819c148982533dcdc4304cf4d8714d68"
        },
        "date": 1793293131940,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1732.281,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 363,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.486,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.045,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4e97a0313495c8673e9e6e9233c5d45cce63bc13",
          "message": "feat(agents): add the pi-durable agent harness (#4837)\n\nCo-authored-by: Sam Bull <git@sambull.org>",
          "timestamp": "2026-10-29T10:51:18-07:00",
          "tree_id": "e8f4510d8431d0348b6eeb5c92731b80badf02a9",
          "url": "https://github.com/dimensionalOS/dimos/commit/4e97a0313495c8673e9e6e9233c5d45cce63bc13"
        },
        "date": 1793296443740,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1728.672,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.006,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.68,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "732c87aa04c8b515e67de9e8f4a6dcfa77b5263b",
          "message": "feat(agents): add the agent gateway (#4839)\n\nCo-authored-by: stash <pomichterstash@gmail.com>",
          "timestamp": "2026-10-29T14:25:51-07:00",
          "tree_id": "b3fe4577b05fd83ef339f74a9fe3428fc8df837b",
          "url": "https://github.com/dimensionalOS/dimos/commit/732c87aa04c8b515e67de9e8f4a6dcfa77b5263b"
        },
        "date": 1793309326398,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1701.938,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 371,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.365,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.565,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "02bf8a5e61909f3e5fabea485270f96ef2f0da7d",
          "message": "feat(agents): add the agent client SDK (#4842)",
          "timestamp": "2026-10-29T15:13:05-07:00",
          "tree_id": "72124d5c5adab5db112fd1c860162adca89a888c",
          "url": "https://github.com/dimensionalOS/dimos/commit/02bf8a5e61909f3e5fabea485270f96ef2f0da7d"
        },
        "date": 1793312163220,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1746.059,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.262,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.703,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.508,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": false,
          "id": "1792eda890c88c5a04279216b2f6d1cd42abd786",
          "message": "feat(agents): add the agent CLI (#4843)",
          "timestamp": "2026-10-29T15:29:39-07:00",
          "tree_id": "9a92b87b026e563324ae9aa81768115cd732960c",
          "url": "https://github.com/dimensionalOS/dimos/commit/1792eda890c88c5a04279216b2f6d1cd42abd786"
        },
        "date": 1793313148931,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1725.449,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.748,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.52,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.001,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ebe17bd8d065b1c5703a2827aa69bbb388bddffe",
          "message": "feat(agents): add the agent terminal UI (#4840)",
          "timestamp": "2026-10-29T15:34:39-07:00",
          "tree_id": "0dc9c521ab9fd4d89f2faa8fac31cbc4a2a9ba2a",
          "url": "https://github.com/dimensionalOS/dimos/commit/ebe17bd8d065b1c5703a2827aa69bbb388bddffe"
        },
        "date": 1793313446345,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1730.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.98,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.785,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.843,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": false,
          "id": "74661f5ec10f4e2011c49528d4c2d240f269d0ea",
          "message": "feat(evals): add agent adapter for new DimOS agent (#4874)",
          "timestamp": "2026-10-29T17:06:25-07:00",
          "tree_id": "be4d7110cceba0ed8d5e9ec9d465cd487fdc338d",
          "url": "https://github.com/dimensionalOS/dimos/commit/74661f5ec10f4e2011c49528d4c2d240f269d0ea"
        },
        "date": 1793318943916,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1736.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 375,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.043,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.977,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6b57dae425c17f41f9db1f0df7862c62cc81ab8f",
          "message": "ray tracing: graze sparing only protects voxels hit within graze_recent_frames (#4877)",
          "timestamp": "2026-10-30T00:47:09Z",
          "tree_id": "d17f97a22bb36f3f369e93031fb0fef00eb8efa8",
          "url": "https://github.com/dimensionalOS/dimos/commit/6b57dae425c17f41f9db1f0df7862c62cc81ab8f"
        },
        "date": 1793321393237,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1754.344,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.055,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.711,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.3,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ada8b1f16928aa38a6c1ece49cac5bf30b6bb985",
          "message": "feat(agent): expand control of backgrounded tools (#4890)",
          "timestamp": "2026-10-29T21:03:06-07:00",
          "tree_id": "72e6697b9a949a24c6fb9fe0e3cf29be3d49df5f",
          "url": "https://github.com/dimensionalOS/dimos/commit/ada8b1f16928aa38a6c1ece49cac5bf30b6bb985"
        },
        "date": 1793333153615,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1746.422,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.993,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.555,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "52c95dcf7c751acdbecbcdc3804ece55c82ee631",
          "message": "feat(evals): run eval suites on real hardware (#4896)",
          "timestamp": "2026-10-30T04:23:14Z",
          "tree_id": "6d05399bf2936888f501c8a08d1a95746f701fde",
          "url": "https://github.com/dimensionalOS/dimos/commit/52c95dcf7c751acdbecbcdc3804ece55c82ee631"
        },
        "date": 1793334355910,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1721.434,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.563,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.469,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.481,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "64b2d8aa6bcfdc7c252b7e8c412e7671947dd157",
          "message": "docs(agents): move agent guides into docs (#4922)",
          "timestamp": "2026-10-29T22:10:01-07:00",
          "tree_id": "4cffec877b2edb366aaffa5e9683bcbf0729717f",
          "url": "https://github.com/dimensionalOS/dimos/commit/64b2d8aa6bcfdc7c252b7e8c412e7671947dd157"
        },
        "date": 1793337170560,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1755.215,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 460/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 376,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 460/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.094,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 460/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 460/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.984,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 460/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26144f7fd8424bdc33ab97dcd58e9497f2d413ee",
          "message": "Flakey: fix test_a_poll_from_a_finished_run_does_not_disturb_the_next_one (#4943)",
          "timestamp": "2026-10-30T09:12:07+02:00",
          "tree_id": "7d5c52116d1d70dfa0fe46ebb5bfe61e92a3002f",
          "url": "https://github.com/dimensionalOS/dimos/commit/26144f7fd8424bdc33ab97dcd58e9497f2d413ee"
        },
        "date": 1793344482474,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1748.285,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 373,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.909,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.028,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1267d6ed879e13aa5df51d6b4e7565a9db1ae1df",
          "message": "remove mapping/nav owners (#4944)",
          "timestamp": "2026-10-30T10:08:43+02:00",
          "tree_id": "8aff75ba910894ad304f2ef0ce6e8410990b1b22",
          "url": "https://github.com/dimensionalOS/dimos/commit/1267d6ed879e13aa5df51d6b4e7565a9db1ae1df"
        },
        "date": 1793347886829,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1754.074,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 378,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.38,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.516,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 138.062,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d79ab4d873e34f4bea9dd9b4eb4fae701a4b0b21",
          "message": "ci(tests): run e2e tests daily, retry or skip my flaky tests (#4942)",
          "timestamp": "2026-10-30T10:30:11+02:00",
          "tree_id": "428606e23668b1c34d2d5fdb5ada45ecc07e42e7",
          "url": "https://github.com/dimensionalOS/dimos/commit/d79ab4d873e34f4bea9dd9b4eb4fae701a4b0b21"
        },
        "date": 1793349186074,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1729.973,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 374,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.274,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.869,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2acf428786b776914724cc0e622cfe3d2d02d686",
          "message": "Flakey: fix test_relaunch_starts_the_last_launch_with_its_secrets (#4904)",
          "timestamp": "2026-10-30T23:10:09+02:00",
          "tree_id": "bac57f639e41a478d208322608b91d5d3b663473",
          "url": "https://github.com/dimensionalOS/dimos/commit/2acf428786b776914724cc0e622cfe3d2d02d686"
        },
        "date": 1793394780105,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1729.016,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.453,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1505f3575378746e2ff29915bdea9b6cc0f7e50b",
          "message": "Flakey: fix test_launch_runs_dimos_with_exactly_the_config_it_is_sent (#4905)",
          "timestamp": "2026-10-30T23:10:57+02:00",
          "tree_id": "4b11fc759da8e57ba9e4a9f0c5ba951ab1b43424",
          "url": "https://github.com/dimensionalOS/dimos/commit/1505f3575378746e2ff29915bdea9b6cc0f7e50b"
        },
        "date": 1793394834151,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1735.602,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 372,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.511,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.898,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 136.483,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d116330d47b6c8677a45392a013e7634372482fc",
          "message": "refactor(agents): use namespaced Node subpath imports (#4932)",
          "timestamp": "2026-10-30T17:38:48-07:00",
          "tree_id": "403a5b90823219705836061cc27555b2de269f1f",
          "url": "https://github.com/dimensionalOS/dimos/commit/d116330d47b6c8677a45392a013e7634372482fc"
        },
        "date": 1793407301844,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1744.488,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.333,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.762,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 138.117,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6e71b6defd1af1e592ea2ceb2127cfefdf17c7c9",
          "message": "refactor(manipulation): simplify module workflow state (#4891)",
          "timestamp": "2026-10-31T00:49:15Z",
          "tree_id": "aaa5c5d03e3f9e5959cf0c770d7e089cfda3d009",
          "url": "https://github.com/dimensionalOS/dimos/commit/6e71b6defd1af1e592ea2ceb2127cfefdf17c7c9"
        },
        "date": 1793407914443,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1748.445,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 372,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2728.338,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.758,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 138.09,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "aromeoes@gmail.com",
            "name": "aromeoes",
            "username": "aromeoes"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c339308739505674da59f80fa2d174b6e823a75f",
          "message": "fix(teleop): run the keyboard teleop window in a child process on macOS (#4950)d",
          "timestamp": "2026-10-31T00:49:23Z",
          "tree_id": "a22de9f8e5b90ae99c764f76e5f7fca3c941adff",
          "url": "https://github.com/dimensionalOS/dimos/commit/c339308739505674da59f80fa2d174b6e823a75f"
        },
        "date": 1793407922517,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1759.73,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2736.274,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 138.156,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "ronan.lamy@gmail.com",
            "name": "Ronan Lamy",
            "username": "rlamy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dee3c36afdb09bfd1c7ea2b163b539e57cb8771e",
          "message": "fix(agents): a failed agent turn no longer kills McpClient's thread (#4917)",
          "timestamp": "2026-10-31T02:53:38+02:00",
          "tree_id": "1f757de4663ac0b7592b479f75232ae53d75d337",
          "url": "https://github.com/dimensionalOS/dimos/commit/dee3c36afdb09bfd1c7ea2b163b539e57cb8771e"
        },
        "date": 1793408181447,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1758.609,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 369,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.839,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 137.98,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      }
    ]
  }
}