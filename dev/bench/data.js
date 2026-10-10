window.BENCHMARK_DATA = {
  "lastUpdate": 1791594768099,
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
      }
    ]
  }
}