# Stream replying for testing


## Goals
- super easy binary data delivery for testing, no data stored in git but lfs
- super easy data stream storage

## Usage

you get this SensorReply class that you can use to stream previously stored rxpy streams.
(example below will auto download and extract `raw_odometry_rotate_walk` directory and load & reply a stream stored within it: 

to simply print all messages:

```python
    SensorReplay(name="raw_odometry_rotate_walk").iterate(print)
```

or to get an rxpy stream

```python
    SensorReplay(name="raw_odometry_rotate_walk").stream().pipe(...)
```

to set the rate

```python
    SensorReplay(name="raw_odometry_rotate_walk").stream(rate_hz=10).pipe(...)
```

```sh
ls tests/data/raw_odometry_rotate_walk/
 000.pickle   019.pickle   005.pickle   032.pickle   023.pickle   052.pickle   056.pickle   045.pickle   065.pickle   071.pickle   091.pickle   088.pickle   110.pickle   105.pickle   108.pickle   118.pickle   119.pickle   140.pickle   151.pickle   142.pickle   165.pickle   159.pickle   172.pickle 012.pickle   010.pickle   016.pickle   027.pickle   030.pickle   043.pickle   044.pickle   039.pickle   062.pickle   076.pickle   079.pickle   092.pickle   112.pickle   106.pickle   111.pickle   126.pickle   123.pickle   135.pickle   143.pickle  # etc etc
```

TODO - some smarter reply that actually takes into account message timestamps.
can easily be done as a stream processor.

Slightly fancier test that calculates total change in radians of some odometry stream:

```python
def test_total_rotation_travel_rxpy() -> None:
    total_rad = (
        SensorReplay(name="raw_odometry_rotate_walk", autocast=Odometry.from_msg)
        .stream()
        .pipe(
            ops.map(lambda odom: odom.rot.z),
            ops.pairwise(),  # [1,2,3,4] -> [[1,2], [2,3], [3,4]]
            ops.starmap(sub),  # [sub(1,2), sub(2,3), sub(3,4)]
            ops.reduce(add),
        )
        .run()
    )

    assert total_rad == pytest.approx(4.05, abs=0.01)
```

Lidar data reply example (200mb dir)

```python
def test_robot_mapping():
    lidar_stream = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
    map = Map(voxel_size=0.5)
    # this will block until map has consumed the whole stream
    map.consume(lidar_stream.stream()).run()

    # we investigate built map
    costmap = map.costmap

    assert costmap.grid.shape == (404, 276)
    
    # etc etc 
```


lower level acces to data is

```python

absolute_path: Path = testData("some_name") 

```
it will return a Path, that either points to a file or a dir, and you can do whatever you want with it (could be a video file, a directory with a model or some code even, etc)

# Implementation

Anything new you add to `tests/data/*` will be autodetected and you are prompted to push into our LFS store. 
It can then be pulled on-demand programatically when/if needed like described above (so that we don't make dimos repo checkout large by default)

IRL Usage examples:
    - `dimos/robot/unitree_webrtc/type/test_odometry.py`
    - `dimos/robot/unitree_webrtc/type/test_map.py`

# Message storage

if you mark your test with `tool` it will not be ran by default but is a convinient place for codebase specific tooling? not super set on this but convinient for now.
Below uses SensorStorage (as opposed to SensorReplay) to store new messages in `tests/data/*.pickle`

```python
@pytest.mark.tool
def test_store_odometry_stream() -> None:
    load_dotenv()

    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")
    robot.standup()

    storage = SensorStorage("raw_odometry_rotate_walk")
    storage.save_stream(robot.raw_odom_stream())

    shutdown = threading.Event()

    try:
        while not shutdown.wait(0.1):
            pass
    except KeyboardInterrupt:
        shutdown.set()
    finally:
        robot.liedown()
```

Anything new that you add to tests/data (can be a directory or file) will be considered "a data blob" and is automatically compressed into `tests/data/.lfs/*.tar.gz` and pushed to LFS. raw data in `tests/data/` is not commited anywhere and is .gitignored. This data is then pulled on demand when needed by others. Upload is done by a simple tool `./bin/lfs_push`. will autodetect new stuff in tests/data/ and automatically compress to `tests/data/.lfs` and push to lfs. You can then commit this new file in `.lfs/` into your PR. 

Below is not neccessary but recommended, if you install pre-commit

```sh
apt install pre-commit
cd $REPO
pre-commit install
```

This will register hooks that will auto-detect changes in `tests/data` (among other things like formatting etc) so you don't forget to add stuff on your commits

```sh
echo blabla > tests/data/bla.txt
pre-commit run

CRLF end-lines checker...............................(no files to check)Skipped
CRLF end-lines remover...............................(no files to check)Skipped
Insert license in comments...........................(no files to check)Skipped
ruff format..........................................(no files to check)Skipped
check for case conflicts.............................(no files to check)Skipped
check json...........................................(no files to check)Skipped
check toml...........................................(no files to check)Skipped
check yaml...........................................(no files to check)Skipped
format json..........................................(no files to check)Skipped
LFS data.................................................................Failed
- hook id: lfs_check
- exit code: 1

✗ New test data detected at /tests/data:
  bla.txt

Either delete or run ./bin/lfs_push
(lfs_push will compress the files into /tests/data/.lfs/, upload to LFS, and add them to your commit)
```

