import rerun as rr
import time

print("Initializing rerun...")
rr.init("test_spawn")
print("Spawning rerun...")
rr.spawn()
print("Spawned. Sleeping for 5 seconds...")
rr.log("test", rr.TextLog("Hello Rerun"))
time.sleep(5)
print("Done.")
