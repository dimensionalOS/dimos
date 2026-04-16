import rerun as rr
import time

print("Initializing rerun...")
rr.init("test_spawn_grpc")
print("Serving gRPC on 9876...")
rr.serve_grpc(grpc_port=9876)
print("Spawning rerun...")
rr.spawn(connect=True)
print("Spawned. Sleeping for 5 seconds...")
rr.log("test", rr.TextLog("Hello Rerun with gRPC"))
time.sleep(5)
print("Done.")
