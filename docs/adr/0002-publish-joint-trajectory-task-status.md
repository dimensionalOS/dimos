# Publish joint trajectory task status as a stream

The joint trajectory task owns trajectory lifecycle status and publishes it through the control coordinator's typed output stream. The coordinator transports this status without interpreting it, and manipulation consumes the stream to implement blocking execution and explicit execution waits. This keeps trajectory lifecycle out of the coordinator RPC interface while giving remote modules authoritative completion, abort, fault, and progress information.
