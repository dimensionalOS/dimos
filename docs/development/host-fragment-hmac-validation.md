# Host control HMAC validation

This branch requires a 32-byte or longer shared random key on the Host service
and each Host control caller. Set `DIMOS_HOST_CONTROL_KEY` to its base64 encoding
before starting either process. All Host control methods use the authenticated
RPC codec, including `describe`, `start`, `status`, and `stop`. A missing or
malformed key prevents the service or caller from starting.

The codec authenticates the raw request before `pickle.loads` runs. It also
authenticates replies before the caller unpickles them. The tag covers the RPC
method name, message direction, timestamp, and payload. A five-minute validity
window limits stale messages. The Host epoch and deployment identity checks
still govern repeated start and stop requests.

This approach is straightforward to deploy on a small trusted fleet, but all
controllers share a secret capable of authorizing Host operations. Rotating it
requires coordinated configuration. A controller with the key can still send
arbitrary Python pickle content, so the key is a strong trust boundary rather
than a safe serialization format. This branch does not add encryption.

Run the local validation with:

```bash
.venv/bin/pytest -p no:rerunfailures dimos/hosted/test_rpc_auth.py
```
