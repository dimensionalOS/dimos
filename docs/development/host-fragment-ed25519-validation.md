# Host control Ed25519 validation

This branch authenticates every Host control RPC message before `pickle.loads`.
The client signs requests with an Ed25519 private key, and the Host holds only
its public key. The Host signs replies with a separate private key, and the
client verifies them before deserialization. Both sides reject missing keys.

Set these environment variables to base64 encodings of the raw 32-byte keys:

| Process | Variable | Key |
| --- | --- | --- |
| Client | `DIMOS_HOST_CLIENT_SIGNING_KEY` | Client private key |
| Client | `DIMOS_HOST_SERVER_VERIFY_KEY` | Host public key |
| Host | `DIMOS_HOST_CLIENT_VERIFY_KEY` | Client public key |
| Host | `DIMOS_HOST_SERVER_SIGNING_KEY` | Host private key |

The signature covers the RPC method name, message direction, timestamp, and
payload. A five-minute validity window limits stale messages. The Host epoch
and deployment identity checks govern repeated start and stop requests.

Compared with a shared HMAC secret, a Host cannot impersonate a controller
using the public key it stores. Separate credentials also make it possible to
revoke a controller without changing the Host signing key. This prototype
supports one controller public key per Host and one Host public key per client;
a fleet needs key distribution and rotation. An authorized controller can still
send arbitrary pickle content. This branch does not add encryption.

Run the local validation with:

```bash
.venv/bin/pytest -p no:rerunfailures dimos/hosted/test_rpc_auth.py
```
