#!/usr/bin/env python3
"""Raw-socket LCM channel-rate meter for udpm://239.255.76.67:7300 (lo)."""
import socket, struct, time, collections, sys

GRP, PORT, DUR = "239.255.76.67", 7300, float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
s.bind((GRP, PORT))
# join on loopback (LCM ttl=0 routed via lo) and default
for ifip in ("127.0.0.1", "0.0.0.0"):
    try:
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                     socket.inet_aton(GRP) + socket.inet_aton(ifip))
    except OSError:
        pass
s.settimeout(1.0)

counts = collections.Counter()
bytes_ = collections.Counter()
t0 = time.time()
while time.time() - t0 < DUR:
    try:
        pkt, _ = s.recvfrom(65535)
    except socket.timeout:
        continue
    if len(pkt) < 8:
        continue
    magic = struct.unpack(">I", pkt[:4])[0]
    if magic == 0x4C433032:  # LC02 small
        end = pkt.find(b"\0", 8)
        if end > 0:
            ch = pkt[8:end].decode(errors="replace")
            counts[ch] += 1
            bytes_[ch] += len(pkt)
    elif magic == 0x4C433033:  # LC03 fragmented
        # header: magic(4) seq(4) msg_size(4) frag_offset(4) frag_no(2) frags_in_msg(2)
        frag_no = struct.unpack(">H", pkt[16:18])[0]
        bytes_[("__frag__")] += len(pkt)
        if frag_no == 0:
            end = pkt.find(b"\0", 20)
            if end > 0:
                ch = pkt[20:end].decode(errors="replace")
                counts[ch] += 1
                bytes_[ch] += len(pkt)

el = time.time() - t0
print(f"--- LCM {GRP}:{PORT}, {el:.1f}s ---")
if not counts:
    print("NO TRAFFIC SEEN")
for ch, n in sorted(counts.items(), key=lambda kv: -kv[1]):
    print(f"{ch:55s} {n/el:7.2f} Hz  {bytes_[ch]/el/1e6:8.3f} MB/s")
