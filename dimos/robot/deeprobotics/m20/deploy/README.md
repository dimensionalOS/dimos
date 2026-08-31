# M20 deployment

Run `deeprobotics-m20-kronknav-control` on GOS (`10.21.31.104`).
Run the provisioning commands below from this `deploy` directory.

## One-time robot setup

On NOS (`10.21.31.106`), enable lidar forwarding and stop the vendor planner
that also publishes `/NAV_CMD`:

```bash
sudo install -Dm755 dimos-m20-multicast-relay-supervisor \
  /usr/local/libexec/dimos-m20-multicast-relay-supervisor
sudo install -Dm644 multicast-relay.service.d/10-dimos-network-readiness.conf \
  /etc/systemd/system/multicast-relay.service.d/10-dimos-network-readiness.conf
sudo systemctl daemon-reload
sudo systemctl enable --now multicast-relay.service
sudo systemctl disable --now planner.service
```

On GOS, let the normal `user` account read the root-owned Fast DDS objects
created by `rsdriver`:

```bash
sudo install -Dm755 dimos-m20-rsdriver-shm-permissions \
  /usr/local/libexec/dimos-m20-rsdriver-shm-permissions
sudo install -Dm644 rsdriver.service.d/10-dimos-shm-permissions.conf \
  /etc/systemd/system/rsdriver.service.d/10-dimos-shm-permissions.conf
sudo systemctl daemon-reload
sudo systemctl enable --now rsdriver.service
```

## Run

```bash
cd /var/opt/robot/data/dimos-m20-kronknav
source .venv/bin/activate
dimos --transport lcm --rerun-host 0.0.0.0 run deeprobotics-m20-kronknav-control --daemon
```

Connect the viewer:

```bash
dimos-viewer \
  --connect rerun+http://10.21.31.104:9877/proxy \
  --ws-url ws://10.21.31.104:3030/ws
```

Attach the RPC shell:

```bash
dimos --transport lcm shell
```
