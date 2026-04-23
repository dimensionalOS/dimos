# M20 System Software Upgrade Plan — V1.1.8.5

**Released:** 2026-04-23 by DeepRobotics
**Our baseline:** whatever V1.x was installed prior (need to capture)

## TL;DR

Upgrade via DeepRobotics's OTA. Likely wipes the rootfs (so our rsdriver
config edits will vanish) but probably preserves `/var/opt/robot/data/`
(where our nix store + dimos source live). Three items need manual
re-application after upgrade; everything else is automated via
`deploy.sh provision`.

## Changelog Risk Assessment

### HIGH — potentially breaking

**Fixed abnormal IMU delay**
> Fixed the issue of abnormal IMU delay

DeepRobotics knows about IMU delay issues too. Could be upstream fix
to the same class of timing issues we worked around (Findings
#7, #8, #18, #28). After upgrade:
- Re-run `scripts/diag_harness.py` and check imu_vs_frame_end
  distribution — if DeepRobotics fixed it, our scan_start header fix
  (Finding #28) might become redundant or misaligned.
- Verify drdds slot stamp semantics haven't changed. If the slot
  stamp is now scan_START (their bug fix), our bridge's
  `scan_start = points[0].time` heuristic still works but is
  redundant.

**Radar/camera switch states reset after reboot — fixed**
> Fixed the issue of radar and camera switch states being reset
> after reboot

We set `send_separately: true` in rsdriver config. If this fix means
"config now persists", we re-set once and it stays. If it means
"default behavior reverts to merged on every boot", we regress.
Verify by checking config after first reboot post-upgrade.

### MEDIUM — could interact

**Fixed no point cloud after boot**
Might be related to drdds_recv discovery ordering (our Finding #19).
If fixed upstream, our `drdds-recv.service` Before/After ordering
dropin may be unnecessary. Verify: does the vanilla boot sequence
now produce clouds?

**Added global planning module with 2D grid map**
DeepRobotics-internal autonomous planning. Unless we use their
navigation API, shouldn't affect our NavCmdPub-based velocity
commands.

**Fixed external cross-version ROS2 listeners crashing internal
ROS2**
We don't run ROS2 listeners; we use drdds + LCM. N/A unless a dev
machine is running ros2 topic echo at startup.

### LOW — unlikely to matter

Camera distortion correction, charging docking (we don't use),
APP UI changes, GPS mode, joint calibration, battery OTA.

**SDK mode items DO NOT affect us.** Per MEMORY, M20 has three
control modes:
  - *Regular Mode* — UDP teleop (we don't use)
  - *Navigation Mode* — drdds `/NAV_CMD` (THIS is what NavCmdPub
    uses)
  - *SDK Mode* — raw joint-level control for custom RL
    locomotion policies (we don't use)

The changelog's SDK-mode items — authorization code, prone/idle
state requirement, joint data topic frequency config — are all
constraints on custom-locomotion users, not on navigation users.
Our `NavCmdPub` continues using Navigation Mode, which the
changelog doesn't mention changing.

## Pre-upgrade backup

Already saved via `/tmp/backup_m20_customizations.sh` →
`~/m20_backup_<timestamp>/`. Contents:

- `proc_cmdline.txt` — kernel cmdline including `isolcpus=4,5,6,7`
- `system_info.txt`, `lscpu.txt` — platform identity
- `etc_fstab.txt` — fstab entries (nix bind mount + swap)
- `etc_systemd_ls.txt` — listing of /etc/systemd/system/
- `systemd_*.txt` — full text of our added units
  - drdds-recv.service
  - drdds-bridge.service (to re-disable if re-installed)
  - m20-nav-net.service
  - rsdriver_dropin.conf (the Before=drdds-recv ordering)
  - ssh_oom.conf (sshd OOMScoreAdjust)
- `rsdriver_config.yaml` — THE rsdriver config (lives in rootfs, will
  be overwritten; we need to re-apply our send_separately:true,
  ts_first_point:false, pitch/yaw/roll edits)
- `nix_gcroots.txt` — our pinned store paths (by hash)
- `nix_store_summary.txt` — store path count + our builds enumerated
- `procs_and_network.txt` — pre-upgrade process/network baseline
  for post-upgrade diff

## Upgrade steps

1. **Capture baseline** (already done)
   ```
   /tmp/backup_m20_customizations.sh
   ```

2. **Stop our stack gracefully**
   ```
   cd dimos/robot/deeprobotics/m20
   ./deploy.sh stop --host m20-770-gogo
   ```

3. **Upgrade via DeepRobotics OTA/APP** (user-driven)

4. **Post-upgrade verification**

   NOS (our concern):
   - SSH reachable? (sshd oom protection may have been reinstalled
     or wiped; sshd itself should survive)
   - Compare `/proc/cmdline` against `proc_cmdline.txt` —
     `isolcpus=4,5,6,7` is in bootloader, usually preserved
   - Compare `/etc/fstab` — our nix bind mount + swap entries
     may have been overwritten
   - Check `/var/opt/robot/data/` exists, `/var/opt/robot/data/nix`
     populated, `/var/opt/robot/data/dimos` populated
   - Check `/opt/robot/share/node_driver/config/config.yaml` —
     this WILL be overwritten; reapply our edits

   GOS + AOS (coordinated with houmanoids_www/crew/gus — see
   mail thread `hq-wisp-zh33h`):
   - On GOS: `systemctl is-masked docker.service docker.socket`
     (expect: masked; factory image has docker-ce that flushes
     FORWARD + sets DROP policy, breaking NOS→AOS routing)
   - On GOS: `test -f /etc/iptables.rules && echo STALE_FILE`
     (should NOT exist; stale factory file that rc.local loads)
   - On GOS: `grep iptables-restore /etc/rc.local` (should find
     nothing; factory rc.local loads the stale iptables.rules)
   - On GOS: `iptables -C FORWARD -i eth0 -o eth0 -j ACCEPT`
     (expect: rule exists; this is the hairpin that lets NOS
     traffic reach AOS for the drdds multicast path)
   - On AOS: `ip -br addr show eth2` (expect: DHCP lease from
     XR60 at 192.168.1.x; factory netplan was static
     192.168.8.100 which is now dead on our LAN)

5. **Re-provision**

   Both provisions are idempotent and touch different filesystems,
   so order doesn't matter. Run in parallel:

   NOS (us):
   ```
   ./deploy.sh provision --host m20-770-gogo
   ```
   Re-installs:
   - /nix bind mount (if fstab got wiped)
   - drdds-recv.service
   - rsdriver-order dropin
   - m20-nav-net.service
   - sshd OOM protection
   - Disables legacy drdds-bridge.service

   GOS + AOS (gus):
   ```
   # gus's script — houmanoids_www/relay repo
   provision-m20.sh
   ```
   Re-installs:
   - AOS netplan (DHCP via XR60, replaces factory static)
   - GOS iptables FORWARD rule + MASQUERADE on AOS
   - Docker mask (prevents docker-ce from flushing FORWARD)
   - rc.local neutralization (prevents stale iptables.rules
     reload)
   - sysctl ip_forward + rp_filter=2
   - internet-watchdog service + timer (ethernet → 5G failover)

   See Gus's mail `hq-wisp-zh33h` for full list of GOS + AOS
   touched paths.

6. **Re-apply rsdriver config manually**
   ```
   ssh [...] 'sudo cp $HOME/m20_backup_*/rsdriver_config.yaml \
       /opt/robot/share/node_driver/config/config.yaml'
   ssh [...] 'sudo systemctl restart rsdriver'
   ```
   (Or surgically re-apply the three knobs: send_separately:true,
   ts_first_point:false, per-lidar pitch/yaw/roll.)

7. **Rebuild nix modules if GC'd or store path changed**
   Our `dimos/robot/deeprobotics/m20/nix_wrappers/build_all.sh`
   handles this:
   ```
   ssh [...] 'cd /var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/nix_wrappers && ./build_all.sh'
   ```

8. **Test the IMU timing**
   ```
   ssh [...] 'python3 /path/to/diag_harness.py \
       --outdir /tmp/diag_post_upgrade \
       --yaw -0.4 --rotate_duration 10 --warmup 2 --cooldown 5'
   ```
   Compare `imu_vs_frame_end` distribution vs Finding #35 baseline
   (±50ms). If it's wildly different, our scan_start fix may need
   to be revisited (DeepRobotics may have changed slot stamp
   semantics).

9. **Check NavCmdPub still matches `basic_server`**
   Expected: `nav_cmd_pub` reports `matched=1` in smartnav log.
   If `matched=0`, something about the Navigation Mode handshake
   changed (DDS type name, subnet routing, etc). This would be
   unexpected — the changelog doesn't mention Navigation Mode
   changes, only SDK mode.

10. **Retest click-to-goal** end-to-end.

## Coordination with houmanoids_www/crew/gus

Gus owns the network plumbing (AOS netplan, GOS iptables,
docker mask, internet-watchdog). Zero overlap with our SLAM-side
paths. Mail thread: `hq-wisp-zh33h` (subject: "Re: M20 V1.1.8.5
upgrade coordination — no overlap, network-side snapshot plan").

Agreed sequencing:
```
T-5  Both snapshot scripts run in parallel:
     Us:  ./dimos/robot/deeprobotics/m20/scripts/backup_m20_customizations.sh
          → ~/m20_backup_<ts>/ (NOS focus)
     Gus: his network snapshot → ~/m20_backup_gus_<ts>/
          (AOS + GOS netplan/iptables/sysctl/systemd units +
           docker mask state + tailscale state)

T-2  Us:  ./deploy.sh stop --host m20-770-gogo
     Gus: note GOS+AOS baseline state

T+0  Afik triggers OTA via DeepRobotics APP. All three boards
     reboot (5-10 min).

T+5  Both provisions in parallel (idempotent, disjoint):
     Us:  ./deploy.sh provision --host m20-770-gogo
          + manual re-apply rsdriver config (lives in rootfs)
     Gus: provision-m20.sh (AOS netplan + GOS iptables/mask/
          rc.local neutralize)

T+10 Both verify independently:
     Us:  NOS imu_vs_frame_end via diag_harness + 10s rotation
     Gus: GOS+AOS network checks (see step 4 list above)

T+15 Clean: done.
     Regression: huddle, diff against backup tarballs.
```

Gus's concerns worth watching:
- `"Self-recovery mechanism for reloading network configuration
  after AOS host network segment failure"` — could interpret the
  DHCP-via-XR60 setup as a "failure" and restore factory static
  netplan. Gus's provision-m20.sh re-applies but there'd be a
  blackout window.
- If docker gets unmasked by the upgrade, GOS FORWARD chain gets
  flushed by dockerd and NOS loses internet route to AOS.

If either of those fires, NOS reachability via Tailscale still
works but nav stack can't discover drdds peers across boards.

## Critical unknowns to confirm with DeepRobotics

- Does the OTA preserve `/var/opt/robot/data/`? (Our entire nix
  store + dimos source lives there.)
- Does the OTA preserve the `isolcpus=4,5,6,7` kernel cmdline
  parameter?
- Did the Navigation Mode `rt/NAV_CMD` interface change at all?
  (Changelog only mentions SDK mode, but worth a sanity check.)

## Decision

Proceed with the upgrade because:
- Our modifications are well-documented (Findings #24-#35) + in git
- Backup captures pre-upgrade state for forensic diff
- `deploy.sh provision` automates most of re-installation
- Their IMU-delay fix might genuinely help our 60s-rotation stability

Risk items — address post-upgrade if encountered:
- Rootfs changes invalidate our assumptions about slot stamps →
  re-measure imu_vs_frame_end, possibly revert Finding #28 logic
- isolcpus parameter wiped → our cpu_affinity still works with any
  core set, but we'd lose exclusive core access
- NavCmdPub stops matching basic_server → unexpected; would need to
  investigate Navigation-Mode protocol changes (not flagged in
  changelog).
