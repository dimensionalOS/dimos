# Galaxea A1Z + G1Z

The A1Z integration uses native Linux SocketCAN, the vendor's 250 Hz MIT
position-control loop, the G1Z URDF for gravity compensation, and the vendor
G1Z gripper implementation.

## Vendor SDK

The G1Z requires the vendor SDK's `gripper` branch; vendor `main` does not
accept `with_gripper` and cannot actuate CAN motor 7.

```bash
git clone --branch gripper https://github.com/userguide-galaxea/GALAXEA-A1Z.git
uv pip install -e ./GALAXEA-A1Z
```

The Jetson checkout currently used by DimOS is `/home/dimos/GALAXEA-A1Z` and
must remain on that branch. DimOS deliberately has no Linux userspace-CAN
fallback. After boot or reconnecting the HHS adapter, bind it to the kernel
driver, configure the stable `a1zcan` SocketCAN interface, and verify that the
driver can actually transmit:

```bash
sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh
```

Do not start DimOS unless the script prints `A1Z CAN setup passed`. Galaxea's
HHS USB-CANFD adapter is incompatible with `gs_usb` in some Linux kernels. An
affected kernel still creates a normal-looking, UP CAN interface but drops
every transmission. The setup script detects that misleading state and prints
the supported-kernel and exact-kernel patch options. Galaxea recommends kernel
6.8.0-124 or newer; Jetsons and other pinned-kernel hosts require a persistent
driver patch built for their exact kernel.

The A1Z has no brakes. Support the arm and keep the workspace clear before
starting a hardware blueprint. Enabling the G1Z also initializes the gripper.
