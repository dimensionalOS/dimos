# Unitree Go2 Backpack LIO Setup

The onboard odometry of the Unitree Go2 naturally suffers from drift over time. To maintain high accuracy for long periods, we can [utilize cutting-edge LiDAR Inertial Odometry (LIO)](https://github.com/hku-mars/FAST_LIO) via a Livox Mid-360 sensor. Because this LIO pipeline requires dedicated processing power, an external compute module must be mounted to the Go2.

## Requirements

* **External Compute:** Must be capable of running an OS supported by DimOS, and small enough to mount to the Go2.
* **Power:** The compute must be able to draw power directly from the Go2's power supply.
* **Networking:** The compute requires **two** Ethernet ports (USB-to-Ethernet adapters work). 
* **Sensor:** Livox Mid-360 LiDAR.

---

## Hardware Setup

1. **Mount the Hardware:** Attach your external compute and the Livox Mid-360 to the Go2. *Message Mustafab on Discord for our CAD Files*
2. **Connect Power:** Connect the external compute directly to the Go2's power supply. 
3. **Connect Data:** * Use an Ethernet cable to connect the **Go2** to the **external compute**.
   * Use your second Ethernet port to connect the **Livox Mid-360** to the **external compute**.
4. **Configure the Sensor:** Follow this dedicated guide to finalize the sensor networking: [Livox Mid-360 Setup Guide](sensors/mid360.md).

---

## Software Configuration

### 1. Install DimOS
Download and run the installer on your external compute: `[Installer](../../README.md#interactive-install)`

### 2. Configure Sensor Pitch Angle
If you are using a custom mount with a different pitch angle than our standard design, you will need to update this. 

Open `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_backpack.py` and modify the `ANGLE_OF_MID_360_ON_ROBOT` variable.

> **Note on Pitch Direction:** If the Mid-360's wire connector is facing the rear (the opposite direction the dog faces), a **positive pitch** means the sensor is tilting down.

---

## Network Access & Port Forwarding

To access the Go2's control center from your personal machine (e.g., your laptop), you need to set up port forwarding. Run the following command on the machine you are using to control the robot:


`ssh -L 9090:localhost:9090 -L 9876:localhost:9876 -L 7779:localhost:7779 <external-compute-username>@<external-compute-ip>`


---

## How to Run

1. On the external compute, launch the backpack system by running:
   `dimos --viewer rerun-web --ceiling-height <your-ceiling-heigh-in-meters> unitree-go2-backpack`
2. On your controlling device, open a web browser and navigate to the viewer:
   [http://localhost:7779/](http://localhost:7779/)

