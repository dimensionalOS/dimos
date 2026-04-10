# Software Development Guide

Version: V1.0.3-0


# 1 Inspection Communication Protocol


## 1.1 Protocol Overview

This protocol is based on the TCP or UDP (selectable according to specific development needs) and applies to the communication between the robot and the host computer (external board card or system).


### 1.1.1 Protocol Hierarchies

The layer of this protocol in OSI model, and the data structure of protocol stack, as shown in Table below:

| Inspection Protocol | Application layer (layer 7) |
| --- | --- |
| TCP/IP or UDP/IP protocol | Transport layer (layer 4) |
| Network layer (layer 3) |
| Ethernet | Link layer (layer 2) |
| Physical layer (layer 1) |
| Notes: Layer 5 and layer 6 are not used |


### 1.1.2 Protocol Port Number

When using this protocol, the robot is the TCP/UDP server, and the host computer (external board card or system) is the TCP/UDP client. The UDP server address and port number for the protocol is 10.21.31.103:30000, and the TCP server address and port number is 10.21.31.103:30001.


### 1.1.3 Interaction Mechanism

As shown in Figure below, this protocol uses the request/response mechanism. The host computer can actively issue the requests such as data query and control command to the robot, then robot responds to some of the requests.


![Interaction Mechanism Diagram](https://alidocs.dingtalk.com/core/api/resources/img/5eecdaf48460cde58272ab7060997fd277fce252ea93ed5575b8339e1c4c24830dd9a450996693588d68742cd653602afc228fb22ff2c4af25a76dc9e2110a16161c8c5ba82d152ad45d3b5df0c96fedb9fc0308db36ddc9a46b1276a680b1b1)


### 1.1.4 Application Protocol Data Unit

This protocol, APDU (Application Protocol Data Unit), adopts the structure of "Protocol Header + ASDU (Application Service Data Unit)", and each APDU can carry 1 ASDU.


![APDU Structure Diagram](https://alidocs.dingtalk.com/core/api/resources/img/5eecdaf48460cde58272ab7060997fd277fce252ea93ed5575b8339e1c4c24830dd9a450996693588d68742cd653602a8169d9d9ddf3b83fabd0536990c79ac0096d224a34d14de4540536ac882c3542e9e776da5008a4596089bf559e2ddd35)


### 1.1.5 Protocol Header Structure

The length of the protocol header is fixed to 16 bytes, as shown in Table below.

| No. | Content | Length | Value | Remarks |
| --- | --- | --- | --- | --- |
| 1 | Synchronization character | 1 | 0xeb | Fixed |
| 2 | Synchronization character | 1 | 0x91 | Fixed |
| 3 | Synchronization character | 1 | 0xeb | Fixed |
| 4 | Synchronization character | 1 | 0x90 | Fixed |
| 5 | Length | 2 |  | The length of the ASDU byte segment in APDU, which is in little endian, with the low bytes first. The maximum length of ASDU is limit to 65,535 bytes. |
| 6 | Message ID | 2 |  | The unique identifier of each frame of the message, used to identify the corresponding relationship between the request frame and the response frame. The value is controlled by the requester, and the response frame replies with the same value.It increments from 0, then starts at 0 again after reaching 65,535.It is in little endian, with the low bytes first. |
| 7 | ASDU Structure | 1 |  | ASDU data format type identification bit;When using XML format, the value is 0x00;When using JSON format, the value is 0x01. |
| 8 | Reserved | 7 | 0x00 | Reserve 7 bytes |


### 1.1.6 ASDU Structure

The ASDU data content of this protocol is in JSON/XML format (Refer to Section "1.1.5 Protocol Header Structure"; must be specified in the protocol header), containing the following general fields:

| Field | Meaning |
| --- | --- |
| Type | Message type |
| Command | Message command code |
| Time | Message sending time (local time zone), with the format of: YYYY-MM-DD HH:MM:SS |
| Items | Parameters of the message |

[Notes] It is recommended to use JSON format, which has faster processing performance and more comprehensive data structures for ASDU, and supports more message types.

There is a JSON ASDU case:

```
{	"PatrolDevice": {		"Type": 1002,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

There is an XML ASDU case:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1002</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items/>  </PatrolDevice>
```


## 1.2 ASDU Message Set (Control Type)


### 1.2.1 Heartbeat

You can send heartbeat commands to the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 100 | 100 | Heartbeat |

[Notes] It is recommended to send this command at a frequency of no less than 1 Hz. The robot will report real-time status and abnormal status information to the IP and port that continuously sends heartbeat commands. For details, please refer to Section "1.3 ASDU Message Set (Status Type)".

JSON request:

```
{	"PatrolDevice": {		"Type": 100,		"Command": 100,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>100</Type>	<Command>100</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items/></PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.


### 1.2.2 Usage Mode

You can switch the usage mode of the robot by this request and determine whether the operation was successful by referring to the response information in Section "1.3 ASDU Message Set (Status Type)".

| Type | Command | Message type |
| --- | --- | --- |
| 1101 | 5 | Usage Mode Switching |

JSON request:

```
{	"PatrolDevice": {		"Type": 1101,		"Command": 5,		"Time": "2023-01-01 00:00:00",		"Items": {             "Mode": 0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1101</Type>	<Command>5</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<Mode>0</Mode>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Mode | The usage mode of the robot | int | Regular Mode = 0Navigation Mode = 1Assist Mode = 2 |

[Notes] In regular mode, axis commands are supported (refer to Section "1.2.5 Motion Control (Axis Command)"). In navigation mode, navigation tasks are supported.


### 1.2.3 Motion State

You can switch the motion state information of the robot by this request and determine whether the operation was successful by referring to the response information in Section "1.3 ASDU Message Set (Status Type)".

| Type | Command | Message type |
| --- | --- | --- |
| 2 | 22 | Motion State Switching |

JSON request:

```
{	"PatrolDevice": {		"Type": 2,		"Command": 22,		"Time": "2023-01-01 00:00:00",		"Items": {             "MotionParam": 0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>2</Type>	<Command>22</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<MotionParam>0</MotionParam>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| MotionParam | Robot motion state[1] | int | Idle = 0 Stand = 1 Soft Emergency Stop = 2 Power-on Damping = 3 Sit = 4 RL Control = 17 |

[1] The conversion relationship of the robot's motion state is shown in the figure below:


> **Motion State Transition Diagram:**
> RL Control State ↔ Power-on Damping State ↔ Sit-to-Stand State ↔ Standard Motion State / Agile Motion State
> Idle State → [Stand Command] → Stand State → Sit Command → Sitting State
> Soft Emergency STOP State


[Notes] In Agile Motion Mode, gait speed responds more effectively, making it suitable for navigation and other autonomous algorithm development.


### 1.2.4 Gait Switching

You can switch the gait of the robot by this request and determine whether the operation was successful by referring to the response information in Section "1.3 ASDU Message Set (Status Type)".

| Type | Command | Message type |
| --- | --- | --- |
| 2 | 23 | Gait Switching |

JSON request:

```
{	"PatrolDevice": {		"Type": 2,		"Command": 23,		"Time": "2023-01-01 00:00:00",		"Items": {             "GaitParam": 0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>2</Type>	<Command>23</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<GaitParam>0</GaitParam>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| GaitParam | Robot gait | int | Basic (Standard Motion State) = 0x1001Stair (Standard Motion Mode) =0x1003Flat (Agile Motion Mode) = 0x3002Stair (Agile Motion Mode) = 0x3003 |

[Notes] Different motion modes support different gaits. When switching gaits, the robot will automatically switch to the corresponding motion mode. Likewise, when switching motion modes, the robot will automatically switch to the default gait for that mode. For details on motion modes, refer to Section "1.2.3 Motion State".


### 1.2.5 Motion Control (Axis Command)

You can control the motion of the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 2 | 21 | Motion Control |

[Notes] This command is only supported in Regular Mode.

JSON request:

```
{	"PatrolDevice": {		"Type": 2,		"Command": 21,		"Time": "2023-01-01 00:00:00",		"Items": {             "X": 0.0,             "Y": 0.0,             "Z": 0.0,             "Roll": 0.0,             "Pitch": 0.0,             "Yaw": 0.0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>2</Type>	<Command>21</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<X>0.0</X>		<Y>0.0</Y>		<Z>0.0</Z>		<Roll>0.0</Roll>		<Pitch>0.0</Pitch>		<Yaw>0.0</Yaw>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| X | Forward and backward speed | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, 1 and -1 indicate moving forward or backward at maximum speed) |
| Y | Left and right movement speed | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, 1 and -1 indicate moving left or right at maximum speed) |
| Z | Vertical movement speed | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, 1 and -1 indicate upward and downward movement at maximum speed) |
| Roll | Roll angle | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, the sign of each parameter value is defined according to the right-hand coordinate system) |
| Pitch | Pitch angle | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, the sign of each parameter value is defined according to the right-hand coordinate system) |
| Yaw | Yaw angle | float | [-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed, the sign of each parameter value is defined according to the right-hand coordinate system) |

The positive directions for the robot's movement and rotation are shown in the diagram below:


![Robot Axis Directions](https://alidocs.dingtalk.com/core/api/resources/img/5eecdaf48460cde5e6267e6ad72b3972b71e1e22442494ca75b8339e1c4c24830dd9a450996693588d68742cd653602a60709c486a81f6b462d1387e4a37a436c3645387995061b44cbf064aa2c00cbeca1f5e3d2388569a062f98583fd15764)


[Notes] Recommended frequency for translation/rotation control: 20 Hz. Only X, Y, and Yaw are effective in RL control state.


### 1.2.6 Flashlight

You can turn the robot's front and rear lights on or off by this request and determine whether the operation was successful by referring to the response information in Section "1.3 ASDU Message Set (Status Type)".

| Type | Command | Message type |
| --- | --- | --- |
| 1101 | 2 | Flashlight |

JSON request:

```
{	"PatrolDevice": {		"Type": 1101,		"Command": 2,		"Time": "2023-01-01 00:00:00",		"Items": {             "Front": 0,             "Back": 0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1101</Type>	<Command>2</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<Front>0</Front>		<Back>0</Back>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Front | Front flashlight of the robot | int | Light off= 0Light on= 1 |
| Back | Back flashlight of the robot |


### 1.2.7 Autonomous Charging

You can initiate autonomous charging of the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 2 | 24 | Autonomous Charging |

JSON request:

```
{    "PatrolDevice":{        "Type":2,        "Command":24,        "Time":"2023-01-01 00:00:00",        "Items":{            "Charge":1        }    }}
```

XML request:

```
<PatrolDevice>  <Type>2</Type>  <Command>24</Command>  <Time>2023-01-01 00:00:00</Time>  <Items>    <Charge>1</Charge>  </Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Charge | Robot charging status | int | Stop Charging = 0Start Charging = 1Clear Charging Status = 2 |


### 1.2.8 Set the Sleep Mode and Settings

You can set the current sleep mode settings of the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 1101 | 6 | Set Sleep Mode Settings |

JSON request:

```
{    "PatrolDevice":{        "Type":1101,        "Command":6,        "Time":"2023-01-01 00:00:00",        "Items":{            "Sleep":false,            "Auto":true,            "Time":5        }    }}
```

XML request:

```
<PatrolDevice>  <Type>1101</Type>  <Command>6</Command>  <Time>2023-01-01 00:00:00</Time>  <Items>    <Sleep>0</Sleep>    <Auto>0</Auto>    <Time>5</Time>  </Items></PatrolDevice>
```

In this request message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Sleep | Enter/exit robot sleep mode | bool | Enter sleep mode = trueWake up (exit sleep mode) = false |
| Auto | Automatic sleep switch: whether to automatically enter sleep mode after no messages have been sent for a period of time | bool | Yes = trueNo = false |
| Time | Waiting time before entering sleep mode after no messages are sent (effective only when automatic sleep is enabled) | int | [5, 30] |


## 1.3 ASDU Message Set (Status Type)


### 1.3.1 Obtain Real-time State

The robot will actively report status information to the IP address and port that sent the heartbeat command (refer to section "1.2.1 Heartbeat").


#### 1.3.1.1 Basic Status Report

This message type is actively reported at 2 Hz, you can obtain the current basic status of the robot through the response of this message type.

| Type | Command | Message type |
| --- | --- | --- |
| 1002 | 6 | Obtain Basic Status |

JSON response:

```
{	"PatrolDevice": {		"Type": 1002,		"Command": 6,		"Time": "2023-01-01 00:00:00",		"Items": {			"BasicStatus": {				"MotionState": 0,				"Gait": 0,				"Charge":0,				"HES"：0,				"ControlUsageMode"：0,				"Direction"：0,				"OOA"：0,				"PowerManagement"：0,				"Sleep"：false,				"Version"：STD			}		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1002</Type>	<Command>6</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<BasicStatus>			<MotionState>0</MotionState>			<Gait>0</Gait>			<Charge>0</Charge>			<HES>0</HES>			<ControlUsageMode>0</ControlUsageMode>			<Direction>0</Direction>			<OOA>0</OOA>			<PowerManagement>0</PowerManagement>			<Sleep>false</Sleep>			<Version>STD</Version>		</BasicStatus>	</Items></PatrolDevice>
```

The Items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| MotionState | Robot motion state[1] | int | Idle = 0Stand = 1Soft Emergency Stop = 2Power-on Damping = 3Sitting = 4RL Control = 17 |
| Gait | Robot gait | int | Basic (Standard Motion State) = 0x1001High Obstacles (Standard Motion State) = 0x1002Flat (Agile Motion State) = 0x3002Stair (Agile Motion State) = 0x3003 |
| Charge | Robot charging status | int | Idle = 0Enter charge dock = 1Charging = 2Exiting charge dock = 3Robot error = 4Robot is on the dock but not charged = 5 |
| HES | Hard emergency stop status | int | Not triggered = 0triggered = 1 |
| ControlUsageMode | Robot control usage mode | int | Regular mode = 0Navigation mode = 1Assist mode = 2 |
| Direction | The robot's forward direction[2] | int | Front = 0Back = 1 |
| OOA | Obstacle avoidance status in assist mode | int | Not started = 0Idle = 1Obstacle avoidance not triggered=2Active obstacle avoidance = 3 |
| PowerManagement | Power management mode | int | Regular mode = 0Single battery mode = 1 |
| Sleep | Robot sleep status | int | Awake = 0 Sleep = 1Entering Sleep = 2 |
| Version | Device version | / | Lynx M20 = STDLynx M20 Pro = PRO |

[2] The robot's forward direction is defined as the movement direction when a positive X-axis velocity is commanded. If the forward direction is set to "Back", the side with the Hard Emergency Stop is considered forward direction.


#### 1.3.1.2 Motion Control Status Report

This message type is actively reported at 10 Hz, you can obtain the current motion control status of the robot through the response of this message type.

| Type | Command | Message type |
| --- | --- | --- |
| 1002 | 4 | Obtain Motion Control Status |

JSON response:

```
{	"PatrolDevice": {		"Type": 1002,		"Command": 4,		"Time": "2023-01-01 00:00:00",		"Items": {			"MotionStatus": {				"Roll": 0.0,				"Pitch": 0.0,				"Yaw": 0.0,				"OmegaZ": 0.0,				"LinearX": 0.0,				"LinearY": 0.0,				"Height": 0.0,				"Payload":0.0,				"RemainMile":0.0			},			"MotorStatus": {				"LeftFrontHipX": 0.0,				"LeftFrontHipY": 0.0,				"LeftFrontKnee": 0.0,				"LeftFrontWheel": 0.0,				"RightFrontHipX": 0.0,				"RightFrontHipY": 0.0,				"RightFrontKnee": 0.0,				"RightFrontWheel": 0.0,				"LeftBackHipX": 0.0,				"LeftBackHipY": 0.0,				"LeftBackKnee": 0.0,				"LeftBackWheel": 0.0,				"RightBackHipX": 0.0,				"RightBackHipY": 0.0,				"RightBackKnee": 0.0,				"RightBackWheel": 0.0            }		}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1002</Type>	<Command>4</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<MotionStatus>			<Roll>0.0</Roll>			<Pitch>0.0</Pitch>			<Yaw>0.0</Yaw>  			<OmegaZ>0.0</OmegaZ>			<LinearX>0.0</LinearX>			<LinearY>0.0</LinearY>			<Height>0.0</Height>			<Payload>0.0</Payload>			<RemainMile>0.0</RemainMile>		</MotionStatus>		<MotorStatus>			<LeftFrontHipX>0.0</LeftFrontHipX>			<LeftFrontHipY>0.0</LeftFrontHipY>			<LeftFrontKnee>0.0</LeftFrontKnee>			<LeftFrontWheel>0.0</LeftFrontWheel>			<RightFrontHipX>0.0</RightFrontHipX>			<RightFrontHipY>0.0</RightFrontHipY>			<RightFrontKnee>0.0</RightFrontKnee>			<RightFrontWheel>0.0</RightFrontWheel>			<LeftBackHipX>0.0</LeftBackHipX>			<LeftBackHipY>0.0</LeftBackHipY>			<LeftBackKnee>0.0</LeftBackKnee>			<LeftBackWheel>0.0</LeftBackWheel>			<RightBackHipX>0.0</RightBackHipX>			<RightBackHipY>0.0</RightBackHipY>			<RightBackKnee>0.0</RightBackKnee>			<RightBackWheel>0.0</RightBackWheel>		</MotorStatus>	</Items>
```

In this response message, the Items field contains the MotionStatus and MotorStatus parameter groups:

The MotionStatus parameter group provides feedback on the motion status of the robot:

| Parameters | Meaning | Type |
| --- | --- | --- |
| Roll/Pitch/Yaw | The posture angle of robot (rad) | float |
| OmegaZ | Z-axis angular velocity of robot (rad/s) | float |
| LinearX / LinearY | X-axis/Y-axis linear velocity of robot (m/s) | float |
| Height | The height of the robot (m) | float |
| Payload | Invalid parameter | / |
| RemainMile | Estimated remaining range (km) | float |

The MotorStatus[3] parameter group provides feedback on the motion status of each joint of the robot:

| Parameters | Meaning | Type |
| --- | --- | --- |
| *HipX | Angle of hip joint for abduction and adduction (rad) | float |
| *HipY | Angle of hip joint for flexion and extension (rad) | float |
| *Knee | Angle of knee joint (rad) | float |
| *Wheel | Speed of wheel joint (rad/s) | float |

[3] The asterisk (*) to the left of a parameter item in the MotorStatus parameter group indicates the position of an omitted parameter name. The left side is the side where the battery bin is located. The back side is the side with the Hard Emergency STOP button. For example, LeftBackHipX refers to the left front HipX joint for abduction and adduction, and the parameter value indicates the angle of the joint. The names and meanings of other parameters follow the same principle.


#### 1.3.1.3 Device State Report

This message type is actively reported at 2 Hz, you can obtain the current device state of the robot through the response of this message type.

| Type | Command | Message type |
| --- | --- | --- |
| 1002 | 5 | Obtain Device state |

JSON response:

```
{	"PatrolDevice": {		"Type": 1002,		"Command": 5,		"Time": "2023-01-01 00:00:00",		"Items": {			"BatteryStatus": {				"VoltageLeft": 0.0,				"VoltageRight": 0.0,				"BatteryLevelLeft": 0.0,				"BatteryLevelRight": 0.0,				"battery_temperatureLeft": 0.0,				"battery_temperatureRight": 0.0,				"chargeLeft":false,				"chargeRight":false			},			"DeviceTemperature": {				"LeftFrontHipXMotor": 0.0,				"LeftFrontHipXDriver": 0.0,				"LeftFrontHipYMotor": 0.0,				"LeftFrontHipYDriver": 0.0,				"LeftFrontKneeMotor": 0.0,				"LeftFrontKneeDriver": 0.0,				"LeftFrontWheelMotor": 0.0,				"LeftFrontWheelDriver": 0.0,				"RightFrontHipXMotor": 0.0,				"RightFrontHipXDriver": 0.0,				"RightFrontHipYMotor": 0.0,				"RightFrontHipYDriver": 0.0,				"RightFrontKneeMotor": 0.0,				"RightFrontKneeDriver": 0.0,				"RightFrontWheelMotor": 0.0,				"RightFrontWheelDriver": 0.0,				"LeftBackHipXMotor": 0.0,				"LeftBackHipXDriver": 0.0,				"LeftBackHipYMotor": 0.0,
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1002</Type>	<Command>5</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<BatteryStatus>			<VoltageLeft>0.0</VoltageLeft>			<VoltageRight>0.0</VoltageRight>			<BatteryLevelLeft>0.0</BatteryLevelLeft>			<BatteryLevelRight>0.0</BatteryLevelRight>			<Battery_temperatureLeft>0.0</Battery_temperatureLeft>			<Battery_temperatureRight>0.0</Battery_temperatureRight>			<chargeLeft>false</chargeLeft>			<chargeRight>false</chargeRight>		</BatteryStatus>		<DeviceTemperature>			<LeftFrontHipXMotor>0.0</LeftFrontHipXMotor>			<LeftFrontHipXDriver>0.0</LeftFrontHipXDriver>			<LeftFrontHipYMotor>0.0</LeftFrontHipYMotor>			<LeftFrontHipYDriver>0.0</LeftFrontHipYDriver>			<LeftFrontKneeMotor>0.0</LeftFrontKneeMotor>			<LeftFrontKneeDriver>0.0</LeftFrontKneeDriver>			<LeftFrontWheelMotor>0.0</LeftFrontWheelMotor>			<LeftFrontWheelDriver>0.0</LeftFrontWheelDriver>			<RightFrontHipXMotor>0.0</RightFrontHipXMotor>			<RightFrontHipXDriver>0.0</RightFrontHipXDriver>			<RightFrontHipYMotor>0.0</RightFrontHipYMotor>			<RightFrontHipYDriver>0.0</RightFrontHipYDriver>			<RightFrontKneeMotor>0.0</RightFrontKneeMotor>			<RightFrontKneeDriver>0.0</RightFrontKneeDriver>			<RightFrontWheelMotor>0.0</RightFrontWheelMotor>			<RightFrontWheelDriver>0.0</RightFrontWheelDriver>			<LeftBackHipXMotor>0.0</LeftBackHipXMotor>			<LeftBackHipXDriver>0.0</LeftBackHipXDriver>			<LeftBackHipYMotor>0.0</LeftBackHipYMotor>
```

In this response message, the Items field contains six parameter groups: BatteryStatus、DeviceTemperature、Led、GPS、DevEnable and CPU.

The BatteryStatus[4] parameter group provides feedback on the status of batteries of the robot:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Voltage* | Voltage of the battery (V) | float |  |
| BatteryLevel* | Percentage of remaining battery power (%) | float | [0,100] |
| Battery_temperature* | Temperature of battery (℃) | float |  |
| Charge* | Charging status of battery | bool | true = Chargingfalse = Not Charging |

[4] The asterisk (*) to the right of the parameter item indicates the position of the omitted parameter name. The right side is the side closer to the hard emergency stop button. For example, ‘BatteryLevelRight’ refers to the remaining battery percentage on the right side (the side closer to the hard emergency stop button). The meanings of other parameters follow the same principle.

The DeviceTemperature[5] parameter group provides feedback on the temperature information of each joint motor and driver:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| *Motor | Temperature of motor (℃) | float |  |
| *Driver | Temperature of driver (℃) | float |  |

[5] The asterisk (*) on the left side of the parameter item indicates the omitted parameter name information. Please refer to [3] to determine the joint name. For example, RightFrontKneeDriver refers to the right front knee joint driver, and the parameter value indicates the temperature information of the right front knee joint driver. The names and meanings of other parameters follow the same principle.

The LED parameter group provides feedback on the front and back flashlight status of the robot:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Fill:Front / Back | Status of front / back flashlight | int | Off = 0, On = 1 |

The GPS parameter group provides feedback satellite positioning module positioning data:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Latitude | Latitude of the robot in the world coordinate system (deg) | float | Positive values indicate north of the equator; Negative values indicate south of the equator. |
| Longitude | Longitude of the robot in the world coordinate   system (deg) | float | Positive values indicate east of the prime meridian; Negative values indicate west of the prime meridian. |
| Speed | Ground speed (km/h) | float |  |
| Course | Robot course in the world coordinate system (deg) | float | The angle between the direction of travel and true north |
| FixQuality | Robot positioning quality | float | The higher the value, the more accurate the positioning. |
| NumSatellites | Number of satellites involved in positioning | int |  |
| Altitude | Altitude of the robot (m) | float |  |
| HDOP | Position Dilution of PrecisionComprehensive indicator reflecting positioning accuracy | float | [0.5, 99.9]A smaller value indicates higher precision. |
| VDOP | Horizontal Dilution of PrecisionReflects positioning accuracy in the horizontal direction | float |
| PDOP | Vertical Dilution of PrecisionReflects positioning accuracy in the vertical direction | float |
| VisibleSatellites | Total number of visible satellites | int |  |

The DevEnable parameter group provides feedback on the operating status of the robot's internal components. The meanings of some of the parameters are as follows:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Lidar:Front/Back | Front and back power switch | int | Off = 0 On = 1Initializing = 2 |
| GPS | Satellite positioning module power switch | int | Off = 0, On = 1 |
| Video:Front/Back | Front and back camera power switch | int | Off = 0, On = 1 |

The CPU[6] parameter group provides feedback on the operating status of each CPU inside the robot:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Temperature | Maximum CPU temperature | float |  |
| FrequencyInt | Efficiency core (A55) usage (%) | float |  |
| FrequencyApp | Performance core (A76) usage (%) | float |  |

[6] In the CPU parameter group, AOS, NOS, and GOS correspond to the CPUs of the three hosts inside the robot, respectively. GOS is only effective on the Lynx M20 Pro.


### 1.3.2 Obtain Abnormal Status

You can obtain the current abnormal status information of the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 1002 | 3 | Obtain Abnormal Status |

[Notes] The response will actively report to the IP and port that sent the heartbeat command at a fixed frequency of 2 Hz, when the status changes (such as when an abnormal occurs or is resolved), the robot will perform an additional report.

JSON response:

```
{	"PatrolDevice": {		"Type": 1002,		"Command": 3,		"Time": "2023-01-01 00:00:00",		"Items": {   			"ErrorList":[			{				"errorCode": errorCode,				"component": value			},			{				"errorCode": errorCode,				"component": value			},			…//Other omitted errors			]		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  <Type>1002</Type>  <Command>3</Command>  <Time>2023-01-01 00:00:00</Time>  <Items>    <ErrorList>      <errorCode>errorCode</errorCode>      <component>value</component>    </ErrorList>    <ErrorList>      <errorCode>errorCode</errorCode>      <component>value</component>    </ErrorList>  </Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| errorCode | Error code | int | Check the table below |
| component | Location of the component where the error occurred[7] | int | Use bits as part numbers |

[7] Please refer to Section "1.3.1.2 Motion Control Status Reporting" for joint numbering, which follows the order of parameters in the <MotorStatus> group. Joints are numbered from low to high bits in the order of FL/FR/BL/BR + HipX/HipY/Knee/Wheel. For example, 0x21 (0000 0000 0010 0001) indicates that the 1st joint (bit 0, LeftFrontHipX) and the 6th joint (bit 5, RightFrontHipY) have errors corresponding to the errorCode. Other joint bits follow the same convention. For battery numbering, bit 0 represents the right-side battery (the side near the hardware emergency stop button), and bit 1 represents the left-side battery.

The meaning of the value of errorCode is shown in the following table:

| Value | Meaning | Value | Meaning |
| --- | --- | --- | --- |
| 0x8001 | Motor Temperature Warning | 0x8116 | Battery Charge Low-temperature Protection |
| 0x8002 | Motor Over-temperature Protection | 0x8117 | Battery Cell Overvoltage Protection |
| 0x8003 | Motor Temperature Critical Shutdown | 0x8118 | Battery Pack Overvoltage Protection |
| 0x8007 | Joint Driver Over-temperature | 0x8119 | Battery Pack Undervoltage Protection |
| 0x8008 | Driver Undervoltage Protection | 0x8120 | Battery Charging Overcurrent Protection |
| 0x8009 | Driver Overvoltage Protection | 0x8121 | Battery Discharge Overcurrent Protection |
| 0x8012 | Joint Driver Communication Timeout | 0x8122 | Short Circuit Protection |
| 0x8016 | No Encoder Value | 0x8123 | Battery Front-end Detection IC Error |
| 0x8020 | Driver Overcurrent Protection | 0x8124 | Battery Software MOS Lock |
| 0x8021 | Temperature Sensor Disconnected | 0x8125 | Battery Discharge Over-temperature Warning |
| 0x8022 | Joint Angle Limit Exceeded | 0x8126 | Battery Discharge Low-temperature Warning |
| 0x8024 | Joint Data is NaN | 0x8127 | Battery Charging Over-temperature Warning |
| 0x8025 | Joint Data Update Error | 0x8128 | Battery Charging Low-temperature Warning |
| 0x8027 | Body Attitude Error | 0x8129 | Battery Output Minimum Voltage Warning |
| 0x8028 | Driver Status Error | 0x8201 | CPU Usage Overload Warning |
| 0x8029 | Motion Attitude Error | 0x8202 | CPU Temperature Overheat Warning |
| 0x8030 | Joint Driver Over-temperature Warning | 0x8211 | CPU Usage Overload Protection |
| 0x8102 | Low Battery Warning | 0x8212 | CPU Temperature Overheat Protection |
| 0x8103 | Protected Battery Level | 0x8501 | Retrieving positioning data timeout during autonomous charging |
| 0x8106 | Battery Output Minimum Voltage Protection | 0x8502 | Failed to enable Reflector localization algorithm during autonomous charging |
| 0x8107 | Battery Discharge Over-temperature Protection | 0x8503 | Abnormal positioning information during autonomous charging |
| 0x8108 | Battery Cell Undervoltage Protection | 0x8506 | No current at the charging dock during autonomous charging |
| 0x8112 | Battery Discharge Low-temperature Protection | 0x8509 | Dock entry timeout during autonomous charging |
| 0x8115 | Battery Charging Over-temperature Protection | 0x8510 | Dock withdrawal timeout during autonomous charging |


### 1.3.3 Query the Sleep Mode and Settings Information

You can query the current sleep mode settings of the robot by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 1101 | 7 | Obtain Sleep Mode Settings |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 1101,		"Command": 7,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1101</Type>	<Command>7</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items/></PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 1101,		"Command": 7,		"Time": "2023-01-01 00:00:00",		"Items": {			"Sleep":false,			"Auto":true,			"Time":5		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1101</Type>	<Command>7</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<Sleep>0</Sleep>		<Auto>0</Auto>		<Time>5</Time>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Sleep | Robot sleep mode | bool | Sleep = trueAwake = false |
| Auto | Whether to automatically enter sleep mode | bool | Automatically enter sleep mode after arrival time=trueWill not automatically enter sleep mode=false |
| Time | The waiting time before automatically entering sleep mode when no messages are received (minute) | int | [5, 30] |


## 1.4 ASDU Message Set (Inspection Type)

[Notes]  The ASDU message set contained in this section is only supported by the M20 Pro. The M20 does not support the commands in this section.


### 1.4.1 Location Initialization and Reset

You can control robot to initialize and reset location by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 2101 | 1 | Location Initialization and Rest |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 2101,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"PosX":0.0,			"PosY":0.0,			"PosZ":0.0,			"Yaw":0.0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>2101</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<PosX>0.0</PosX>		<PosY>0.0</PosY>		<PosZ>0.0</PosZ>		<Yaw>0.0</Yaw>	</Items></PatrolDevice>
```

In this request message, the Items field contains the following parameters:

| Parameters | Meaning | Type |
| --- | --- | --- |
| PosX/PosY/PosZ | Coordinates that need to be repositioned in the map coordinate system[8](m) | float |
| Yaw | Attitude angle of the robot rotating around the Z-axis in the map coordinate system[8](rad) | float |

[8] The conversion formula for the robot position in Map Coordinate System and Pixel Coordinate System is as follows:

{xp=Pos⁡X−X0Res⁡yp=H−Pos⁡Y−Y0Res⁡(xp,yp are rounded down )\left\{\begin{array}{l}
x_{p}=\frac{\operatorname{Pos} X-X 0}{\operatorname{Res}} \\
\\
y_{p}=H-\frac{\operatorname{Pos} Y-Y 0}{\operatorname{Res}}
\end{array} \quad\left(x_{p}, y_{p} \text { are rounded down }\right)\right.⎩⎨⎧​xp​=ResPosX−X0​yp​=H−ResPosY−Y0​​(xp​,yp​ are rounded down )

|  |  |
| --- | --- |
| Map Coordinate System | Pixel Coordinate System |

(xp,yp)(x_{p}, y_{p})(xp​,yp​)is the coordinate values in Pixel Coordinate System.The remaining parameter information is located in the occ_grid.yaml file. Please refer to the Software User Guide  to view the parameters in the occ_grid.yaml file in the map folder.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 2101,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"ErrorCode":0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>2101</Type>	<Command>1</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<ErrorCode>1</ErrorCode>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| ErrorCode | The result of executing the location initialization command | int | Success = 0Failure = 1 |


### 1.4.2 Obtain location information in the map coordinate system

You can obtain the robot's current location information in the map coordinate system by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 1007 | 2 | Obtain location information in the map coordinate system |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 1007,		"Command": 2,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1007</Type>	<Command>2</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>	</Items></PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 1007,		"Command": 2,		"Time": "2023-01-01 00:00:00",		"Items": {			"Location":1,			"PosX":0.0,			"PosY":0.0,			"PosZ":0.0,			"Roll":0.0,			"Pitch":0.0, 			"Yaw":0.0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1007</Type>	<Command>2</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<Location>1</Location>		<PosX>0.0</PosX>		<PosY>0.0</PosY>		<PosZ>0.0</PosZ>		<Roll>0.0</Roll>		<Pitch>0.0</Pitch>		<Yaw>0.0</Yaw>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Location | Location status | int | Normal location = 0Location lost = 1 |
| PosX/Y/Z | Robot coordinates in the map coordinate system[8](m) | float |  |
| Roll/Pitch/Yaw | Robot attitude angle in the map coordinate system[8](rad) | float |  |


### 1.4.3 Obtain perception software status information in navigation

You can obtain perception software status information in robot navigation by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 2002 | 1 | obtain perception software status information in navigation |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 2002,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>2002</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>	</Items></PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 2002,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"Location":1,			"ObsState":0.0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>2002</Type>	<Command>1</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<Location>1</Location>		<ObsState>0.0</ObsState>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Location | Location status | int | Normal location = 0Location lost = 1 |
| ObsState | Whether in obstacle state | int | Not in obstacle avoidance mode = 0In obstacle avoidance mode = 1 |


### 1.4.4 Issue Navigation Task

You can issue the navigation task by this request, that is, go to a target point and execute the tasks at this target point.

| Type | Command | Message type |
| --- | --- | --- |
| 1003 | 1 | Issue the navigation task |

[Notes] After receiving the request, and the robot will first execute the navigation task (successful/failed/cancelled) issued by user, then will send a response message to the client.

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 1003,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {             "Value": 0,             "MapID": 0,             "PosX": 0,             "PosY": 0,             "PosZ": 0,             "AngleYaw": 0,             "PointInfo": 0,             "Gait": 0,             "Speed": 0,             "Manner": 0,             "ObsMode": 0,             "NavMode": 0		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1003</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items>		<Value>0</Value>  		<MapID>0</MapID>  		<PosX>0.0</PosX>  		<PosY>0.0</PosY>  		<PosZ>0.0</PosZ>  		<AngleYaw>0.0</AngleYaw>  		<PointInfo>0</PointInfo>  		<Gait>0</Gait>  		<Speed>0</Speed>  		<Manner>0</Manner>  		<ObsMode>0</ObsMode>  		<NavMode>0</NavMode>  	</Items></PatrolDevice>
```

In this request message, the Items field contains several parameters which can be used for setting the navigation tasks.

[Notes]  The paramters are divided into two types: point parameter and path parameter. Point parameter defines the target point, such as the serial number <Value>, type <PointInfo>, position <PosX>/<PosY>/<PosZ>, orientation <AngleYaw> of the target point. Path parameter specifies robot's movement from current to target point, such as <Gait>, <Speed>, <Manner>, <ObsMode>, <NavMode> the robot used on the way to the target point.

The meaning of each parameter with their specific definitions is as follows:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Value | No. of the target point | int |  |
| MapID | No. of the grid map where the target point is located | int | No meaning at the moment |
| PosX/PosY/PosZ | Position of the target point in Map Coordinate System (m) | float |  |
| AngleYaw | Orientation of the target point in Map Coordinate System (rad) | float |  |
| PointInfo | Type of the target point[9] | int | Transition point = 0Task point = 1Charge point = 3 |
| Gait | Gait on the way to the target point[10] | int | Flat (Agile Motion Mode) = 0x3002Stair (Agile Motion Mode) = 0x3003 |
| Speed | Velocity on the way to the target point | int | Normal speed = 0Low speed = 1High speed = 2 |
| Manner | Mode of motion on the way to the target point | int | Walk forward = 0Walk backward = 1 |
| ObsMode | Whether to enable the navigation module to avoid obstacles when arriving at the target point[11] | int | Enable = 0Disable = 1 |
| NavMode | Navigation mode on the way to the target point[12] | int | Straight = 0 Auto = 1 |

[9] There are four types of target points: transition point, task point and charge point.

1.Transition point: Only used to overcome terrain and constrain paths with low positioning accuracy.

2.Task point: With high precision in reaching the designated point. Once the robot arrives at the task point, it will stop moving to maintain its position for executing tasks such as recognizing meters during inspections, until receiving the request for next target point.

3.Charge point: Only used to identify and locate the reflective piles in front of the charging dock. Once the robot arrives at the charge point, it will autonomously enter the charging dock and keep charging until receiving the next target point.

[10] For the gaits that can be performed in navigation:

1.Flat Gait: Suitable for flat surfaces, grass terrain, and supports climbing slopes up to ≤30°;

2.Stair Gait: Suitable for traversing obstacles such as slopes (≤30°) and stairs (≤25 cm).

[11] Whether the obstacle avoidance function of the navigation module is enabled or not, the obstacle avoidance function based on the local elevation map, as a safety module of the robot itself, will always remain enabled. When the obstacle avoidance function of the navigation is enabled, for Straight navigation, the robot will stop at an obstacle, and for Auto navigation, it behaves as obstacle avoidance.

[12] When using the Straight navigation, if two points cannot be planned globally in a straight line, the navigation task will fail. When using the Auto navigation, the path points should be set based on terrain complexity. The higher the terrain complexity, the higher the execution risk of the navigation path, and in this case, the more intensive target points should be set. Try to use multi-segment Straight navigation instead of Auto navigation.


![Navigation Stair Diagram](https://alidocs.dingtalk.com/core/api/resources/img/5eecdaf48460cde58272ab7060997fd277fce252ea93ed5575b8339e1c4c24830dd9a450996693588d68742cd653602a822951806323f8966c4b1b4e1186e4d2685c031c553150705a08c28babb53c43a7bfd6872089aaf1d69a21d0394542d6)


[Notes] When using Straight NavMode to control the robot climbing stairs , it is recommended to set the target point at least 45 cm beyond the edge of the last stair step. Additionally, the navigation system's ObsMode must be disabled.[Notes] When using Auto NavMode, only Flat gait and Low Speed mode are supported.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 1003,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"Value":0,			"Status":0,			"ErrorCode":0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1003</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:01</Time>	<Items>		<Value>0</Value>		<Status>0</Status>		<ErrorCode>0</ErrorCode>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Value | No. of the target point in navigation task | int | Correspond to the request for issuing the navigation task |
| Status | Execution state of navigation task | int | Idle = 0Exiting charging station = 1Navigation preprocessing = 2Navigating = 3Navigation complete = 4Entering charging dock = 5Paused = 0xff |
| ErrorCode | Error code | / | Check the table below |

The meaning of the value of ErrorCode is shown in the following table:

| Value | Meaning | Value | Meaning |
| --- | --- | --- | --- |
| 0 | Default value | 0xA328 | Failed to switch to navigation mode |
| 0x2300 | Single point navagation task cancelled | 0xA341 | Failed to issue a new task while currently executing a task |
| 0x2302 | Single point navagation task executed successfully | 0xA343 | Failed to exit from charging dock |
| 0xA301 | Abnormal motion state(soft stop or fall) | 0xA34B | Persistent stop at an obstacle(over 30s) |
| 0xA302 | Battery level below 20% | 0xA34C | Navigation global planning failure |
| 0xA303 | Motor overtemperature | 0xA34D | Continuous navigation speed not updated, navigation failed |
| 0xA312 | Navigation module communication is abnormal, unable to issue tasks | 0xA34E | Failure to issue a task during autonomous charging process |
| 0xA313 | Location abnormal (over 30s) |  |  |


### 1.4.5 Cancel Navigation Task

You can cancel the navigation task that is being executed currently by this request, even if it terminates.

| Type | Command | Message type |
| --- | --- | --- |
| 1004 | 1 | Cancel the navigation task |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 1004,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1004</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items/></PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 1004,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"ErrorCode":0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>	<Type>1004</Type>	<Command>1</Command>	<Time>2023-01-01 00:00:01</Time>	<Items>		<ErrorCode>1</ErrorCode>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| ErrorCode | Error code | int | Success = 0Failure = 1 |


### 1.4.6 Query the Execution State of Navigation Task

You can query the execution state of navigation task by this request.

| Type | Command | Message type |
| --- | --- | --- |
| 1007 | 1 | Query execution state of navigation task |

Request

JSON request:

```
{	"PatrolDevice": {		"Type": 1007,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {		}	}}
```

XML request:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1007</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:00</Time>	<Items/>  </PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

Response

JSON response:

```
{	"PatrolDevice": {		"Type": 1007,		"Command": 1,		"Time": "2023-01-01 00:00:00",		"Items": {			"Value":0,			"Status":0,			"ErrorCode":0		}	}}
```

XML response:

```
<?xml version="1.0" encoding="UTF-8"?><PatrolDevice>  	<Type>1007</Type>	<Command>1</Command>  	<Time>2023-01-01 00:00:01</Time>	<Items>		<Value>0</Value>		<Status>0</Status>		<ErrorCode>0</ErrorCode>	</Items></PatrolDevice>
```

In this response message, the items field contains the following parameters:

| Parameters | Meaning | Type | Value |
| --- | --- | --- | --- |
| Value | No. of the target point in navigation task | int | Correspond to the request for issuing the navigation task |
| Status | Execution state of navigation task | int | Idle = 0Exiting charging station = 1Navigation preprocessing = 2Navigating = 3Navigation complete = 4Entering charging dock = 5Paused = 0xff |
| ErrorCode | Error code | / | Check the table below |

The meaning of the value of ErrorCode is shown in the following table:

| Value | Meaning | Value | Meaning |
| --- | --- | --- | --- |
| 0 | Default value | 0xA328 | Failed to switch to navigation mode |
| 0x2300 | Single point navagation task cancelled | 0xA341 | Failed to issue a new task while currently executing a task |
| 0x2302 | Single point navagation task executed successfully | 0xA343 | Failed to exit from charging dock |
| 0xA301 | Abnormal motion state(soft stop or fall) | 0xA34B | Persistent stop at an obstacle(over 30s) |
| 0xA302 | Battery level below 20% | 0xA34C | Navigation global planning failure |
| 0xA303 | Motor overtemperature | 0xA34D | Continuous navigation speed not updated, navigation failed |
| 0xA312 | Navigation module communication is abnormal, unable to issue tasks | 0xA34E | Failure to issue a task during autonomous charging process |
| 0xA313 | Location abnormal (over 30s) |  |  |

[Notes]  If the query shows that the navigation task cannot be executed, please check whether the navigation program has not been started or it is abnormal.


# 2 ROS2 Topics

This section describes ROS2 topics in Lynx M20 robot, some topics use custom ROS 2 message types.

[Notes] Before using ROS2 tools to receive topics, check frequencies, etc., you need to source the environment variables with: source /opt/robot/scripts/setup_ros2.sh.[Notes] Before using ROS 2 tools to subscribe to topics, check message frequencies, etc., it is recommended to obtain administrator privileges via su to ensure that the relevant commands can be executed properly.


## 2.1 Topics of Sensor Driver

The topics in this section are all published to the topics upon robot power-up, and developers can subscribe to topics for sensor data.

| Topic Name | Description | Message Type | Frequency |
| --- | --- | --- | --- |
| /IMU | Generic format IMU data topic | sensor_msgs/msg/Imu | 200Hz |
| /LIDAR/POINTS | Generic format point cloud data topic | sensor_msgs/msg/PointCloud2 | 10Hz |

[Notes] The /LIDAR/POINTS topic depends on the multicast-relay.service service. If the topic has no data, please refer to Appendix 2 to check the service status and enable or disable the corresponding service as needed.[Notes]  You must obtain administrator (root) privileges via su to access data from the /LIDAR/POINTS topic.[Notes]  The /LIDAR/POINTS topic is restricted to transmission only by the robot’s host. If external access is required, please contact technical support.


## 2.2 Motion-related Topics

[Notes] The topics in this section depend on the rl_deploy service. If the topic message does not respond or topic messages cannot be received, please refer to Appendix 2 to check the service status and enable or disable the corresponding services as needed.


### 2.2.1 Motion State Switching

The topic is activated when the motion program is started, and developers can publish messages to the topic to switch the motion state.

| Topic Name | Description | Message Type |
| --- | --- | --- |
| /MOTION_STATE | motion state[1] switching command | drdds/msg/MotionState |

The message type drdds/msg/MotionState for the topic /MOTION_STATE is a custom message type that contains drdds::msg::MetaType and drdds::msg::MotionStateValue:

```
MetaType header  uint64 frame_id  Timestamp timestamp    int32 sec    uint32 nsecMotionStateValue data      int32 state
```

The state field of the topic message that needs to be published to set different motion states is shown in the table below:

| State to Be Set | Field Value to Be Published |
| --- | --- |
| Idle | 0 |
| Stand | 1 |
| Soft Emergency Stop | 2 |
| Power-on Damping | 3 |
| Sit | 4 |
| RL Control | 17 |

[1] The conversion relationship of the robot's motion state is shown in the figure below:


![Motion State Transition Diagram](https://alidocs.dingtalk.com/core/api/resources/img/5eecdaf48460cde58272ab7060997fd277fce252ea93ed5575b8339e1c4c24830dd9a450996693588d68742cd653602aaac77b6f1abe50dfc7620c6de82fbe3f8e4098dd4b01b6148d74310b6e580070a7dc930af018efdcade83a9d385200d2)


[Notes] In the RL control state, there are two modes: standard motion mode and agile motion mode. The standard motion mode is suitable for manual control, while the agile motion mode offers better gait speed response performance and is suitable for navigation and other autonomous algorithm development.

Developers who want to subscribe to the topic in their own programs need to define the structure of this custom message type in codes.


### 2.2.2 Gait Switching

The topic is activated when the motion program is started, and developers can publish messages to the topic to switch the gait.

| Topic Name | Description | Message Type |
| --- | --- | --- |
| /GAIT | gait switching command | drdds/msg/Gait |

The message type drdds/msg/Gait for the topic /GAIT is a custom message type that contains drdds::msg::MetaType and drdds::msg::GaitValue:

```
MetaType header  uint64 frame_id  Timestamp timestamp    int32 sec    uint32 nsecGaitValue data      uint32 gait
```

The gait field of the topic message that needs to be published to switch gait is shown in the table below:

| Gait to Be Set | Field Value to Be Published |
| --- | --- |
| Flat Gait (Agile Moton Mode) | 0x3002 |
| Stair Gait (Agile Moton Mode) | 0x3003 |

Developers who want to subscribe to the topic in their own programs need to define the structure of this custom message type in codes.


### 2.2.3 Obtain Basic Motion State and Gait

This topic is published to the topics upon robot power-up, and developers can subscribe to this topic to obtain basic motion status and gait information.

| Topic Name | Description | Message Type | Frequency |
| --- | --- | --- | --- |
| /MOTION_INFO | Basic motion state and gait of the robot | drdds/msg/MotionInfo | 20Hz |

The message type drdds/msg/MotionInfo for the topic /MOTION_INFO is a custom message type that contains drdds::msg::MetaType and drdds::msg::MotionInfoValue:

```
MetaType header  uint64 frame_id  Timestamp timestamp    int32 sec    uint32 nsecMotionInfoValue data  float32 vel_x  float32 vel_y  float32 vel_yaw   float32 height  MotionStateValue data    int32 state  GaitValue data    uint32 gait  float32 payload  float32 remain_mile
```

The detailed definitions of the fields in the message are shown in the table below:

| Field Name | Meaning | Type | Field Value Description |
| --- | --- | --- | --- |
| vel_x | Forward/Backward Movement Speed(m/s) | float32 | Positive and negative values represent forward and backward speeds respectively |
| vel_y | Left/Right Movement Speed(m/s) | float32 | Positive and negative values represent left and right movement speeds respectively |
| vel_yaw | Horizontal Steering Speed(rad/s) | float32 | Positive and negative values represent counterclockwise and clockwise rotation speeds respectively |
| height | The height of the robot (m) | float32 |  |
| state | The motion state of the robot | int32 | Idle = 0Stand = 1Soft Emergency Stop = 2Power-on Damping = 3Sit = 4RL Control = 17 |
| gait | The gait of the robot | uint32 | Basic (Standard Motion Mode) = 0x1001Flat (Agile Moton Mode) = 0x3002Stair (Agile Motion Mode) = 0x3003 |
| payload | Invalid parameter | / |  |
| remain_mile | Estimated remaining range (km) | float32 |  |

[Notes] The standard motion mode is suitable for manual control. The agile motion mode offers better gait speed response performance and is suitable for navigation and other autonomous algorithm development.

Developers who want to subscribe to the topic in their own programs need to define the structure of this custom message type in codes.


## 2.3 Velocity-related Topics


### 2.3.1 Issue velocity commands for navigation planning

[Notes] When publishing this topic, potential conflicts may arise with the robot's built-in planner service. Please refer to Appendix 2 to disable the planner service.This topic also depends on the basic_server service. If the velocity command does not take effect, please refer to Appendix 2 to check the service status and start or stop the corresponding service as needed. Furthermore, publishing this topic may conflict with the charge_manager service used by the robot's autonomous charging program. Avoid publishing this topic while the robot is executing an autonomous charging task, or refer to Appendix 2 to disable the charge_manager service to deactivate the autonomous charging function. [Notes] This velocity topic is only effective when the robot is in navigation mode, and it is recommended to publish it at a fixed frequency of 10 Hz.[Notes] During development, if the robot’s built-in localization service is not required, you can refer to Appendix 2 to stop or disable the localization service to save resources.

Developers can publish messages to the topic to issue velocity commands for navigation planning to control the robot's movement; and subscribe to the topic to obtain the velocity commands issued by the robot’s own navigation system.

| Topic Name | Description | Message Type |
| --- | --- | --- |
| /NAV_CMD | Issue velocity commands for navigation planning | drdds::msg::NavCmd |

The message type drdds/msg/NavCmd for the topic /NAV_CMD is a custom message type that contains drdds::msg::MetaType and drdds::msg::NavCmdValue:

```
MetaType header  uint64 frame_id  Timestamp timestamp    int32 sec    uint32 nsecNavCmdValue data  float32 x_vel       float32 y_vel      float32 yaw_vel
```

The meanings of the fields corresponding to the message content are shown in the table below:

| Field | Meaning | Type | Field Value Description |
| --- | --- | --- | --- |
| x_vel | Forward/Backward Movement Speed(m/s) | float32 | Positive and negative values represent forward and backward speeds respectively |
| y_vel | Left/Right Movement Speed(m/s) | float32 | Positive and negative values represent left and right movement speeds respectively |
| yaw-vel | Horizontal Steering Speed(rad/s) | float32 | Positive and negative values represent counterclockwise and clockwise rotation speeds respectively |

The positive directions for the robot's movement and rotation are shown in the diagram below:

Developers who want to subscribe to the topic in their own programs need to define the structure of this custom message type in codes.


# Appendix 1: UDP Sample Code

```cpp
#include <iostream>#include <cstring>#include <sys/socket.h>#include <arpa/inet.h>#include <unistd.h>#define SERVER_IP "10.21.31.103"#define PORT 30000#define BUFFER_SIZE 1024struct udpMessage{    unsigned char header[16];    unsigned char data[BUFFER_SIZE];};int main() {    // Create a UDP socket    int client_fd = socket(AF_INET, SOCK_DGRAM, 0);    if (client_fd < 0) {        perror("socket creation failed");        return -1;    }    // Set server address    struct sockaddr_in server_addr;    server_addr.sin_family = AF_INET;    server_addr.sin_port = htons(PORT);    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {        perror("Invalid address/ Address not supported");        close(client_fd);        return -1;    }    udpMessage message;
```


# Appendix 2: Robot Service Status Inquiry and Management Method

●Query service availability status

sudo systemctl status XXX.service   # XXX is the service name

●Temporarily stop service

sudo systemctl stop XXX.service     # XXX is the service name

●Temporarily start service

sudo systemctl start XXX.service    # XXX is the service name

●Disable service (remove auto-start at boot)

sudo systemctl disable XXX.service  # XXX is the service name

●Enable service (auto-start at boot)

sudo systemctl enable XXX.service   # XXX is the service name

[Notes] After each OTA update, the auto-start settings of all services will be reset to factory defaults. You will need to execute the disable command again to disable the corresponding services.