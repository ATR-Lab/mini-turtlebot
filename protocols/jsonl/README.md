# Mini-TurtleBot JSONL Protocol (v0.1)

This folder defines the wire protocol used between PC-side bridges (ROS 2 Bridge, Scratch Bridge) and the robot-side Control JSONL Server.

The architecture expects a Control JSONL Server on the robot and WebSocket clients on the PC side, with transport over WebSocket / TCP / UDP. The message format is JSONL (one JSON object per line).  (See architecture diagram.)

## 1) Message framing (JSONL)

- Encoding: UTF-8.
- Framing: **one JSON object per line**, newline separated (`\n`).
- No pretty printing, no multi-line JSON.
- Each line MUST be a valid JSON object.
- Max line length (recommended): 1024 bytes for control messages.
- Unknown fields MUST be ignored (forward compatibility).

### Example line
{"v":1,"robot_id":"tb_01","type":"cmd","cmd":"vel","linear":0.10,"angular":0.00}

## 2) Common fields (shared by all messages)

Required:
- `v` (int): protocol version. Start with `1`.
- `type` (string): `"cmd"`, `"state"`, `"ack"`, `"err"`.
- `robot_id` (string): unique ID, e.g. `"tb_01"`.

Recommended:
- `seq` (int): monotonically increasing message id per sender.
- `ts_ms` (int): sender timestamp in milliseconds (optional).
- `src` (string): `"ros2"`, `"scratch"`, `"cli"`, `"fw"` etc.
- `priority` (int): 0 (highest) .. 9 (lowest). Suggested:
  - 0 = estop
  - 2 = safety
  - 4 = ROS2 Nav/Teleop
  - 6 = Scratch
  - 8 = demos

Swarm fields (optional but supported from day 1):
- `group` (string): e.g. `"alpha"` (if targeting a group)
- `broadcast` (bool): if true, receiver may fan-out locally (PC side)

**Routing rule**
- If `robot_id` is present: message targets that robot.
- If `group` is present (and `robot_id` absent): message targets all robots in that group (handled on PC bridge side).
- Robot firmware only needs to honor `robot_id` (ignore `group`).

## 3) Units & coordinate convention

Velocity command uses ROS convention:
- `linear` is forward velocity in **m/s** (base_link +X).
- `angular` is yaw rate in **rad/s** (base_link +Z).

Positive `angular` = CCW rotation when viewed from above.

## 4) Command messages (`type: "cmd"`)

All commands are JSON objects with:
- `type: "cmd"`
- `cmd: <string>`

### 4.1 Velocity command (minimum for ROS2 bring-up)

Fields:
- `cmd`: `"vel"`
- `linear` (float, m/s)
- `angular` (float, rad/s)

Example:
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"vel","linear":0.12,"angular":-0.30,"seq":12,"src":"ros2","priority":4}

### 4.2 Stop command (semantic stop)
Equivalent to vel with zeros, but explicit.

Example:
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"stop","seq":13,"src":"ros2","priority":2}

### 4.3 Emergency stop (latched until cleared)
- Robot must stop motors immediately and ignore non-estop motion commands while latched.

Example (engage):
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"estop","enabled":true,"seq":14,"src":"ros2","priority":0}

Example (clear):
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"estop","enabled":false,"seq":15,"src":"ros2","priority":0}

### 4.4 LED command (optional)
Example:
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"led","id":1,"state":true,"seq":20,"src":"scratch","priority":6}

### 4.5 Servo command (optional)
Servo position:
- `deg` 0..180 (or define your range)

Example:
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"servo","id":1,"deg":90,"seq":21,"src":"scratch","priority":6}

### 4.6 Ping (connectivity test)
Example:
{"v":1,"type":"cmd","robot_id":"tb_01","cmd":"ping","seq":99,"src":"cli","priority":8}

## 5) Acknowledgements (`type: "ack"`) and errors (`type: "err"`)

ACK is optional (recommended for ping / configuration commands).
- `ack_seq` is the seq being acknowledged.

Example:
{"v":1,"type":"ack","robot_id":"tb_01","ack_seq":99,"ok":true,"ts_ms":123456789}

Error:
- `code` short string, `msg` human readable.

Example:
{"v":1,"type":"err","robot_id":"tb_01","code":"bad_cmd","msg":"Unknown cmd: foo","ack_seq":42}

## 6) State messages (`type: "state"`)

Robot may periodically publish state over the same connection.

### 6.1 Heartbeat (recommended 2–5 Hz)
Example:
{"v":1,"type":"state","robot_id":"tb_01","state":"heartbeat","uptime_ms":53012,"estop":false}

### 6.2 Battery (recommended 1 Hz)
- `v_bat` volts
- `pct` 0..100 (if estimated)

Example:
{"v":1,"type":"state","robot_id":"tb_01","state":"battery","v_bat":3.98,"pct":71}

### 6.3 Odometry (optional; later)
If you publish odom from firmware:
- `x,y,theta` meters / radians in local frame
- `vx,wz` current velocities

Example:
{"v":1,"type":"state","robot_id":"tb_01","state":"odom","x":1.20,"y":-0.30,"theta":0.52,"vx":0.08,"wz":0.00}

### 6.4 IMU (optional; later)
Example:
{"v":1,"type":"state","robot_id":"tb_01","state":"imu","ax":0.01,"ay":0.02,"az":9.80,"gz":0.00}

## 7) Safety defaults

- If no `vel` or `stop` received for `cmd_timeout_ms` (default 500 ms), robot should stop motors.
- `estop` overrides everything.
- Invalid JSON line: ignore + optionally send `type:"err"`.

## 8) Versioning

- Increment `v` only for breaking changes.
- New fields are additive and must be ignored by old receivers.

