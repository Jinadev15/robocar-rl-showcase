# Robocar: RL navigation on motor PWM with local sensing — system design

Status: design, not implemented. Version 3, 2026-11-17.
History: v1 assumed a known map; v2 added local sensing and a two-run
(Micromouse) mission; v3 follows the team's answers: a continuous GPS-like world,
one run, low latency over exact optimality, ROS 2 Lyrical, and the parts list.

## 1. The task

1. An RL model that outputs **motor PWM**, not discrete moves.
2. It drives a **ROS 2 rover**, first in **simulation** of a real-life-like,
   GPS-style world (a destination given as coordinates).
3. It finds a route **close to the shortest** and drives it, with **as little
   latency as possible**. Exact optimality is not needed: the bar is "good enough
   for an autonomous car" (the team's previous approach: greedy best-first,
   compared with a near-shortest path).
4. The rover **does not know the world**. It senses only obstacle distance
   right / front / left (3 x HC-SR04), heading (HMC5883L) and wheel odometry (N20
   encoders), and must localise itself and map from that ("pre-SLAM").
5. Bonus: a Python script that visualises the route.

Long-term (design for it now, build later): an autonomous **warehouse bot**;
destinations set by **gesture or voice**, "pinned" in the world.

### Acceptance criteria (proposed)

| # | Criterion | How measured (simulation, random worlds, randomised rover) |
|---|---|---|
| A1 | Motors are driven only by the RL model's PWM pair | `/wheel_pwm` is the only motor command |
| A2 | Reaches the destination | ≥ 95% of runs |
| A3 | Route close to the shortest | driven length / A* length on the true world: median ≤ 1.3, 90th percentile ≤ 1.6; and shorter than greedy best-first on the same worlds |
| A4 | Low latency | decision latency (sensor reading in → PWM out) ≤ 20 ms on the laptop CPU, 95th percentile |
| A5 | No collisions | 0 contacts across the test set |
| A6 | Localisation | final position error ≤ 10 cm over a 10 m drive |
| A7 | Visualiser | built map, planned route, driven trail, true world and A* route (sim) |

## 2. Architecture

```
 3 x HC-SR04 ranges ─┐   HMC5883L heading ─┐   N20 encoder ticks ─┐
                     ▼                     ▼                      ▼
 ┌─────────────────────────────────────────────────────────────────────┐
 │ LOCALISATION + MAPPING ("pre-SLAM")                       50 Hz      │
 │ EKF: encoders predict, magnetometer corrects heading,                │
 │      ranges to mapped obstacles correct position                     │
 │ occupancy grid, 5 cm, log-odds, along each ultrasonic cone           │
 └───────────────┬─────────────────────────────────────┬───────────────┘
   pose, local map window                        pose, ranges
                 ▼                                      ▼
 ┌───────────────────────────────┐   waypoint  ┌──────────────────────────┐
 │ L1 ROUTE POLICY (RL)   5-10 Hz │ ──────────▶ │ L2 MOTION POLICY (RL)    │
 │ egocentric local map + goal    │             │ SAC, 20-50 Hz            │
 │ direction → next waypoint      │             │ → left / right PWM       │
 │ unknown space = open (optimism)│             │ ranges keep it off walls │
 └───────────────────────────────┘             └────────────┬─────────────┘
          ▲  destination ("pin", world coordinates)          │ /wheel_pwm
          │  RViz click today; voice / gesture later         ▼
                                        ESP32 + micro-ROS → TB6612FNG → N20s
```

### One run, replanning as it goes

The rover drives straight towards the destination through unknown space,
treating it as open. When the ultrasonics find an obstacle, the map changes and
the next decision accounts for it. This is how D* Lite navigation and
autonomous cars behave: plan on what is known, assume the rest is drivable,
replan on new information. It has no search run, so the first route is not
always the shortest, but it is close and has no start-up delay (A3).

### Why the route policy sees a local window

Latency (A4). The planner network does one propagation step per cell of its
input, so a large world map would mean thousands of steps per decision. L1
instead sees an **egocentric window**: for example 32 x 32 cells of 10 cm (3.2 m
square) centred on the rover and turned to its heading, with the destination
outside the window drawn on the window border in its true direction (with its
distance as a separate input). Cost per decision is then fixed, whatever the
world size. Far-away structure the rover has already mapped beyond the window
is kept in the world map; a coarse global view (one cell per 0.5 m, same
network) can be added if dead ends larger than the window turn up in testing
(risk R4).

### Why two RL levels

Route choice changes a few times a second; PWM must change 20-50 times a second.
Split, each policy has a short horizon and a direct reward, is trained and tested
on its own, and the fast loop (L2, a small MLP, < 1 ms) never waits for the
slower one.

## 3. Hardware (team parts list)

| Part | Model | Design notes |
|---|---|---|
| Motors x2 | GA12-N20 with hall encoder, 3 PPR | counts per wheel turn = 3 x 4 (quadrature) x gear ratio; **gear ratio needed** (e.g. 1:100 → 1200) |
| Wheels | N20, 44 mm | radius 0.022 m; **track width needed** |
| Range x3 | HC-SR04 (left, front, right) | 5 V; echo through a 1k/2k divider to 3.3 V; fire in turn, ≥ 25 ms apart → ~13 Hz per sensor; ~15° cone; misses at shallow angles; 2 cm minimum |
| Heading | HMC5883L | I2C 0x1E. Many boards sold as HMC5883L carry a QMC5883L (0x0D, other registers): check with an I2C scan. Mount away from the motors; hard/soft-iron calibration |
| Driver | TB6612FNG (DRV8871 alternative: one per motor) | PWM 20 kHz; 1.2 A per channel, enough for N20s; STBY pin as a hardware stop |
| MCU | ESP32 DevKit V1 | LEDC PWM, PCNT encoders, micro-ROS over WiFi |

The existing CAD (DESIGN/) is the TT-motor kit; the N20 build changes the wheel
radius, track and footprint, which the simulator takes as parameters.

## 4. Components

### 4.1 Simulator `robocar_rl.rover_sim` (new)

Continuous 2-D world, vectorised NumPy, thousands of episodes a minute:

- Worlds: random warehouse-like layouts (aisles, shelves, walls, rooms, open
  floor, scattered boxes) and maze-like ones; start pose and destination random,
  far apart.
- Diff-drive kinematics; N20 motor model: dead band, lag, duty x battery
  voltage, left/right mismatch.
- HC-SR04 model: cone ray cast, noise, max range, specular miss at shallow
  angles, dropouts, sequential firing.
- HMC5883L model: heading + bias + noise + position-dependent distortion.
- Encoders: 3 PPR quantisation through the gear ratio, slip.
- WiFi delay 10-60 ms.
- All randomised per episode.

### 4.2 Localisation and mapping `robocar_rl.estimation` (new)

- EKF on (x, y, θ): encoder prediction; magnetometer heading update; range
  update against already-mapped obstacles (expected vs measured distance).
- Occupancy grid, 5 cm, log-odds along each ultrasonic cone (free inside the
  measured range, occupied at the arc).
- World frame: the start pose is the origin, destinations are given in it
  ("pins"). A real GPS-style fix, or a camera fix, can later be one more EKF
  update with no other change.

### 4.3 L1 route policy (RL)

- Network: the existing gated planner DQN, retrained on egocentric windows.
- Observation per window cell: occupied, free, unknown, visited, passage memory;
  plus destination direction and distance.
- Action: one of 8 neighbouring cells, turned into a waypoint for L2.
- Training: first on the grid version of the simulator (fast), then on windows
  cut from the continuous simulator's live map.
- Reward: progress to the destination, step cost, revisit / passage-reuse
  penalties (in the code already), large reward at the destination.
- Route memory (Trémaux, in the code) on top, so it cannot loop: the hybrid
  that solved 100% of the hard mazes.

### 4.4 L2 motion policy (RL)

| | |
|---|---|
| Algorithm | SAC (stable-baselines3) |
| Observation | next 2 waypoints in the robot frame; heading error; forward and turning speed; wheel speeds; 3 ranges; previous PWM |
| Action | left, right duty in [-1, 1] |
| Reward | progress; − deviation from the waypoint line; − PWM change; + waypoint reached; − time; large − on contact |
| Curriculum | straight → turns → narrow aisles → full worlds |

### 4.5 Baselines (same worlds, same sensors)

- **Greedy best-first** (the team's previous approach) on the live map.
- **A\*** on the live map with unknown space as open, replanning (D* Lite behaviour).
- **A\*** on the true world: the shortest possible, for A3.

The report compares route length, success and decision latency of all of them
with the RL rover.

## 5. ROS 2 (Lyrical Luth)

| Node | Subscribes | Publishes | Rate |
|---|---|---|---|
| `esp32` (micro-ROS) / `rover_sim` | `/wheel_pwm` | `/wheel_ticks`, `/range/{left,front,right}` (`sensor_msgs/Range`), `/mag` (`sensor_msgs/MagneticField`) | ticks 50 Hz, ranges ~13 Hz, mag 50 Hz |
| `localiser` | ticks, ranges, mag, `/map` | `/pose` (`PoseWithCovarianceStamped`), TF `map → base_link` | 50 Hz |
| `mapper` | ranges, `/pose` | `/map` (`nav_msgs/OccupancyGrid`) | 10 Hz |
| `route_policy` (L1) | `/map`, `/pose`, `/goal_pose` | `/waypoint`, `/route` (`nav_msgs/Path`) | 5-10 Hz |
| `motion_policy` (L2) | `/waypoint`, `/pose`, ranges, ticks | `/wheel_pwm` (`robocar_msgs/WheelPwm`) | 20-50 Hz |
| `mission_manager` | all | `/mission/state`, `/estop` | 10 Hz |
| `sim_truth` (sim) | – | `/sim/true_pose`, `/sim/world` | – |

- **Destinations**: `/goal_pose` (`geometry_msgs/PoseStamped`), the topic RViz's
  "2D Goal Pose" tool publishes. A voice or gesture node later publishes the same
  topic; a `/pins` service can store named places ("dock A") in the map frame.
- **Latency**: L1 and L2 in one process with intra-process communication; the
  latency from a sensor message to `/wheel_pwm` is logged (A4).
- **Safety on the ESP32**: 250 ms command watchdog → TB6612 STBY low; duty clamp
  and ramp; front range < 8 cm → stop.

## 6. Visualiser (bonus)

`scripts/visualize_route.py`, live from ROS topics or from a recorded bag:
built occupancy map, pose with uncertainty ellipse, driven trail, current route;
in simulation also the true world and the A* and greedy routes, with lengths and
latency in the title. PNG export. RViz config for the same.

## 7. Environment

ROS 2 is a system package, not on PyPI, so `uv` cannot install it alone. System
ROS 2 Lyrical runs on Python 3.14, so:

```bash
uv venv .venv-ros --python /usr/bin/python3.14 --system-site-packages
```

with `torch`, `stable-baselines3` and this project (`-e .`) in it. Training stays
in `.venv` (Python 3.12). Missing ROS pieces (micro-ROS agent, possibly built from
source for Lyrical) need `sudo apt` by you. Firmware: PlatformIO + micro-ROS.

## 8. Repository layout (to be created)

```
src/robocar_rl/rover_sim/      # continuous simulator: worlds, kinematics, motors, sensors
src/robocar_rl/estimation/     # EKF localiser, occupancy mapper
src/robocar_rl/navigation/     # L1 egocentric route env + training; baselines
src/robocar_rl/motion/         # L2 SAC training and evaluation
ros2/robocar_msgs/             # WheelPwm.msg
ros2/robocar_ros/              # nodes, launch (sim, real), config, RViz
firmware/esp32_microros/       # PlatformIO project
scripts/visualize_route.py     # bonus
```

## 9. Build order (simulation first)

| Phase | Work | Done when |
|---|---|---|
| 0 | `.venv-ros`; a node imports `rclpy`, `torch`, `robocar_rl` | runs |
| 1 | Simulator: worlds, kinematics, N20 motors, HC-SR04, HMC5883L, encoders | geometry tests pass |
| 2 | Localiser + mapper with a scripted driver | A6; map matches the true world where seen |
| 3 | Baselines: greedy best-first, replanning A*, true-world A* | numbers to beat |
| 4 | L1 route policy on egocentric windows | A2, A3 on grid worlds; latency measured |
| 5 | L2 SAC motion policy | waypoint driving, 0 contacts under randomisation |
| 6 | ROS 2 nodes, launch, visualiser; whole mission in simulation | A1-A7 from one launch command |
| 7 | Firmware + bench test; calibration; simulator refit | real rover drives in a test area |

## 10. Risks

| # | Risk | Mitigation |
|---|---|---|
| R1 | HMC5883L distorted by motors and indoor steel; or the board is a QMC5883L | I2C scan; mount away; calibrate; EKF weighs range corrections; heading noise randomised in training |
| R2 | HC-SR04 misses and cross-talk | sequential firing; log-odds averaging; misses in the simulator |
| R3 | Odometry drift (3 PPR is coarse before the gearbox) | quadrature on both edges; EKF; range corrections |
| R4 | Dead ends larger than the local window | world map keeps them; passage memory + route memory; coarse global layer if needed |
| R5 | Sim-to-real gap | randomisation; calibration and refit (phase 7) |
| R6 | WiFi latency | in training; watchdog; serial fallback |
| R7 | micro-ROS agent on Lyrical | build from source or run in Docker |

## 11. Default specification (until the team measures the real values)

Every value is a parameter (`RoverParams`, `WorldConfig`); the real numbers
replace these without code changes. Values marked *randomised* vary per training
episode in the given range.

| Item | Default | Randomised range | Source |
|---|---|---|---|
| Wheel diameter | 44 mm | ±3% | N20 44 mm wheel |
| Track (wheel centre to centre) | 120 mm | ±5 mm | typical N20 chassis (assumed) |
| Body | 150 x 130 mm, collision circle r = 85 mm | – | assumed |
| Gear ratio | 1:100 | – | common GA12-N20 (assumed) |
| Encoder | 3 PPR x 4 (quadrature) x 100 = 1200 counts per wheel turn | – | from the above |
| Free-running wheel speed at full duty | 300 RPM → 0.69 m/s | 0.5-0.8 m/s | GA12-N20 1:100 at 6 V |
| Motor dead band | 18% duty | 10-25% | typical small DC gear motor |
| Motor time constant | 0.10 s | 0.06-0.18 s | assumed |
| Left/right gain mismatch | 1.0 | ±10% | – |
| HC-SR04 range | 0.02-4.0 m, 15° cone | – | datasheet |
| HC-SR04 noise | σ = 0.5 cm + 1% | x0.5-2 | – |
| HC-SR04 echo lost if incidence > | 40° from the wall's normal | 30-55° | specular reflection |
| HC-SR04 firing | one sensor every 25 ms (L, F, R) | – | avoids cross-talk |
| Sensor mounts | front centre; left and right at ±90°, 50 mm forward of the axle | – | assumed |
| HMC5883L | heading σ = 1.5°, bias up to ±5°, local distortion up to ±8° | – | indoor, near motors |
| Control delay (WiFi) | 30 ms | 10-60 ms | – |
| World | 4 x 4 m to 8 x 8 m; walls, aisles, boxes; narrowest gap 0.35 m | – | indoor warehouse scale |
| Simulation step | 100 Hz | – | – |

## 12. Still to confirm with the team

1. N20 gear ratio and the chassis track width.
2. World scale and obstacle types for the final demo.

## 13. As built: acceptance criteria against the final results

The measured numbers are in [RESULTS.md](RESULTS.md). Default stack: A* routes,
SAC PWM policy, collision monitor, EKF, and an overhead camera fix when one is
mounted.

| # | Criterion | Result | Status |
|---|---|---|---|
| A1 | Motors driven only by the RL PWM pair | `/wheel_pwm` from the SAC policy. The collision monitor only scales forward duty down | met |
| A2 | ≥ 95% reach the goal | 93% in the 4 m camera arena, 90% in the arena without the camera, 75% in 4-8 m worlds | not met: closest in the camera arena |
| A3 | Route median ≤ 1.3, p90 ≤ 1.6 x shortest | arena + camera: 1.01 / 1.47. 4-8 m worlds: 1.10 / 1.80 | met in the arena. p90 not met in large worlds (drift) |
| A4 | PWM decision ≤ 20 ms p95 | 3-4 ms (SAC). A* replan 10-28 ms runs at 2 Hz, separate from the PWM loop | met |
| A5 | No collisions | 2-6 wall contacts per mission: low-speed touches, which the contact guard and the monitor stop and recover from | not met. Contact rate down from 18 to 2-6 per mission through G1-G6 |
| A6 | Position error ≤ 10 cm | 1 cm with the camera, 6 cm in the arena, 10 cm in large worlds | met (large worlds at the limit) |
| A7 | Visualiser | `scripts/visualize_route.py` (offline and live) and RViz config | met |

Changes from the original plan:

- A* is the default route planner. The RL route policy (`planner:=rl`) is kept
  as an option, because it is measurably behind A* (RESULTS.md, section 5).
- Electronics follow the team slides. A Raspberry Pi Pico does the motor and
  sensor I/O. An ESP32 is the micro-ROS Wi-Fi relay. The two talk over a UART
  protocol with CRC-8 and a 250 ms watchdog (`firmware/`).
- An overhead ArUco camera gives an absolute fix (team report). One 720p camera
  covers areas up to about 4 m on the short side.
