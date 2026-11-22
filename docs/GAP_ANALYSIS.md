# Gap analysis: Robocar vs established systems with the same objective

Objective (team slides and report): a small rover that senses its surroundings,
localises itself, avoids obstacles and navigates autonomously to a target, with
AI/ML in the loop, on N20 motors with encoders, 3 x HC-SR04, an HMC5883L, a
TB6612FNG, a Raspberry Pi Pico + ESP32, ROS 2 and an overhead webcam.

Reference systems compared against:

| Reference | What it is | Why it is the right yardstick |
|---|---|---|
| **Nav2** (Macenski et al., 2020) | the ROS 2 navigation stack | the professional standard for ROS 2 rovers: costmaps, planners, controllers, recoveries, collision monitor |
| **Tai, Paolo & Liu, 2017** "Virtual-to-real deep RL: continuous control of mobile robots for mapless navigation" | RL policy from 10 sparse range beams + goal to velocity commands, trained in simulation, run on a real robot | the closest published match to "RL that drives a rover from sparse range sensors" |
| **Elfes, 1987** "Sonar-based real-world mapping and navigation" | probabilistic occupancy grids from sonar | the classic method for mapping with ultrasonics |
| **Anderson et al., 2018** "On evaluation of embodied navigation agents" | SPL metric and evaluation protocol | how navigation results are reported |
| **Micromouse / overhead-camera robot arenas** (e.g. Duckietown's watchtowers) | an external camera giving an absolute pose | the report's "overhead webcam for global position" |

Every change below was measured on missions in unseen worlds with randomised
rovers (`scripts/evaluate_navigation.py`, `scripts/ablation.py`). A change is kept
only if nothing got worse; rejected attempts are listed too.

## 1. Gaps found and closed

| # | Gap (reference) | What was implemented | Measured effect |
|---|---|---|---|
| G1 | Nav2 **collision monitor**: slow-down/stop zones from live sensor data, applied after any controller | `navigation/safety.py: limit_duty` scales forward PWM by the front sonar (0.40 m to 0.12 m), keeps turning, never limits reversing | RL controller 38% -> 50% success, contacts 18.1 -> 15.7; pure pursuit 56% -> 59%, contacts 10.8 -> 7.9 (32 missions each) |
| G2 | Nav2 **inflation layer** (graded cost near walls) | `CostmapConfig.inflation_cost` rings 0.8/0.4/0.15 per 10 cm cell beyond the robot radius | routes keep to corridor centres; with G1 + G3: RL controller 38% -> 53% |
| G3 | Nav2 **recoveries** (clear costmap, spin, back up) | `Navigator`: on "no movement for 15 s" or "no route for 2 s": forget the map within 1 m, spin a full turn (re-sense), back off, replan; 3 attempts | neutral on its own (19/24 both), kept: no metric worse, and it is what turns a dead stop into a retry |
| G4 | Nav2's **lethal vs inflated** split: the hard limit is the robot footprint, preference comes from cost | costmap hard margin 0.04 -> 0 m, planning cells sampled at their centre instead of "any fine cell blocked" | **stuck missions eliminated**: 4 m arena 19 -> 22 / 24, 4-8 m worlds 18 -> 19 / 24. Cause found by tracing: a mapped wall's own 3-8 cm thickness plus 4 cm margin plus cell rounding closed 0.45 m maze corridors |
| G5 | Absolute pose source (report: overhead webcam; Duckietown watchtowers) | `vision/overhead.py`: ArUco marker on the rover, real OpenCV detector on rendered frames in simulation and on the webcam on the rover (`overhead_camera` node), homography calibration (`scripts/calibrate_overhead.py`), EKF `absolute_fix` with latency compensation and gating | localisation error 5.5 cm -> 0.8 cm (4 m arena, 20 missions). Measured limit: one 720p camera resolves the 11 cm marker only over areas up to ~4 m on the short side (detection 92% at 4 m, 58% at 5 m, 8% at 6 m) |
| G6 | Tai et al.: policy trained where it is deployed | motion policy v4 trained half on route following, half on **whole missions through the real Navigator** (live map, replanning, EKF drift) | missions with A*: v2 (route following only) 65% -> v4 77%, contacts 11.7 -> 6.5 |
| G7 | Reward design: potential shaping must not pay for standing still | route policy shaping changed from gamma-discounted (paid +0.001 per cell of distance per step for not moving) to plain distance difference | route DQN at 25k steps 23% -> 35% success; final 93.7% on 300 held-out worlds, route 1.22 x shortest |
| G8 | Anderson et al. evaluation | SPL added to every evaluation table | see section 4 and RESULTS.md |
| G9 | Hardware realism (team slides) | simulator on N20 12 V 200 RPM + 3S pack; policies randomised around it | models trained on the real motor speeds |
| G10 | Engineering parity with Nav2 deployments | URDF + robot_state_publisher, wheel joint states, modes auto/manual/stop, teleop (`/cmd_vel`), `/estop`, battery telemetry, Pico + ESP32 firmware split with a CRC'd UART protocol and heartbeat | tested over ROS 2 in simulation (manual 0.2 m/s -> 0.56 duty; e-stop -> 0) |

## 2. Tried and rejected (made something worse)

| Attempt (reference) | Result | Decision |
|---|---|---|
| Elfes probabilistic sonar beam model | arena 19 -> 18 / 24, 4-8 m worlds 17 -> 15 / 24 | kept as an option (`beam_model="elfes"`), off |
| EKF corrections from ranges to already-mapped walls (scan matching) | final error 12 cm -> 17 cm | off by default: the map is built from the same drifting pose; needs full SLAM |
| Fine-tuning SAC from the route-following model on missions | collapsed (42% -> 3%): critics trained on another reward | trained v4 from scratch on the mix instead |
| Visit-count penalty at decision time (maze model) | 16 x 16 single-route 53% -> 27% | replaced by route memory |

## 3. Gaps that remain (and why)

| Gap | Reference | Why not closed here |
|---|---|---|
| Loop closure / full SLAM | Cartographer, slam_toolbox | three sonar beams do not give scans to match; the overhead camera removes the need inside its view |
| Dynamic obstacles (people, other robots) | Nav2 + spatio-temporal voxel layer | worlds are static; needs moving obstacles in the simulator and tracking |
| Gazebo physics model | Gazebo (slides) | not installed on this machine (Lyrical); the 2-D simulator is calibrated to the parts and is what the policies trained on |
| Voice commands, object detection | slides "AI-based decision making", report "future scope" | need microphone/camera data and models; the `/goal_pose` interface is ready for a voice node |
| Hardware validation | all | needs the assembled rover: calibration, firmware build and flash (firmware/README.md) |

## 4. Where the system stands now (final evaluation, RESULTS.md)

| Setting | Best combination | Success | SPL | Route / shortest (median) | Position error |
|---|---|---|---|---|---|
| 4 m arena + overhead camera (60 missions) | A* + SAC | 93% | 0.86 | 1.01 | 1 cm |
| 4 m arena, onboard only (60) | A* + SAC | 90% | 0.82 | 1.04 | 6 cm |
| 4-8 m worlds, onboard only (100) | A* + SAC | 75% | 0.62 | 1.10 | 10 cm |

Against the references:

- **Nav2.** The layers it has are all present here: costmap with inflation,
  global planner, local controller, recoveries, collision monitor and lifecycle
  modes. The gaps that remain are in section 3.
- **Tai et al. (2017).** Their published success rates for mapless RL navigation
  with sparse range beams were in unstructured rooms with a lidar-like fan of 10
  beams. This rover has three sonar beams. Here the RL PWM policy beats the
  tuned classical controller on every setting (large worlds: 75% vs 68%).
