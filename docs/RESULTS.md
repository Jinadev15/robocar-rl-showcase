# Rover results (final, 2026-11-22)

All numbers come from `scripts/evaluate_navigation.py`, run on unseen worlds. Every
mission uses a randomised rover (motor gains, dead band, battery, wheel slip, sonar
dropouts, magnetometer bias) built around the team's real parts: N20 12 V 200 RPM
motors, a 3S pack, 3 x HC-SR04 and an HMC5883L. The rover starts with no map.
Raw data is in `results/navigation_eval*.json`.

Columns:

- **Success**: the rover stops within 0.25 m of the goal before the time limit.
- **SPL**: success weighted by path length (Anderson et al., 2018); 1.0 = always
  arrives, on the shortest path.
- **Route / shortest**: driven distance divided by the true shortest path
  (median, 90th percentile).
- **Position error**: EKF pose error at the end of the mission.
- **Plan p95 / PWM decision p95**: 95th-percentile time per route plan and per
  20 Hz motor command.

Models: `models/rover/route_dqn.pt` (L1 route policy) and `models/rover/motion_sac.zip`
(L2 PWM policy). Provenance is in `models/rover/card.json`.

## 1. 4 m arena with the overhead camera (60 missions)

This is the team's set-up: a webcam above the arena tracks an ArUco marker on the rover.

| Route planner | Motion controller | Success | SPL | Route / shortest | Wall contacts | s per metre | Position error |
|---|---|---|---|---|---|---|---|
| A* | **SAC (RL PWM)** | **93%** | **0.86** | 1.01, 1.47 | 3.7 | 4.2 | 1 cm |
| A* | pure pursuit | 90% | 0.86 | 0.96, 1.14 | 3.1 | 5.2 | 2 cm |
| greedy best-first | pure pursuit | 90% | 0.84 | 0.95, 1.37 | 4.5 | 5.8 | 2 cm |
| **RL route (DQN)** | **SAC (RL PWM)** | 83% | 0.70 | 1.15, 1.53 | 3.0 | 4.5 | 1 cm |

## 2. 4 m arena, onboard sensors only (60 missions)

| Route planner | Motion controller | Success | SPL | Route / shortest | Wall contacts | s per metre | Position error |
|---|---|---|---|---|---|---|---|
| A* | SAC | 90% | 0.82 | 1.04, 1.39 | 1.9 | 4.3 | 6 cm |
| A* | pure pursuit | 90% | 0.84 | 0.97, 1.27 | 2.6 | 5.6 | 7 cm |
| greedy best-first | pure pursuit | 83% | 0.80 | 0.96, 1.17 | 3.3 | 4.9 | 8 cm |
| RL route | SAC | 75% | 0.61 | 1.19, 1.66 | 2.7 | 4.9 | 6 cm |

## 3. Large worlds, 4-8 m mazes, warehouses and rooms, onboard sensors only (100 missions)

This is the hardest setting. The worlds are bigger than one camera can cover, and
the pose drifts with no absolute fix.

| Route planner | Motion controller | Success | SPL | Route / shortest | Wall contacts | s per metre | Position error |
|---|---|---|---|---|---|---|---|
| A* | SAC | 75% | 0.62 | 1.10, 1.80 | 6.0 | 5.1 | 10 cm |
| greedy best-first | SAC | 74% | 0.61 | 1.13, 1.91 | 7.2 | 5.2 | 10 cm |
| RL route | SAC | 75% | 0.57 | 1.25, 2.03 | 9.0 | 5.8 | 11 cm |
| A* | pure pursuit | 68% | 0.60 | 1.01, 1.62 | 8.6 | 5.7 | 14 cm |
| RL route | pure pursuit | 64% | 0.53 | 1.11, 1.88 | 9.1 | 6.6 | 14 cm |
| greedy best-first | pure pursuit | 63% | 0.54 | 1.02, 1.87 | 12.0 | 6.0 | 15 cm |

## 4. Latency

| Stage | Time |
|---|---|
| SAC PWM decision (L2) | 3-4 ms p95, budget 50 ms (20 Hz) |
| Pure pursuit decision | under 1 ms |
| A* replan | 10-28 ms p95 |
| Greedy best-first replan | 7-14 ms p95 |
| RL route plan (L1), 24 cells | about 6 ms per cell, so about 0.14 s per plan with 8 CPU threads (same on the RTX 4050) |

The evaluation tables above report the RL plan time as 3.2-3.3 s p95. That figure
was measured with 12 missions running in parallel, each limited to one CPU thread.
It is not the rover's latency. The 0.14 s figure was measured in a single process.

## 5. What the numbers say

- **The RL PWM controller is the best motion controller on every setting**:
  - large worlds: 75% vs 68% with A*
  - camera arena: 93% vs 90%
  - pure pursuit is a tuned classical baseline that also drives PWM directly.

  The SAC controller also reaches the goal faster (4.2-5.1 s per metre vs 5.2-5.7) and touches walls less often in the large worlds.
- **The overhead camera works as the team intended**: position error drops from 6 cm to 1 cm, and success with SAC rises from 90% to 93%.
- **The RL route planner is not better than A***. It succeeds and gets close (83-93% of A* success in the arenas, equal in the large worlds). Its routes are 1.15-1.25 x shortest, against A*'s 1.01-1.10. It is kept because the brief asked for RL in the loop. The default is A* for routes plus SAC for PWM; this is the "near-shortest, lowest latency" combination the team asked for.
- **Greedy best-first** was the team's previous approach. It plans fastest, but with SAC it is 1 point behind A* in success and leaves longer tails (p90 1.91 vs 1.80).
- **Failures in the large worlds**: almost all are timeouts. In these cases the pose has drifted far enough that the map no longer lines up with the walls. Section 3 of `GAP_ANALYSIS.md` covers this (loop closure needs a richer sensor or the camera).

## 6. Earlier ablations (for the gap analysis)

| Change | Before | After |
|---|---|---|
| collision monitor, SAC (32 missions) | 38% | 50% |
| costmap centre sampling, hard margin 0 (A* + PP, arena / large) | 19 / 18 of 24 | 22 / 19 of 24 |
| overhead camera, position error | 5.5 cm | 0.8 cm |
| SAC v2 (route following) -> v4 (mixed with whole missions), A* | 65% | 77% |
| route DQN reward fix, at 25k steps | 23% | 35% |
| route DQN final, 300 held-out worlds | | 93.7%, 1.22 x shortest |
