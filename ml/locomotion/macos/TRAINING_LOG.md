# CW-1 Locomotion Training Log

## Goal
Train a quadruped robot to walk using PPO + MuJoCo on macOS Apple Silicon.

## Iteration History

### Run 0 — Baseline (broken)
- **Issue**: URDF→MJCF joint name mismatch, NaN instability
- **Fix**: Handcrafted MJCF, NaN guard, arm damping 2→20
- **Result**: Pipeline runs but robot just stands still (reward function broken)

### Run 1 — Reward rebalancing
- **Changes**: alive 0.5→0.1, orientation -5→-1, track_lin_vel 1.5→4.0, action_scale 0.25→0.5, 8 parallel envs
- **Result**: Avg reward 53.38 (+50% vs random), distance 0.37m/20s, 90% survival
- **Problem**: Robot vibrates/shuffles but doesn't walk. 0.02 m/s vs target 0.3-1.5 m/s

### Run 2 — Legged Gym aligned
- **Changes**: Rewrote feet_air_time to first-contact style, removed orientation penalty (0.0), removed alive bonus (0.0), aligned reward weights to legged_gym
- **Result**: FAILED — robot falls over in 2 seconds (WORSE than Run 1)
- **Root cause**: Our robot has a tall asymmetric mast/arm, not a standard symmetric quadruped. Needs orientation feedback. Also, orientation=0.0 + alive=0.0 = zero incentive to stay upright.

### Run 2.5 — Physics debugging
- **Discovery 1**: Actuator kp=25 too weak for 15.6kg robot (Go1 uses kp=80-100). Robot sinks from 0.35m to 0.18m even with zero actions. Fixed: kp=80, kd=2.0.
- **Discovery 2**: DEFAULT_JOINT_POS had wrong hip_pitch signs! Front hip_pitch=+0.15 made legs go BACKWARD (feet clustered under body center). Support polygon was only ~3cm wide in X. Fixed: front hp=-0.6 (legs forward), rear hp=+0.6 (legs backward). Support polygon: 33cm × 27cm.
- **Discovery 3**: Initial stance kinematically incorrect — old angles gave z_reach=0.256m for target height 0.35m (feet didn't touch ground!). Recalculated: hp=-0.6, kp=1.24 gives exact 0.325m reach.
- **Validation**: After fixes, robot stands 10s+ with <1° tilt (no randomization), 17° with randomization. 100% survival across 20 random episodes.

### Run 3 — Stability first (in progress)
- **Changes**:
  - Actuator kp: 25→80, kd: 0.5→2.0 (joint damping also 0.5→2.0)
  - DEFAULT_JOINT_POS: front legs forward (hp=-0.6, kp=1.24), rear backward (hp=0.6, kp=-1.24)
  - Rewards: alive=0.2, orientation=-0.5 (was 0.0), base_height=1.0 (NEW), collision=-1.0 (NEW)
  - Termination: max_body_tilt 1.2→0.7 rad (69°→40°)
  - Commands: cmd_vx (0.3,1.5)→(0.0,0.8), cmd_vy (±0.3)→(±0.2), cmd_yaw (±1.0)→(±0.5)
  - tracking_sigma: 0.25→0.5 (wider gradient for easier learning)
  - Fixed: _first_contact initialization in __init__/reset()
- **Result** (500k): Survived 56%, tilt 24°, speed -0.038 m/s (backward). Robot balances briefly then falls.
- **Result** (2M): Survived 100%, tilt 1.5°, speed 0.017 m/s. PERFECT BALANCE but standing still.
- **Analysis**: Standing still is a local optimum — alive + base_height + partial vel tracking (73% at sigma=0.5) makes standing rewarding.

### Run 4 — Velocity-focused
- **Changes**: tracking_sigma 0.5→0.25, track_lin_vel 1.0→2.0, alive 0.2→0.1, base_height 1.0→0.5, orientation -0.5→-0.3, cmd_vx (0.0,0.8)→(0.2,0.8)
- **Result** (2M): Survived 100%, tilt 3.6°, speed **0.109 m/s**, dist 0.55m/5s
- **Gait analysis**: Feet lifting 5-6cm, hints of trot gait (diagonal pairs). Real stepping locomotion!
- **Key**: Sharper tracking sigma + higher weight broke the standing-still local optimum

### Run 5 — Longer training + gait refinement *** WALKING ACHIEVED ***
- **Changes**: feet_air_time threshold 0.5→0.2s (matches actual step cycle), 5M steps
- **Result**:
  - Speed: **0.382 m/s** (96% of target 0.4 m/s!)
  - Distance: 3.82m in 10s
  - Survival: 100% (10s episodes)
  - Height: 0.340m (97% of target 0.35m)
  - Tilt: 6.5° average
  - Gait: Trot-like (diagonal pair coordination FL+RR 18%, FR+RL 17%)
  - Foot lift: max 10.7cm, avg 4.9cm (front), 3.1cm (rear)
  - Training: 5M steps, 20 min on M4 Max, 8 parallel envs

---

## Summary of Progress

| Run | Speed (m/s) | Survival | Key Issue |
|-----|-------------|----------|-----------|
| 0   | 0.0         | ~50%     | Pipeline broken |
| 1   | 0.02        | 90%      | Vibrating, no walking |
| 2   | N/A         | <10%     | Falls instantly (no orientation) |
| 3   | -0.04       | 56%      | Physics wrong (kp, stance) |
| 3b  | 0.017       | 100%     | Standing still (local optimum) |
| 4   | 0.109       | 100%     | Crawling forward |
| **5** | **0.382** | **100%** | **Walking! Trot gait** |
| **6** | **0.774** | **100%** | **Fast walking! Speed modulation** |
| 7a-7e | 0.02-0.11 | 50-100% | v2 model: standing/shuffling, no walking |
| **7f** | **0.34** | **100%** | **v2 Walking! Gait clock breakthrough** |

### Run 6 — Speed range expansion + 10M steps
- **Changes**: cmd_vx_range (0.2,0.8)→(0.0,1.5), 10M steps (40 min)
- **Result**:
  - Max speed: **0.774 m/s** at cmd=1.2 (+61% vs Run 5)
  - Speed modulation: 0.064 m/s at cmd=0 → 0.774 m/s at cmd=1.2
  - Tracking accuracy: 94% at 0.4, 86% at 0.6, 83% at 0.8
  - 100% survival across all speed commands (0.0–1.5 m/s)
  - Tilt under 7° at all speeds
  - Near-zero drift at cmd=0 (learned to stand still)

---

## Key Findings

### Physics That Matter
1. **Actuator gains**: kp=80, kd=2.0 minimum for 15.6kg robot (kp=25 failed)
2. **Leg stance splay**: Front legs forward, rear backward — NOT all tucked under body
3. **Kinematic correctness**: DEFAULT_JOINT_POS must place feet on ground at initial height
4. **Forward COM bias**: Our arm/head makes robot front-heavy, needs orientation penalty

### Reward Design Lessons
1. **tracking_sigma**: 0.5 too generous (standing gets 73% reward), 0.25 works well
2. **alive bonus**: Needed for stability but too high → standing-still optimum
3. **base_height reward**: Critical for maintaining posture
4. **collision penalty**: Prevents body-ground contact
5. **feet_air_time threshold**: 0.2s matches natural step cycle (0.5s too strict)
6. **orientation penalty**: Required for our asymmetric robot (unlike standard quadrupeds)

### Training Insights (SB3 PPO on macOS CPU)
- 8 parallel envs, ~4200 steps/s on M4 Max
- 500k steps: basic behavior (~2 min)
- 2M steps: good balance (~8 min)
- 5M steps: walking gait (~20 min)
- 10M steps: fast walking + speed modulation (~40 min)
- n_steps=4096 better than 2048 for small env counts

---

## Research Notes

### Reference: ETH legged_gym (leggedrobotics/legged_gym)
The gold standard for quadruped locomotion RL. Key findings:

**Reward weights (default config)**:
| Reward | Weight | Our Run 1 |
|--------|--------|-----------|
| tracking_lin_vel | 1.0 | 4.0 (too high?) |
| tracking_ang_vel | 0.5 | 1.0 |
| feet_air_time | 1.0 | 1.0 |
| lin_vel_z | -2.0 | -1.0 |
| ang_vel_xy | -0.05 | -0.05 |
| orientation | **0.0** | -1.0 (!!!) |
| torques | -0.00001 | -0.00001 |
| dof_acc | -2.5e-7 | -1e-7 |
| action_rate | -0.01 | -0.005 |
| collision | -1.0 | 0 |
| alive | (not used) | 0.1 |
| stand_still | 0.0 | (not used) |

**Critical finding: orientation penalty is 0.0 in legged_gym!**

**feet_air_time implementation (legged_gym)**:
```python
first_contact = (feet_air_time > 0.) * contact_filt
rew = sum((feet_air_time - 0.5) * first_contact)
rew *= norm(commands[:2]) > 0.1  # only when moving
feet_air_time *= ~contact_filt  # reset on contact
```
- Rewards feet LANDING after being airborne (not continuous air time)
- Threshold: 0.5s airborne before reward kicks in
- Only active when robot is commanded to move
- This is fundamentally different from our implementation!

**Other key params**: action_scale=0.5, decimation=4, tracking_sigma=0.25

**IMPORTANT: legged_gym uses PASSIVELY STABLE robots (Anymal, A1)**. These robots have:
- Symmetric body (low, centered COM)
- Wide stance by default
- kp=80+, so PD controller keeps them upright without RL help
- That's why orientation=0.0 works for them!

Our CW-1 is DIFFERENT:
- Asymmetric body (tall arm mast at front, head at front)
- Forward-biased COM
- Needs active balance from RL policy
- Needs orientation penalty and alive bonus

### Why the robot vibrates instead of walks
1. Our feet_air_time rewards continuous air time → robot lifts feet slightly and holds
2. Orientation penalty (-1.0) punishes any tilt → walking requires weight shifting
3. No first-contact reward → no incentive to complete a step cycle
4. Reward for velocity tracking alone can be gamed by oscillating

### Fix strategy
1. Rewrite feet_air_time to legged_gym style (reward on landing)
2. Set orientation penalty to 0.0
3. Align reward weights closer to legged_gym defaults
4. Keep 8 parallel envs, short runs (500k) for iteration
5. Visual analysis via screenshots of rendered demos

---

## v2 Architecture — Mammalian Knees + Terrain (Run 7+)

### Changes from v1
1. **Arm/bag removed**: Mass 15.6→13.9 kg, body nearly symmetric (only 0.5kg head)
2. **Mammalian knees**: All 4 knees bend backward (same range [-0.0995, 2.6005])
3. **Actuator kp bug fixed**: Removed per-actuator kp="25" overrides → defaults to kp=80
4. **Default pose**: All legs [0, -0.6, 1.2] (hip_pitch=-knee/2 for zero foot displacement)
5. **Orientation penalty**: -0.3 → -0.1 (near-symmetric body needs less active balancing)
6. **Heightfield terrain**: 6 types (flat, rough, obstacles, stairs_up, slope_30, slope_debris)
7. **Foot clearance reward**: 0.3 weight, rewards airborne foot height during swing phase
8. **Proportional collision**: Counts all non-foot ground contacts (capped at 4)
9. **Terrain-relative heights**: base_height reward and termination use body_z - terrain_z

### Standing Test Results (v2 model)
- Joints: 13 (1 free + 12 leg), Actuators: 12, all kp=80
- Height: 0.347m (min over 500 steps), tilt: 0.61° max
- All feet at zero forward displacement (symmetric support polygon)

### Run 7a-7e — v2 model, flat training iterations
- **7a** (5M steps, fresh): PPO collapse at 400K. Policy peaks then std explodes (1.0→2.2)
- **7b** (500K, conservative): 81% survival, 0.079 m/s — shuffling, not walking
- **7c** (1M, conservative): Same pattern — standing 400K, crash by 700K
- **7d** (500K, tighter sigma): Standing optimum persists despite sigma 0.25→0.15
- **7e** (500K, kp=40): 100% survival, 0.044 m/s — squatting and shuffling (kp too soft)
- **Root cause**: PPO with 8 envs can't discover coordinated 12-joint stepping from random exploration

### Run 7f — Gait clock breakthrough *** v2 WALKING ***
- **Key change**: Added **gait clock** to observation (4 dims: sin/cos of trot phase for each diagonal pair) and gait matching reward (1.0 weight). This tells the policy WHEN each foot should be in stance vs swing.
- **Other params**: kp=80, action_scale=0.5, tracking_sigma=0.15, cmd_vx (0.3,0.8), alive=0.1, base_height=0.5, lr=3e-4, ent=0.01
- **Result** (500K steps, 2 min):
  - Speed: **0.34 m/s** (85% tracking at cmd=0.4)
  - Survival: **100%** (10s episodes)
  - Height: 0.346m (99% of target)
  - Tilt: 5-7°
  - Monotonic improvement across all checkpoints (no collapse!)
- **Speed range** (10s episodes, 5 trials):
  - cmd=0.2: 0.317 m/s, 100% survival
  - cmd=0.4: 0.325 m/s, 97% survival
  - cmd=0.6: 0.280 m/s, 89% survival
  - cmd=0.8: 0.302 m/s, 84% survival
- **Extended training** (2M+ steps): Speed improves to 0.4-0.7 m/s but stability degrades (50→7% survival). PPO std slowly grows even with lr=5e-5 and clip=0.1.
- **Best checkpoint**: 500K steps from first training (100% survival)

### Key Insight: Gait Clock
Without the gait clock, PPO needs to discover coordinated 12-joint stepping motions from pure random exploration — essentially impossible with 8 parallel environments. The gait clock provides a trot phase signal (sin/cos for FL+RR and FR+RL diagonal pairs) that:
1. Tells the policy exactly when each foot should lift and land
2. Provides a dense reward for matching the desired contact pattern
3. Reduces the exploration problem from "discover walking" to "learn to execute a trot"

Observation space: 48 → 52 dims (4 gait clock signals added)
Gait period: 0.5s (2 Hz stride frequency)
Phase offsets: FL=0, FR=π, RL=π, RR=0 (trot pattern)

### Remaining Issues
1. **No speed modulation**: Robot walks at ~0.30 m/s regardless of velocity command
2. **Extended training instability**: PPO policy degrades after 500K-800K steps
3. **Terrain not yet tested**: Heightfield system built but training only on flat

### Training Curriculum
| Stage | Steps | Terrain | Status |
|-------|-------|---------|--------|
| 1 (Run 7f) | 500K | flat | DONE — 0.34 m/s, 100% survival |
| 2 | TBD | flat, rough | Pending — need speed modulation first |
| 3 | TBD | full mix | Pending |
