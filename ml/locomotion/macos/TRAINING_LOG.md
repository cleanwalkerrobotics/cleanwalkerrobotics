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
