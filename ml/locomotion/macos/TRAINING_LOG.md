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
| **8** | **0.21** | **100%** | **v2 Walking with foot lift! Two-phase training** |
| 9a-9c | 0.33-0.43 | 5-94% | Velocity cmd bug fix, reward balancing |
| **10** | **0.375** | **100%** | **Zero entropy, stable training** |
| HPO v2 | 0.506 | 98% | 190 trials — vibrating gait (wrong objective) |
| HPO v3 | 0.685 | 100% | 62 trials — gait quality objective, stability-aware |
| **15** | **0.364** | **100%** | **Genuine walking! 6cm foot lift, smooth** |

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
| 1b (Run 8) | 700K+50K | flat | DONE — 0.21 m/s, 100% survival, natural foot lift |
| 2 | TBD | flat, rough | Pending — need speed modulation first |
| 3 | TBD | full mix | Pending |

---

### Run 8 — Two-phase training with WTW-inspired shaped rewards

#### Research
Studied Unitree (legged_gym, rsl_rl), walk-these-ways (WTW), IsaacLab Spot. Key findings:
- **Phase-based foot clearance target** (WTW weight -30): penalizes deviation from target swing height
- **Shaped contact force/velocity rewards**: reward stance feet for ground reaction force, swing feet for velocity
- **Action smoothness 1st/2nd order**: prevents stiff/jerky leg movement
- **Foot slip penalty**: penalizes foot XY velocity during stance
- **Adaptive KL-based LR**: industry standard from rsl_rl to prevent PPO collapse

#### Strategy: Two-phase training
Adding all shaped rewards from scratch suppresses exploration (penalties kill any movement before walking can be discovered). Solution: learn to walk first with simple rewards, then refine gait quality.

**Phase 1** (proven Run 7f config): Binary gait clock, minimal penalties, 8 envs, lr=3e-4
**Phase 2** (`--refine`): Load Phase 1 walker, add shaped rewards (clearance target, smoothness, slip, force/velocity)

#### Phase 1 — Get walking (8 envs, lr=3e-4, ent=0.01)
Same reward structure as Run 7f. Training progression:

| Steps | Speed (m/s) | Survival |
|-------|-------------|----------|
| 50K | 0.002 | 100% |
| 100K | 0.003 | 100% |
| 200K | 0.014 | 100% |
| 300K | 0.051 | 100% |
| 400K | 0.078 | 100% |
| 500K | 0.122 | 100% |
| **700K** | **0.190** | **100%** |
| 800K | 0.221 | 80% |
| 900K | 0.207 | 60% |
| 1M+ | collapse | 0% |

Best Phase 1: **700K steps at 0.190 m/s, 100% survival**

#### Phase 2 — Refine gait quality (shaped rewards)
Added from `cw1_policy_phase1_best.zip`:
- `rew_gait_force=1.0`, `rew_gait_vel=1.0` (shaped contact rewards)
- `rew_clearance_target=-30.0` (L1 swing height target, 6cm)
- `rew_action_smooth_1=-0.1`, `rew_action_smooth_2=-0.1` (smoothness)
- `rew_foot_slip=-0.25` (anti-slide)
- `rew_air_time_var=-1.0` (even gait)
- `rew_hip_vel=-1e-2` (anti-wobble)
- `rew_torques=-1e-4` (10x stronger energy penalty)

Best: **Phase 2 50K steps at 0.209 m/s, 100% survival**

#### Gait analysis (best checkpoint)
- Foot lift: FL=6.4cm, FR=9.1cm, RL=8.2cm, RR=5.3cm (**real stepping!**)
- Step cycles: ~40 per foot over 10s (~4 Hz)
- Body height: 0.353m stable (0.327-0.378 range)
- Survival: 100%

#### Key findings
1. **Two-phase training works**: Phase 1 discovers walking, Phase 2 refines gait quality
2. **Shaped rewards suppress exploration**: Adding too many penalties prevents PPO from discovering walking
3. **PPO collapse persists**: Even with constant lr=3e-4, std grows from 1.01→1.8 and policy collapses after 800K-1M steps
4. **Adaptive LR too aggressive**: SB3's LR is quickly reduced to 7.5e-5 which slows learning too much
5. **Natural foot lift achieved**: Phase 2 clearance target + smoothness produces 5-9cm foot lift per step

---

### Run 9 — Velocity Command Bug Fix + Reward Rebalancing (5M steps)

**Critical bug found**: In `cw1_env.py` `reset()`, `_velocity_cmd` was randomized (lines 371-375) then **overwritten to zeros** (line 396: `self._velocity_cmd = np.zeros(3, dtype=np.float32)`). This made cmd_speed=0 always, disabling ALL gait rewards (they check `cmd_speed > 0.1`). Runs 7f and 8 trained with effectively zero velocity commands.

**Fix**: Removed the zeroing line. Gait rewards now active.

#### Run 9a — First attempt (no adaptive LR)
- Config: 5M steps, n_envs=8, n_steps=4096, lr=3e-4, flat terrain
- Orientation penalty reduced to -0.1 (too weak)
- **Result**: KL exploded to 0.109, std hit 1.16+ by 700K. Robot sprinted at 0.44 m/s but only 8% survival.
- **Stopped at 800K** — PPO instability without adaptive LR

#### Run 9b — Adaptive LR (KL target 0.02)
- Config: 5M steps, n_envs=8, n_steps=2048, adaptive LR, flat terrain
- Orientation restored to -0.3, cmd_vx range (0.0, 0.8)
- **Result**: Adaptive LR controlled KL well (→7.5e-5). But 0.6 m/s with 6-16% survival — still sprint-and-crash.
- **Root cause**: Speed incentives (tracking 3.0 + gait 1.0 = 4.0) overwhelmed stability (alive 0.1 + orientation 0.3 = 0.4).
- **Stopped at 1.5M**

#### Run 9c — Stability-focused rewards
- Changes from 9b: alive 0.1→0.5, orientation -0.3→-1.0, gait_clock 1.0→0.5, fall penalty -2.0 (new), cmd_vx (0.2, 0.5)
- **Result**: Walking at 800K-1.45M range!

| Checkpoint | Speed | Survival | Tilt |
|-----------|-------|----------|------|
| 800K | 0.333 | 91% | 4.0° |
| 1M | 0.414 | 85% | 4.3° |
| **1.3M** | **0.431** | **94%** | **4.5°** |
| 1.45M | 0.448 | 90% | 4.3° |
| 1.5M | 0.677 | 5% | 14.6° |

- PPO collapse at 1.5M: std grew from 1.0→1.16+ and policy collapsed to sprint-and-crash
- Velocity modulation: 0.35-0.49 for commands 0.2-0.6 (some tracking but not precise)

---

### Run 10 — Zero Entropy Training (5M steps) ★ BEST

**Key insight**: ent_coef=0.01 drives std growth, which causes PPO collapse at ~1.5M steps. Setting ent_coef=0.0 lets std naturally decrease as the policy becomes confident.

- Config: 5M steps, n_envs=8, ent_coef=0.0, adaptive LR (KL target=0.02), flat terrain
- Rewards: alive=0.5, orientation=-1.0, gait_clock=0.5, track=3.0, fall penalty=-5.0
- cmd_vx range (0.0, 0.7)

**Training metrics (healthy throughout)**:
- std: 1.0 → 0.93 (500K) → 0.85 (1.2M) — continuously DECREASING
- KL: 0.022-0.028 (well-controlled near 0.02 target)
- clip_fraction: 0.25-0.30 (healthy)
- No collapse observed at any point

| Checkpoint | Speed | Survival | Tilt | Reward |
|-----------|-------|----------|------|--------|
| 500K | 0.243 | **100%** | 3.1° | 37.1 |
| **1M** | **0.375** | **100%** | **2.4°** | **33.9** |
| 1.5M | 0.301 | 100% | 2.3° | 30.7 |
| 2M | 0.288 | 100% | 2.6° | 30.5 |

Best: **1M steps — 0.366 m/s avg (10 eps), 100% survival, 2.3° tilt**

Policy converges to safe-but-slightly-slow gait after 1M (std becomes too low, reducing exploration). The 1M checkpoint is the sweet spot before over-convergence.

#### Velocity response (1M checkpoint)
- cmd 0.2 → 0.35 m/s (slight modulation)
- cmd 0.4 → 0.37 m/s
- cmd 0.6 → 0.38 m/s

Minimal tracking — policy walks at ~0.37 regardless. Tracking improvement deferred to terrain training.

#### Run 11 — Sharper tracking (attempted)
- sigma 0.15→0.05, alive 0.5→0.3
- Result: Slower learning, peaked at 0.339 m/s (1.5M). Sharper sigma made learning harder without improving tracking.
- **Discarded** — Run 10 remains best.

#### Key findings (Runs 9-10)
1. **Critical: velocity command bug** invalidated Runs 7f-8 gait rewards
2. **ent_coef=0.0 prevents PPO collapse**: std decreases naturally, no instability
3. **Reward balance matters**: alive + orientation must be competitive with tracking + gait
4. **Fall penalty**: -5.0 is effective but can make policy overly conservative
5. **Adaptive LR with KL target 0.02**: essential for stable training with SB3 PPO
6. **Training sweet spot**: ~1M steps with ent_coef=0. Policy over-converges beyond this.

---

### HPO Study — Optuna Reward Weight Optimization

**Protocol**: 800K steps per trial, eval@200K/400K/600K/800K, triage at 500K (surv<90% or tilt>6° → prune)
**Sampler**: TPE (10 startup, multivariate), MedianPruner (5 startup, 1 warmup)
**Parameters**: 6 reward weights (track_lin_vel, alive, orientation, tracking_sigma, fall_penalty, gait_clock)

Results: 25 trials (9 completed, 15 pruned, 1 failed)

Top 5 trials:
| Trial | Score | track | alive | orient | sigma | fall | gait |
|-------|-------|-------|-------|--------|-------|------|------|
| #10 | 0.341 | 5.43 | 0.26 | -1.30 | 0.175 | 5.86 | 0.21 |
| #5 | 0.336 | 5.66 | 0.28 | -1.27 | 0.248 | 4.77 | 0.13 |
| #24 | 0.325 | 5.03 | 0.29 | -0.61 | 0.179 | 1.84 | 0.36 |
| #13 | 0.309 | 2.79 | 0.45 | -0.81 | 0.052 | 1.50 | 0.94 |
| #4 | 0.282 | 3.98 | 0.78 | -0.66 | 0.067 | 8.60 | 0.82 |

**Clear convergence pattern**: High track_lin_vel (~5.5) + low alive (~0.25) optimizes for 800K-step score.

**Key finding**: HPO params optimized for 800K eval window. When trained to 1M+ steps, they underperform Run 10.

---

### Run 12 — Pure HPO Trial #10 Params (2M steps)

- Config: track=5.43, alive=0.26, orient=-1.30, sigma=0.175, fall=5.86, gait=0.21
- Training: 2M steps, 15m 44s

| Checkpoint | Speed | Survival |
|-----------|-------|----------|
| 500K | 0.195 | 100% |
| 750K | 0.260 | 100% |
| 950K | 0.338 | 99% |
| 1M | 0.316 | 94% |
| 1.5M | 0.136 | 100% |

**Peak: 950K at 0.338 m/s — BELOW Run 10's 0.375**
HPO's 800K eval window found params that learn faster early but peak lower at full training.

---

### Run 13 — Tempered HPO Params (1.5M steps)

- Config: track=4.0, alive=0.35, orient=-1.15, sigma=0.15, fall=5.0, gait=0.35
- Intermediate between Run 10 defaults and HPO Trial #10

| Checkpoint | Speed | Survival |
|-----------|-------|----------|
| 500K | 0.225 | 100% |
| 750K | 0.282 | 100% |
| 1M | 0.338 | 100% |
| 1.25M | 0.338 | 100% |
| 1.5M | 0.278 | 100% |

**Peak: 1M at 0.338 m/s — still below Run 10.** Reverted to Run 10 defaults.

---

### Full Assessment (after Run 13)

**SOTA Research** (legged_gym, WTW, Rapid Locomotion, MuJoCo Playground):
1. Primary bottleneck: 8 envs (vs 4096 industry standard)
2. Missing foot contact signals in observation (all production systems include these)
3. ent_coef=0.0 prevents collapse but causes over-convergence (std too low)
4. action_scale=0.5 too conservative (35% of joint range, should be 75%)
5. 8 of 21 reward terms disabled
6. Recommended: ent_coef=0.005 with kl_target=0.01

**MJCF Analysis**:
1. Generator has wrong kp/kv (25/0.5 vs active 80/2.0) — critical bug in convert_urdf_to_mjcf.py
2. action_scale=0.5 limits to ±29° (too conservative)
3. Missing foot contacts and body height in observation

---

### Run 14 — SOTA-Aligned Architecture (5M steps, IN PROGRESS)

**Changes from Run 10 baseline:**

| Parameter | Run 10 | Run 14 | Rationale |
|-----------|--------|--------|-----------|
| n_envs | 8 | 32 | 4x more samples/rollout |
| n_steps | 2048 | 1024 | Keep buffer size reasonable (32K) |
| ent_coef | 0.0 | 0.005 | Prevent over-convergence, maintain exploration |
| kl_target | 0.02 | 0.01 | Tighter policy updates (SOTA standard) |
| action_scale | 0.5 | 0.75 | 50% more joint authority |
| fall_penalty | 5.0 | 2.0 | Less conservative (SOTA uses 0-2) |
| rew_clearance_target | 0.0 | -5.0 | Enable foot clearance reward |
| rew_action_smooth_1 | 0.0 | -0.05 | Mild action smoothness |
| only_positive_rewards | N/A | true | legged_gym standard: clip reward >= 0 |
| **obs space** | **52 dims** | **56 dims** | **+4 foot contact binary signals** |

Observation space: body_state(9) + cmds(3) + joints(24) + prev_actions(12) + gait_clock(4) + **foot_contacts(4)** = 56

Training: `python train.py --timesteps 5000000 --n-envs 32 --n-steps 1024 --ent-coef 0.005`
Speed: ~5,500 steps/s (2.5x faster than old 8-env setup), ETA ~15 min

---

### HPO v2 — Full Reward Weight Optimization (190 trials)

Extended HPO with 15 parameters including gait-shaped rewards. 800K steps per trial, objective: `speed * min(1, survival/95)^2`.

**Best Trial #21**: score 0.506, speed 0.506 m/s, 98% survival.
- track=4.49, alive=0.14, orient=-1.71, sigma=0.145, fall=9.08, gait=0.67
- smooth1=-0.085, clearance=-1.69, ent=0.0017, action_scale=0.574

**Problem**: Speed-only objective produced vibrating/shuffling gaits. The "0.506 m/s" was achieved by rapid small oscillations, not actual walking with foot clearance.

---

### HPO v3 — Gait Quality Optimization (62 trials)

**Root cause of vibrating gait**: HPO v2 optimized `speed * survival` which rewards any forward velocity regardless of gait quality. L1 smoothness weight -0.085 was 30x too weak (equivalent to L2 -0.007). Air time threshold 0.5s was physically impossible (> half gait period).

**New composite objective**: `speed^0.3 * survival^0.2 * step_quality^0.3 * smoothness^0.1 * energy^0.1`
- Evaluates action_smoothness, avg_foot_clearance, contact_regularity, cost_of_transport
- 20 search parameters with corrected ranges
- 12 parallel envs (up from 8)

**Critical discovery: 8 of 13 completed trials COLLAPSE during training.** The "best" Trial #30 (score 0.75) peaked at 500K then crashed to 0.018 by 1.5M. Caused by using `max(intermediate_scores)` as trial score.

**Fix**: Stability-aware scoring: `best_score * sqrt(final_score / best_score)`. Penalizes collapse, rewards sustained quality.

**Re-ranked top 3 by stability**:
| Trial | Max Score | Final Score | Stable Score |
|-------|-----------|-------------|--------------|
| **#35** | **0.685** | **0.658** | **0.672** |
| #50 | 0.578 | 0.469 | 0.521 |
| #21 | 0.500 | 0.400 | 0.447 |

**Trial #35 key params** (the "anti-vibration" config):
- `action_scale=0.33` — small actions prevent jerky movement
- `smooth1=-0.454` — 5x stronger than v2 (prevents vibration)
- `smooth2=-0.083` — 2nd order smoothness
- `gait_force=2.84, gait_vel=1.00` — strong gait shaping
- `clearance=-24.8` — strong foot lift incentive
- `tracking_sigma=0.271` — forgiving velocity tracking
- `ent_coef=0.00258, kl_target=0.021`

---

### Run 15 — Production Training with HPO v3 Trial #35 (5M steps)

**Config**: All defaults updated to Trial #35's stable params. 12 envs, n_steps=2048, batch_size=64.

**Training**: 5M steps, **20 minutes**, 4,108 steps/sec on M4 Max.

**Final metrics**:
- std: 0.617 (converged)
- explained_variance: 0.807
- No collapse observed

**Evaluation** (10 episodes, 500 steps, cmd_vx=0.4):

| Metric | Value |
|--------|-------|
| Speed | **0.364 m/s** |
| Survival | **100%** |
| Body tilt | **1.7°** |
| Body height | 0.375m |
| Consistency | 0.361–0.367 m/s |
| Foot clearance | **6.1cm** |
| Action smoothness | 0.312 |
| Step count | 25.1 steps/10s |
| Composite score | **0.747** |

**Comparison with previous runs**:
| Metric | Run 6 (v1 best) | Run 10 (v2 best) | **Run 15** |
|--------|------------------|-------------------|------------|
| Speed | 0.77 (vibrating) | 0.375 | **0.364** |
| Survival | 100% | 100% | **100%** |
| Tilt | ~7° | 2.4° | **1.7°** |
| Foot clearance | N/A | N/A | **6.1cm** |
| Smoothness | Poor | OK | **Good** |
| Consistency | Variable | Good | **Excellent** |

This is the first policy that demonstrably walks (lifts feet 6cm, 2.5 Hz stepping, smooth actions) rather than vibrating or shuffling. Speed is "slower" than Run 6 but that was fake speed from oscillation.

**Remaining**: Max speed limited to ~0.37 m/s by action_scale=0.33. Next step: terrain training and gradual speed increase.

---

### Run 16 — Fix Hip Yaw + Lateral Drift (fine-tune from Run 15)

**Visual problems observed**: (1) robot drifts sideways, (2) hip yaw joints oscillate wastefully on every step.

#### Run 16a — From scratch with penalties (FAILED)
- Added `rew_hip_vel=-0.005`, new `rew_lat_vel=-2.0` (lateral body velocity penalty)
- Trained from scratch, 5M steps
- **Result**: Speed 0.263 (-28%), tilt 3.9°, drift DOUBLED (0.089 ratio). Local optimum.

#### Run 16b — Fine-tune with penalties (PARTIALLY FAILED)
- Fine-tuned from Run 15 (restored via `git show ff9858b`)
- 2M steps with hip_vel=-0.005, lat_vel=-2.0
- **Result**: Hip yaw -45%, but lateral drift got MUCH WORSE (-1.112m consistent LEFT)

#### Run 16c — Per-joint action scaling (BETTER)
- **Key innovation**: limit hip yaw AUTHORITY instead of penalizing velocity
- Per-joint `action_scales` vector: hip_yaw gets 50% of main scale (0.165 vs 0.33)
- Softer penalties: hip_vel=-0.001, lat_vel=-0.5
- Fine-tuned 2M from Run 15
- **Result**: Hip yaw -81%, speed 0.334 (-8%), tilt 1.9°
- BUT: -0.876m left drift (consistent across all 10 seeds)
- **Discovery**: Run 15 also had consistent 0.398m RIGHT drift (policy weight asymmetry)

#### Run 16d — Stronger heading correction (SUCCESS) ★
- Boosted `rew_track_ang_vel`: 0.5 → 2.0
- Boosted `rew_lat_vel`: -0.5 → -5.0
- Fine-tuned 2M from Run 16c
- **Result**:

| Metric | Run 15 | Run 16d | Change |
|--------|--------|---------|--------|
| Speed | 0.364 | **0.323 m/s** | -11% |
| Survival | 100% | **100%** | — |
| Tilt | 1.7° | **1.7°** | — |
| Hip yaw | 2.58 rad/s | **0.90 rad/s** | **-65%** |
| Lat drift | ±0.40m | **+0.013m** | **-97%** |

**Committed: 6026d1e**

---

### Terrain Training — Bug Fixes + Curriculum

#### Bug 1: Collision detection on rough terrain
Original code penalized ALL non-foot floor contacts. On rough terrain, heightfield bumps caused `body_geom↔floor` (149 contacts/step) and `calf_geom↔floor` contacts, triggering -4.0 collision penalty per step. Robot stood still (0.002 m/s).

**Fix**: Only count `body_geom` and `head_geom` floor contacts as collisions (actual fall indicators). Leg/calf contacts with terrain are normal.

```python
# Before: penalize all non-foot contacts
is_foot = np.isin(other, self.foot_geom_ids)
collision_count = np.sum(has_floor & ~is_foot)

# After: only torso/head contacts = real collisions
is_collision_body = np.isin(other, self.collision_geom_ids)
collision_count = np.sum(has_floor & is_collision_body)
```

#### Bug 2: Heightfield data scaling (the real killer)
`model.hfield_data` in MuJoCo Python bindings is a float array expecting [0, 1] values. Code was applying uint16 scaling: `(data * 65535).astype(np.uint16)`. This wrote values like 546.0 to the float array, which MuJoCo interpreted as terrain at 546 × 6.0 = 3276m height — massive walls enclosing the robot.

**Fix**: Write normalized float data directly: `model.hfield_data[:] = hfield_data`

#### Terrain baselines (Run 16d policy, pre-training)

| Terrain | Speed | Survival | Tilt |
|---------|-------|----------|------|
| flat | 0.288 | 100% | 2.1° |
| rough | 0.285 | 100% | 3.2° |
| obstacles | 0.282 | 100% | 2.4° |
| stairs_up | 0.200 | 60% | 28.4° |
| slope_30 | 0.244 | 40% | 26.1° |

#### Stage 1 — flat + rough (5M steps, fine-tune from Run 16d)

| Terrain | Speed | Survival | Tilt |
|---------|-------|----------|------|
| flat | 0.307 | 100% | 1.3° |
| rough | 0.301 | 100% | 2.1° |
| obstacles | 0.303 | 90% | 5.4° |
| stairs_up | 0.205 | 70% | 19.9° |
| slope_30 | 0.223 | 60% | 21.0° |

#### Stage 2 — flat + rough + obstacles (5M steps, fine-tune from Stage 1)

| Terrain | Speed | Survival | Tilt |
|---------|-------|----------|------|
| flat | 0.306 | 100% | 1.4° |
| rough | 0.304 | 100% | 2.8° |
| obstacles | 0.294 | **100%** | 1.7° |
| stairs_up | 0.184 | **80%** | 13.6° |
| slope_30 | 0.195 | 60% | 22.7° |

#### Stage 3 — full mix including stairs + slopes (10M steps)

| Terrain | Speed | Survival | Tilt | vs. Stage 2 |
|---------|-------|----------|------|-------------|
| flat | 0.236 | 100% | 2.3° | -0.07 speed |
| rough | 0.222 | 100% | 3.5° | ≈same |
| obstacles | 0.230 | **100%** | 4.2° | ≈same |
| stairs_up | 0.170 | **90%** | 8.1° | +10% surv |
| slope_30 | 0.161 | **100%** | 4.1° | **+40% surv!** |
| slope_debris | 0.168 | **90%** | 7.4° | new terrain |

**Key finding**: Terrain training significantly improved terrain robustness (slopes 60→100%!) but caused speed regression on flat (0.30→0.24) and drift regression (0.01→0.5m). The reward weights (optimized for flat only) aren't optimal for multi-terrain training.

---

### HPO v4 — Multi-Terrain Reward Optimization (100 trials, overnight)

**Goal**: Find reward weights that maintain speed while improving terrain robustness.

**Design**:
- Base policy: Stage 2 checkpoint (best flat/rough/obstacles walker)
- Fine-tune 2M steps on ALL 5 terrain types per trial
- Evaluate across all 5 terrains
- 12 search parameters (terrain-relevant rewards)
- Weighted objective: stairs 25%, flat 20%, obstacles 20%, slopes 20%, rough 15%
- ~10 min/trial, 100 trials ≈ 16 hours

**Status**: Running in tmux window 'hpo-v4'
