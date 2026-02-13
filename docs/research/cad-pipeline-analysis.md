# CAD Pipeline Analysis — Bag System Lessons Learned

**Date:** 2026-02-13
**Target:** CW-1 Bag System Assembly (hardware/cad-agent)
**Iterations:** 4 (68% → 28% → 92% → 100%)

## Executive Summary

The CAD agent pipeline successfully produced a 25/25 (100%) bag system model in 4 iterations. However, iteration 2 caused a catastrophic regression from 68% to 28%, wasting an entire cycle. Without this regression, the pipeline could have reached 100% in 2-3 iterations. This analysis identifies root causes, failure patterns, and concrete improvements to target ≤2 iterations for future components.

---

## 1. Iteration 2 Regression: Root Cause Analysis

### What happened
The cad-generator attempted to fix "frame tubes disconnected at corners" (eval_1, check #5) by replacing 4 individual tube extrusions with a single `sweep()` of a circle profile along a closed rectangular wire path.

### Why it failed
In CadQuery 2.x, `wire.sweep(profile)` on a **closed** wire fills the interior enclosed by the wire, producing a solid slab — not a hollow pipe following the wire edges. This is a fundamental CadQuery behavior, not a bug.

### Cascade effects
| Effect | Impact |
|--------|--------|
| Solid slab frame | Obscured all other components in 6/6 renders |
| 4 checks → CANNOT VERIFY | Hidden behind solid (hinge brackets, servo void, support bars, outer clips) |
| 8 checks → FAIL | Solid shape wrong, proportions wrong, components indistinguishable |
| Bounding box bloated | 466mm depth vs 370mm target (26% over) |
| Support bar double-offset | Sketched at Y=hinge_y, rotated around origin, translated by hinge_y again |

### The double-offense pattern
Iteration 2 committed two errors simultaneously:
1. **Replaced working approach** (4 individual tubes) with untested alternative (sweep on closed path)
2. **Introduced new bug** (support bar double-positioning) while fixing unrelated issue

This is the most dangerous anti-pattern: making multiple independent changes in one iteration, where one catastrophic change masks all other progress.

### Prevention rules
1. **NEVER use sweep() on a closed wire path** — codify as an explicit rule
2. **One structural change per iteration** — fix one category of issues at a time
3. **Preserve working code** — when fixing issue X, don't refactor unrelated working code Y
4. **Self-test before execution** — generator should mentally trace the geometry before writing

---

## 2. Failure Mode Taxonomy

Across all 4 evaluations (100 total checks), failures clustered into 5 categories:

### Category A: CadQuery API Misuse (most severe)
| Failure | Iteration | Checks lost |
|---------|-----------|-------------|
| sweep() on closed path → solid slab | 2 | ~15 checks |
| .hole() on >Z face when pin runs along X | 1-3 | cosmetic (not scored) |

**Root cause:** LLM lacks experiential knowledge of CadQuery's edge-case behaviors. The sweep behavior is non-obvious — in many other CAD tools, sweeping along a closed path creates a pipe.

### Category B: Coordinate/Positioning Errors
| Failure | Iteration | Checks lost |
|---------|-----------|-------------|
| Support bar double-offset (sketch at offset → rotate → translate offset) | 2 | 2 checks |
| Servo void positioned outside bracket (X=-52.5 vs bracket at X=-45 to -20) | 1-3 | 1 check |
| Frame depth accumulation (support bar end + full frame depth > target) | 1-3 | 1 check |

**Root cause:** 3D coordinate math with rotations is error-prone. The "build at origin → rotate → translate once" pattern was discovered in iteration 3 and should be the default.

### Category C: Missing Features
| Failure | Iteration | Checks lost |
|---------|-----------|-------------|
| Servo clearance void: parameters defined, never used | 1 | 1 check |
| Frame corners disconnected (no overlap extension) | 1 | 2 checks |

**Root cause:** Generator defines parameters but forgets to use them. No automated check for unused parameters.

### Category D: Wrong Geometry Type
| Failure | Iteration | Checks lost |
|---------|-----------|-------------|
| Roll housing: rectangular box instead of curved cradle | 1 | 1 check |
| Hinge brackets: rectangular blocks instead of L-shaped | 1 | 1 check |

**Root cause:** Generator takes shortcuts (simple rect + extrude) instead of matching the design description. These were straightforward to fix once identified.

### Category E: Spacing/Dimension Errors
| Failure | Iteration | Checks lost |
|---------|-----------|-------------|
| Outer clips use roll_width spacing on frame_width rail | 1 | 1 check |
| Overall depth 410mm vs 370mm target | 1-3 | 1 check |

**Root cause:** Copy-paste errors (reusing inner clip spacing for outer clips) and cumulative dimension miscalculation.

### Failure distribution
```
Category A (API misuse):      ~15 total check-failures  — highest impact per incident
Category B (positioning):     ~4 total check-failures   — recurring, hard to catch visually
Category C (missing features): ~3 total check-failures  — easy to prevent with parameter audit
Category D (wrong geometry):   ~2 total check-failures  — easy to fix once identified
Category E (spacing/dims):     ~2 total check-failures  — needs dimension pre-validation
```

---

## 3. Iteration Timing & Bottlenecks

### Iteration cost breakdown
Each iteration requires 4 subagent invocations:
1. **cad-generator** (Sonnet) — code generation/revision
2. **cad-executor** (Sonnet) — execution + error correction (up to 5 retries)
3. **renderer** (Haiku) — 6-view rendering
4. **evaluator** (Opus) — visual QA + code review

### Where time was wasted
| Waste | Iterations lost | Root cause |
|-------|-----------------|------------|
| Iteration 2 regression | 1 full iteration | sweep on closed path + no guardrails |
| Iteration 3 still had 2 failures | 0.5 iteration worth of fixes | Servo void + depth not fixed together |
| Iteration 4 to fix remaining 2 items | 1 iteration for 2 parameter tweaks | Could have been caught by dimension pre-check |

### Ideal path (with proposed improvements)
```
Iteration 1: Initial generation with pattern guide → 85-90% (pass threshold)
Iteration 2: Fix remaining 2-3 issues → 100%
```

The key bottleneck is **iteration 1 quality**. If the generator has access to proven patterns and known pitfalls, the first attempt should be much closer to passing.

---

## 4. Evaluation Checklist Effectiveness

### Strengths
- **25 items** covers all components, dimensions, positioning, and assembly relationships
- **View-specific checks** force the evaluator to examine the right angles
- **Code review section** catches issues invisible in renders (unused parameters, wrong axis holes)
- **Actionable feedback** with specific line numbers, parameter names, and fix suggestions
- **PARTIAL / CANNOT VERIFY** categories prevent false passes when components are obscured

### Weaknesses
1. **Watertight check always fails for assemblies** — creates noise, evaluator has to explain it every time
2. **No regression detection** — checklist doesn't flag when previously-passing checks now fail
3. **No dimension pre-computation** — evaluator manually calculates expected bounding box each time
4. **Visual verification of small features is unreliable** — L-shaped brackets, servo voids at render scale are hard to confirm visually

### Recommendations
1. Add `[REGRESSION]` tag when a check that passed in iteration N-1 now fails in iteration N
2. Add a computed "expected bounding box" section that the evaluator can compare against
3. For small features (<20mm), rely more on code review than visual inspection
4. Fix the watertight check for assemblies (see section 9)

---

## 5. CadQuery Pattern Analysis

### Patterns that work reliably

| Pattern | Use case | Example |
|---------|----------|---------|
| Individual tubes + union | Rectangular frames | 4 cylinders extended by radius at ends, union all |
| Polyline profiles | L-shapes, T-shapes, custom brackets | `cq.Workplane().polyline([...]).close().extrude(d)` |
| Cylindrical cuts for cradles | Housing/recess shapes | Box + cut(cylinder) = curved recess |
| Build at origin → rotate → translate | Any angled component | Avoids double-offset from rotation around distant point |
| cq.Assembly() for multi-component | Assemblies with distinct parts | Preserves component identity, enables color coding |
| try/except with fallback | Fillets, chamfers, boolean ops | Prevents hard failure on geometry edge cases |

### Known traps

| Trap | What happens | Correct alternative |
|------|-------------|---------------------|
| `sweep()` on closed wire | Fills interior → solid slab | Individual tubes + union |
| Sketch at offset → rotate around origin → translate by offset | Double-offset (position applied twice) | Sketch at origin → rotate → translate once |
| `.hole()` on wrong face selector | Hole drilled perpendicular to intended axis | Select the correct face (e.g., `">Y"` not `">Z"`) |
| `.shell()` on complex geometry | Frequent `StdFail_NotDone` errors | Build hollow geometry manually |
| Reusing spacing variables across different-width rails | Off-center placement | Compute separate spacing per rail width |
| Boolean `cut()` where bodies don't overlap | Silent no-op (no material removed) | Verify overlap in coordinate math before cutting |

---

## 6. Reducing Iterations to ≤2

### Strategy: Front-load quality into iteration 1

**Current flow:**
```
Generate (blind) → Execute → Render → Evaluate → Fix → Loop
```

**Proposed flow:**
```
Generate (with pattern guide + pitfall list) → Self-validate (dimension check) → Execute → Render → Evaluate → Fix → Done
```

### Specific improvements

#### 6.1 Pattern reference document
Create `hardware/cad-agent/prompts/cadquery-patterns.md` with proven patterns and known traps. The cad-generator reads this before writing code. **[DELIVERABLE 3]**

#### 6.2 Dimension pre-validation in generated code
The generator should include a comment block computing expected bounding box:
```python
# DIMENSION CHECK
# Roll front: Y ≈ roll_center_y - roll_diameter/2 = -40mm
# Frame tip: Y ≈ hinge_y + support_bar * cos(45°) + frame_depth * cos(45°) = 362mm
# Total depth ≈ 362 - (-40) = 402mm vs 370mm target → CLOSE ENOUGH / NEEDS ADJUSTMENT
```
This forces the generator to verify its own math before the evaluator catches it.

#### 6.3 Parameter usage audit
Add a check in the executor: grep for all variable assignments in PARAMETERS section and verify each is referenced in CONSTRUCTION section. Flag unused parameters.

#### 6.4 Known-pitfall linting
Add a pre-execution check that scans model.py for known dangerous patterns:
- `sweep(` following a closed wire (`.close()` + `.sweep()`)
- `.move(` with non-zero coordinates followed by `.rotate(` and `.translate(` (double-offset risk)
- Parameters defined but never referenced

#### 6.5 Body plate inclusion
Iteration 4 added a body plate for mounting context. This should be standard for all components that mount to the robot body — it provides visual reference in renders.

#### 6.6 Preserve working code rule
Add to CLAUDE.md: "When fixing evaluation failures, modify ONLY the code related to failing checks. Do NOT refactor or restructure working code. If a fix requires changing the approach for a component, only change that component."

### Expected impact
| Improvement | Iterations saved | Confidence |
|-------------|-----------------|------------|
| Pattern guide (prevents sweep trap) | 1 full iteration | High |
| Dimension pre-validation | 0.5 iteration | Medium |
| Preserve working code rule | 0.5 iteration | High |
| Parameter usage audit | 0.25 iteration | Medium |
| **Total** | **~2 iterations saved** | |

Target: 2 iterations for simple components, 3 for complex assemblies.

---

## 7. Pre-Check Automation Recommendations

### Immediate (implement now)

1. **Assembly-aware mesh validation** — don't fail on non-watertight when assembly has multiple components **[DELIVERABLE 4]**
2. **Bounding box assertion** — generator writes expected bounds, executor checks actual vs expected
3. **Parameter usage check** — flag defined-but-unused parameters

### Future (implement when pipeline matures)

4. **Component overlap detection** — check that components don't accidentally intersect (except at intentional joints)
5. **Minimum feature size check** — flag features smaller than 1mm (likely errors)
6. **Symmetry validation** — if design says "symmetric", check left/right bounding boxes match

---

## 8. Render Quality Assessment

### Current state
- 1024x1024 PNG, 6 views (front, back, left, right, top, isometric)
- Single steel-blue color (STL has no material data)
- Light gray background with axis indicator
- PyVista headless rendering

### What works
- Resolution is sufficient for structural verification
- 6 views cover all angles
- Axis indicator helps orientation

### What's lacking
1. **Single color** — can't distinguish components in assembly. Evaluator can't tell if the roll housing is separate from the body plate.
2. **No wireframe** — small features (clips, brackets) are hard to see as solid fills blend together
3. **No dimension annotations** — evaluator manually estimates proportions

### Recommendations
1. **Short term:** Render STEP files instead of STL to preserve per-component colors from cq.Assembly(). This requires modifying render_views.py to use CadQuery's STEP → mesh conversion or OCP viewer.
2. **Medium term:** Add wireframe overlay mode for a 7th render that shows edges clearly.
3. **Long term:** Add bounding box overlay with dimension labels on the isometric view.

---

## 9. Multi-Component Assembly Validation

### The problem
`validate_mesh.py` checks `mesh.is_watertight`, which always returns `False` for assemblies exported to STL because each component is a separate shell. The validator then fails on `positive_volume` (which requires watertight). This creates a guaranteed failure for every assembly, forcing the evaluator to manually dismiss it.

### Solution
Detect whether the mesh is a single solid or a multi-component assembly using trimesh's `split()` method. For assemblies:
- Split into individual components
- Validate each component independently
- Report per-component results
- Only fail if a component has degenerate geometry, not because the assembly isn't watertight as a whole

**[DELIVERABLE 4]** — Updated validate_mesh.py

---

## 10. Codified Lessons for All Future CAD Targets

### Rules (add to CLAUDE.md)

1. **NEVER use `sweep()` on a closed wire path.** It fills the interior. Use individual tubes + union instead.
2. **Build geometry at the origin, rotate, then translate ONCE.** Never sketch at an offset and then rotate around the origin — this causes double-offset.
3. **Every parameter defined MUST be used in construction.** If you define `servo_clearance_width`, you must have a construction step that uses it.
4. **Include a body plate** for any component that mounts to the robot body. This provides visual context in renders.
5. **Include a DIMENSION CHECK comment block** computing expected bounding box before the assembly section.
6. **When fixing evaluation failures, ONLY modify failing components.** Do not refactor working code.
7. **One structural change per iteration.** Don't replace a working approach AND add a new feature in the same iteration.
8. **Use try/except with simple fallbacks** for complex geometry operations (fillets, chamfers, boolean cuts).
9. **Compute spacing per rail, not globally.** Inner clips use roll_width spacing; outer clips use frame_width spacing.
10. **Verify boolean cut overlap.** Before `A.cut(B)`, check that A and B bounding boxes overlap, or the cut silently does nothing.

### Patterns reference (add to prompts/)

**[DELIVERABLE 3]** — cadquery-patterns.md with proven patterns and known pitfalls.

---

## Appendix: Score Progression

```
Iteration 1:  ████████████████████░░░░░░░░░░  17/25 (68%)
Iteration 2:  ███████░░░░░░░░░░░░░░░░░░░░░░░   7/25 (28%) ← REGRESSION
Iteration 3:  ███████████████████████░░░░░░░░  23/25 (92%)
Iteration 4:  ██████████████████████████████  25/25 (100%)
```

### Per-check progression
| Check | Iter 1 | Iter 2 | Iter 3 | Iter 4 |
|-------|--------|--------|--------|--------|
| Roll cylinder | ✅ | partial | ✅ | ✅ |
| Roll orientation | ✅ | partial | ✅ | ✅ |
| Curved cradle | ❌ | partial | ✅ | ✅ |
| Side brackets | ✅ | partial | ✅ | ✅ |
| Frame tubular rim | ❌ | ❌ | ✅ | ✅ |
| Frame dimensions | ❌ | ❌ | ❌ | ✅ |
| Frame angle | ✅ | partial | ✅ | ✅ |
| Support bars | ✅ | N/V | ✅ | ✅ |
| Support bar position | ✅ | partial | ✅ | ✅ |
| Hinge pin | ✅ | barely | ✅ | ✅ |
| L-shaped brackets | ✅* | N/V | ✅ | ✅ |
| Bracket position | ✅ | ✅ | ✅ | ✅ |
| Inner clips | ✅ | partial | ✅ | ✅ |
| Outer clips | ✅ | N/V | ✅ | ✅ |
| Inner clip spacing | ✅ | ✅ | ✅ | ✅ |
| Outer clip spacing | ❌ | ✅ | ✅ | ✅ |
| Servo void | ❌ | N/V | ❌ | ✅ |
| L-shaped profile | ❌ | ❌ | ✅ | ✅ |
| Curved housing | ❌ | partial | ✅ | ✅ |
| Bounding box | ✅ | ❌ | ✅ | ✅ |
| Frame-hinge connection | ✅ | ❌ | ✅ | ✅ |
| Roll positioning | ✅ | ✅ | ✅ | ✅ |
| Mesh validation | ✅ | ❌ | ✅ | ✅ |
| Pin height | ✅ | ✅ | ✅ | ✅ |
| Frame cross-tubes | ❌ | ❌ | ✅ | ✅ |

*Iter 1: bracket existed but was rectangular, not L-shaped (scored ✅ for existence, ❌ for shape)
