# CAD Design Agent

## Project Overview

AI-powered CAD design pipeline: natural language → CadQuery code → 3D model → visual verification → iterative refinement.

## Architecture: Subagent Pipeline (not Agent Teams)

This project uses **subagents**, not agent teams. The pipeline is sequential — each step depends on the previous step's output. Agent teams are for parallel independent work. This is a serial refinement loop.

```
design_description.md
        │
        ▼
  ┌─ cad-generator ─┐    ← subagent (sonnet): generates CadQuery code
  └────────┬─────────┘
           ▼
  ┌─ cad-executor ──┐    ← subagent (sonnet): runs code + error correction
  └────────┬─────────┘
           ▼
  ┌─ renderer ──────┐    ← subagent (haiku): renders 6 views
  └────────┬─────────┘
           ▼
  ┌─ evaluator ─────┐    ← subagent (opus): VQA visual verification
  └────────┬─────────┘
           ▼
     Lead reads eval
     PASS? → done
     FAIL? → loop with feedback (max 5 iterations)
```

## Directory Structure

```
├── CLAUDE.md
├── design_description.md       # INPUT: what to build
├── agents/
│   ├── cad-generator.md        # Code generation subagent
│   ├── cad-executor.md         # Execution + error correction subagent
│   ├── renderer.md             # Multi-angle rendering subagent
│   └── evaluator.md            # Visual verification subagent
├── models/
│   ├── model.py                # Current CadQuery script
│   ├── model.step              # STEP export
│   ├── model.stl               # STL export
│   └── execution_report.md     # Execution status
├── renders/                    # 6 × 1024px PNG views
│   └── render_report.md
├── evals/
│   └── eval_N.md               # Evaluation report per iteration
├── scripts/
│   ├── render_views.py         # PyVista headless renderer
│   ├── validate_mesh.py        # Mesh validation (trimesh)
│   └── setup.sh                # Dependency installer
└── iteration_log.md            # Running log of all iterations
```

## Lead Agent Orchestration Protocol

You are the lead agent. Follow this protocol exactly.

### Initial Setup (first run only)

```bash
cd hardware/cad-agent
chmod +x scripts/setup.sh && bash scripts/setup.sh
```

### Design Input

The design description lives in `hardware/cad-agent/design_description.md`. For CleanWalker components, extract the relevant subsystem from `docs/design/robot-design-spec.md` and write a focused design description into `design_description.md` before starting the loop.

### Design Loop

For iteration N (starting at 1):

**Step 1 — Generate**
Invoke `cad-generator` subagent. Provide it:
- Contents of `design_description.md`
- If N > 1: contents of the latest `evals/eval_{N-1}.md` and `models/model.py`

Wait for it to write `models/model.py`.

**Step 2 — Execute**
Invoke `cad-executor` subagent. It will:
- Run `models/model.py`
- Handle error correction internally (up to 5 retries)
- Write `models/execution_report.md`
- Produce `models/model.step` and `models/model.stl`

If execution_report says FAILED → log to iteration_log.md, go back to Step 1 with the traceback. This counts as an iteration.

**Step 3 — Render**
Invoke `renderer` subagent. It will:
- Run `scripts/render_views.py`
- Write 6 PNGs to `renders/`
- Write `renders/render_report.md`

**Step 4 — Evaluate**
Invoke `evaluator` subagent. Provide it:
- `design_description.md`
- All 6 renders from `renders/`
- `models/model.py`
- `models/execution_report.md`
- `iteration_log.md`

It writes `evals/eval_N.md`.

**Step 5 — Decide**
Read `evals/eval_N.md`:

- If `VERDICT: PASS` → **Done.** Report final results to user.
- If `VERDICT: FAIL` and N < 5 → Append to `iteration_log.md`:
  ```
  ## Iteration N — FAIL
  Score: X%
  Key issues: <from eval>
  ```
  Go to Step 1 with N+1.
- If `VERDICT: FAIL` and N = 5 → **Done.** Report best result with caveats.

### Final Report

When complete, summarize:
- Total iterations
- Final score
- What was achieved vs what was requested
- Links to output files: `models/model.step`, `models/model.stl`
- Any unresolved issues from the final eval

## Rules

- All dimensions in millimeters
- CadQuery 2.x API only
- Maximum 5 design iterations
- Always log iterations
- Never skip the visual evaluation step
- If dependencies are missing, run setup.sh before proceeding
