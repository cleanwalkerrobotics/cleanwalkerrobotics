# CAD Design Agent — Claude Code Setup

## Why Subagents, Not Agent Teams

The CAD pipeline is **sequential** — you can't render before the model exists, and you can't evaluate before rendering. Agent teams are for parallel work where teammates need to communicate laterally. This pipeline is a strict serial loop with a lead orchestrator.

Use subagents because:
- Each step depends on the previous step's output files
- No lateral communication needed between agents
- The lead agent handles all routing and decision-making
- Lower token cost than agent teams (no coordination overhead)

Agent teams *would* make sense if you were generating **multiple variant designs in parallel** — e.g., "generate 3 different bracket designs and pick the best one." That's a future extension.

## Setup

### 1. Project structure

```bash
# Clone or copy this project to your working directory
cd your-project-dir

# The structure should look like:
# ├── CLAUDE.md
# ├── design_description.md
# ├── agents/
# │   ├── cad-generator.md
# │   ├── cad-executor.md
# │   ├── renderer.md
# │   └── evaluator.md
# ├── scripts/
# │   ├── render_views.py
# │   ├── validate_mesh.py
# │   └── setup.sh
# ├── models/
# ├── renders/
# ├── evals/
# └── iteration_log.md
```

### 2. Enable subagents

Subagents are defined in `agents/` and Claude Code picks them up automatically. No special env vars needed (unlike agent teams which need `CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1`).

### 3. Write your design description

Edit `design_description.md` with your mechanical design. Be specific about:
- All dimensions in mm
- Exact counts (holes, ribs, features)
- Spatial relationships ("centered on the top face", "8mm from each corner")
- Required features (fillets, chamfers, threads)

### 4. Claude Code kickoff prompt

Open Claude Code in this project directory and paste:

```
Read CLAUDE.md and design_description.md. You are the lead agent for a CAD
design pipeline.

First run scripts/setup.sh to install dependencies.

Then follow the orchestration protocol in CLAUDE.md exactly:
1. Invoke the cad-generator subagent to create models/model.py from the design description
2. Invoke the cad-executor subagent to run the code and handle errors
3. Invoke the renderer subagent to produce multi-angle views
4. Invoke the evaluator subagent to verify the model against the description
5. If FAIL, loop with feedback. If PASS, report results.

Maximum 5 iterations. Log every iteration to iteration_log.md.
Start now.
```

That's it. Claude Code reads CLAUDE.md for the full protocol, delegates to each subagent in sequence, and manages the refinement loop.

## Customization

### Swap the CAD engine

Replace CadQuery with Build123d or OpenSCAD by:
1. Update `agents/cad-generator.md` with the new API patterns
2. Update `agents/cad-executor.md` with the new execution command
3. Update `scripts/render_views.py` if the export format changes

### Add FEA simulation

Add a 5th subagent `agents/fea-runner.md` that:
1. Takes the `.step` file from `models/`
2. Meshes it with Gmsh
3. Runs CalculiX or Elmer with defined loads/BCs
4. Produces stress/displacement plots
5. Feeds results back to evaluator

Insert between renderer and evaluator in the CLAUDE.md protocol.

### Parallel variant generation (actual agent teams use case)

If you want to explore multiple designs simultaneously:

```bash
export CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1
```

Then prompt:

```
Create an agent team to explore 3 variant designs for the bracket in
design_description.md:
- Teammate 1: Minimal material — optimize for weight
- Teammate 2: Maximum stiffness — optimize for rigidity
- Teammate 3: Balanced — production-friendly design

Each teammate runs the full subagent pipeline independently.
When all 3 are done, compare their evaluation scores and recommend the best one.
```

This is where agent teams genuinely add value — parallel exploration with comparison.

## Model Selection Per Subagent

| Subagent | Model | Reasoning |
|---|---|---|
| cad-generator | sonnet | Best code generation cost/quality ratio |
| cad-executor | sonnet | Needs code understanding for error fixes |
| renderer | haiku | Just runs a script, minimal reasoning |
| evaluator | opus | Critical: spatial reasoning + vision for VQA |

You can override in each `agents/*.md` frontmatter. Use `model: inherit` to match whatever your Claude Code session is running.

## Token Cost Estimate

Per iteration cycle: ~20-40K tokens (mostly from evaluator + generator).
Full 5-iteration run: ~100-200K tokens.
With agent teams (3 parallel variants × 5 iterations each): ~300-600K tokens.

## Known Limitations

- CadQuery can't do involute gear teeth, complex threads, or spline surfaces well
- Visual evaluation depends on render quality — small features may not be visible at 1024px
- The evaluator can't measure exact dimensions from renders, only proportions and feature presence
- Headless PyVista rendering requires xvfb on Linux servers
- No assembly support yet (single-part only)
