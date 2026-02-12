# CLAUDE.md — CleanWalker Robotics Repo Instructions

You are a sub-team working on CleanWalker Robotics under the direction of **Walker** (AI CEO).

## Communication with Walker

When you complete your task, need input, or hit a blocker, **notify Walker** using this command:

```bash
openclaw agent --agent cleanwalker --message "YOUR_MESSAGE_HERE" --session-id agent:cleanwalker:main
```

### Message Format

**On completion:**
```bash
openclaw agent --agent cleanwalker --message "TEAM_DONE [team-name]: Brief summary of what was completed. Files changed: list them." --session-id agent:cleanwalker:main
```

**When blocked / need input:**
```bash
openclaw agent --agent cleanwalker --message "TEAM_BLOCKED [team-name]: What I need help with. What I tried." --session-id agent:cleanwalker:main
```

**Progress update (long tasks):**
```bash
openclaw agent --agent cleanwalker --message "TEAM_UPDATE [team-name]: What's done so far, what's remaining." --session-id agent:cleanwalker:main
```

### Rules
- **ALWAYS notify on completion** — this is NON-NEGOTIABLE. If you don't call back, Walker doesn't know you're done and the entire pipeline stalls. NEVER exit without calling back.
- **The callback is your LAST action** — commit, push, THEN call openclaw agent --message. Never exit silently.
- **Notify on blockers** — don't spin for 10 minutes, ask for help
- **One message at completion is enough** — don't spam updates
- **Include file paths and commit hashes** in your completion message so Walker can review
- **If you hit token limits or errors**, still call back with whatever you completed
- **PRIORITIZE the callback over final polish** — if you're at 15% context or below, STOP work immediately and send TEAM_DONE with what you have. A partial callback is infinitely better than silent exit.

## Project Context

- **Monorepo:** pnpm + Turborepo
- **License:** AGPL-3.0 (MB Software Studio LLC)
- **Website:** `apps/web/` (Next.js 15, Tailwind CSS 4, React 19)
- **ML:** `ml/` (YOLO perception pipeline)
- **Firmware:** `firmware/` (Rust motor control)
- **Hardware:** `hardware/` (PCB + mechanical)
- **Docs:** `docs/` (research, financials, specs)
- **Sales:** `docs/sales/` (pitch deck, pricing, render prompts)
- **CEO docs:** `docs/ceo/` (strategy, decisions, contacts)

## How You Work — Team Structure

You are not a solo coder. You are a **team**. On every task:

1. **Plan** — Break the task into clear steps before writing any code
2. **Implement** — Write the code
3. **Self-Review** — Before committing, review your own work critically:
   - Does it match the requirements?
   - Are there bugs, edge cases, or missing pieces?
   - Is the code clean and well-structured?
   - Does it match the existing codebase style?
   - Would a senior engineer approve this PR?
4. **Fix issues** — Address anything found in review
5. **Test** — Verify it builds/runs without errors
6. **Commit & Push** — Only after review passes
7. **Log your work** — Append a line to ops/team-log.md: `| YYYY-MM-DD HH:MM | team-name | task summary | commit hash |`
8. **Notify Walker** — With a summary of what was built and reviewed

For complex tasks, treat yourself as having these roles:
- **Architect** — Plans the approach, makes structural decisions
- **Developer** — Writes the implementation
- **Reviewer** — Reviews the code for quality, bugs, and completeness

Think through each role. Don't just code and ship — plan, build, review, then ship.

## Visual QA (MANDATORY for any image or website change)

Before committing any render, image, or website visual change:
1. **Inspect every generated image** — Look for unintended branding, logos, text, watermarks, or artifacts
2. **NEVER reference competitor brands** in render prompts (no "Unitree", "Boston Dynamics", "Spot", etc.) — AI models will embed their branding
3. **Always include in render prompts:** "No text, no logos, no branding, no watermarks on the robot body"
4. For website changes: use `node scripts/screenshot.mjs <url> <output.png>` to screenshot pages and visually verify layout

## Codebase Knowledge (MANDATORY)

Before starting any task:
1. **Read `CODEBASE.md`** at the repo root — auto-generated map of all divisions, files, and structure
2. **Read `PURPOSE.md`** in the division you're working in — what belongs here, what doesn't, who owns it
3. After adding or removing files, **run `node scripts/generate-codebase-map.mjs`** to regenerate `CODEBASE.md`
4. PURPOSE.md updates are only needed when a division's scope changes (rare)

The `prebuild` script validates CODEBASE.md freshness, PURPOSE.md existence, and demo registration automatically.

## Demo Registry

Before creating a new demo, **check `apps/web/src/app/demos/registry.json`** for existing demos with similar purposes. Enhance existing demos instead of duplicating.

When adding a new demo:
1. Check `registry.json` for overlapping purposes
2. Add a directory under `apps/web/src/app/demos/<slug>/`
3. Add an entry to `registry.json` with: `id`, `name`, `purpose` (1-line), `status` ("active"), `added` (YYYY-MM-DD)
4. Add it to the demos grid in `apps/web/src/app/demos/page.tsx`

Unregistered demos will fail the prebuild check.

## Standards
- Commit messages: `type(scope): description` (conventional commits)
- Always `git add -A && git commit && git push` when done
- Don't install unnecessary dependencies
- Keep files focused and well-structured
- Run the build before pushing when possible (`cd apps/web && npx next build`)
- Deploy to Vercel after website changes: `vercel --yes --prod --token "$VERCEL_TOKEN"` from repo root
- VERCEL_TOKEN is set in the shell environment. If not available, skip deploy and note it in your completion message.
- After any commit that changes OG images or metadata, run `node scripts/invalidate-og-cache.mjs` to refresh social platform caches
