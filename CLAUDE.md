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
- **Always notify on completion** — don't just exit silently
- **Notify on blockers** — don't spin for 10 minutes, ask for help
- **One message at completion is enough** — don't spam updates
- **Include file paths** in your completion message so Walker can review
- **Commit and push your work** before sending the completion message

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

## Standards
- Commit messages: `type(scope): description` (conventional commits)
- Always `git add -A && git commit && git push` when done
- Don't install unnecessary dependencies
- Keep files focused and well-structured
