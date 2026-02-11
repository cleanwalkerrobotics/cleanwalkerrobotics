# Daily General Assembly Protocol

Runs daily at 12:00 UTC via cron. Produces a cross-functional status report.

## Phases

### Phase 1: State Capture
- Check tmux sessions (`tmux list-sessions | grep cw-`)
- Read ops/team-status.json, ops/team-log.md
- Review git log since midnight UTC
- Check outreach tracker for reply status

### Phase 2: Department Reports
Compile status for each function:
- **Engineering:** Software, hardware, simulation, ML
- **BizDev:** Outreach, grants, partnerships
- **Research:** Market, regulatory, competitive intel
- **Operations:** Infrastructure, docs, processes

### Phase 3: Assembly Report
Write to `ops/assembly/YYYY-MM-DD/report.md`:
- Executive summary (3-5 bullets)
- Department status tables
- Blockers & risks
- Priority actions for next 24h
- Metrics (commits, files changed, pipeline status)

### Phase 4: Communication
- Update ops/team-status.json
- Send Telegram summary to Maurits (2-5 lines)
- Log to memory/YYYY-MM-DD.md
