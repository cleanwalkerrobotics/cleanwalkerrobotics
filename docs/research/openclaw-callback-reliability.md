# OpenClaw Reliable Callback & Notification System

**Date:** 2026-02-12
**Author:** ops team (Claude Code research agent)
**Status:** Implemented

---

## Problem Statement

Claude Code sub-teams run in tmux sessions. When they finish, they call back via:

```bash
openclaw agent --agent cleanwalker --session-id agent:cleanwalker:main --message "TEAM_DONE: ..."
```

This is **unreliable**. The Walker agent session may be compacted, idle, or not processing callbacks into Telegram notifications. Maurits (the human CEO) often never gets pinged about completed work.

---

## Current Callback Flow Analysis

### How `openclaw agent --message` works

1. CLI connects to gateway WebSocket at `localhost:18789`
2. Gateway resolves session key `agent:cleanwalker:main`
3. Gateway enqueues the message in the per-session queue
4. An **agent turn** is triggered — the LLM processes the message
5. The LLM must decide to notify Maurits (or not)
6. If the LLM replies, the reply is routed via the cleanwalker binding (`accountId: ceo`) to Telegram

### Failure Points Identified

| # | Failure Point | Impact | Likelihood |
|---|---------------|--------|------------|
| 1 | **Session compacted** — Walker's context has been summarized, callback context lost | Agent may not understand the callback or forget to notify | HIGH |
| 2 | **Session idle/expired** — daily reset at 4 AM or idle expiration | New session has no context about pending teams | HIGH |
| 3 | **Gateway restarting** — WebSocket unavailable during restart | CLI fails silently or errors out | MEDIUM |
| 4 | **Agent turn timeout** — 600s default, but complex reasoning may time out | Callback acknowledged but notification never sent | LOW |
| 5 | **LLM decides not to notify** — agent may process but not forward to Telegram | Silent completion, Maurits never knows | MEDIUM |
| 6 | **Queue collision** — another run active on same session key | Message queued, may be collected/summarized with other pending messages | MEDIUM |
| 7 | **Heartbeat suppression** — agent replies HEARTBEAT_OK, stripping actual content | Callback content lost in heartbeat noise | LOW |

**Root cause:** The current flow requires an LLM agent turn to relay a notification. This adds multiple failure points where a simple direct message delivery would suffice.

---

## Notification Methods — Ranked by Reliability

### 1. Direct Telegram Bot API (curl) — HIGHEST RELIABILITY

```bash
curl -s -X POST "https://api.telegram.org/bot${BOT_TOKEN}/sendMessage" \
  -H "Content-Type: application/json" \
  -d "{\"chat_id\": \"${CHAT_ID}\", \"text\": \"${MESSAGE}\"}"
```

**Tested:** Message ID 917 delivered successfully.

| Pros | Cons |
|------|------|
| Zero dependencies on gateway/agent | Bypasses gateway logging |
| Works if gateway is down | Bot token hardcoded in script |
| Immediate delivery | No agent context enrichment |
| No LLM turn required | Doesn't wake Walker session |

### 2. `openclaw message send` — HIGH RELIABILITY

```bash
openclaw message send --channel telegram --account ceo --target 1602324097 --message "TEXT"
```

**Tested:** Message ID 925 delivered successfully.

| Pros | Cons |
|------|------|
| Uses gateway channel system (proper routing) | Requires gateway to be running |
| Retry logic built into gateway | Slightly slower than direct curl |
| No LLM turn required | Requires openclaw CLI |
| Logged in gateway system | |
| One-liner CLI command | |

### 3. `/hooks/agent` endpoint — MEDIUM RELIABILITY

```bash
curl -s -X POST http://localhost:18789/hooks/agent \
  -H "Authorization: Bearer ${TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "...", "deliver": true, "channel": "telegram", "to": "1602324097"}'
```

**Tested:** Returns `runId`, triggers agent run.

| Pros | Cons |
|------|------|
| Can run in isolated session | Triggers full agent turn (LLM cost) |
| Can deliver directly to channel | Agent may not relay properly |
| Supports custom session keys | 600s timeout risk |

### 4. `openclaw agent --message` (current method) — LOW RELIABILITY

The current approach. All failure points listed above apply.

### 5. `/hooks/wake` endpoint — NOT SUITABLE

Only enqueues a system event for the next heartbeat. Does not send a message directly to Telegram.

### 6. `/tools/invoke` endpoint — NOT USABLE

No messaging tools are registered. All tool names return "not found" errors.

---

## Recommended Approach: Dual-Path Notification

**Primary:** `openclaw message send` — Goes through gateway, has retry logic, logged.
**Fallback:** Direct Telegram Bot API — Works if gateway is down.
**Optional:** `openclaw agent --message` — For Walker session context (non-blocking).

### Why dual-path?

- `openclaw message send` handles 95% of cases with proper logging
- Direct Telegram ensures delivery even during gateway maintenance
- The Walker agent session gets a context message so it knows teams completed

---

## Implementation

### Script: `scripts/team-callback.sh`

Usage:
```bash
./scripts/team-callback.sh "TEAM_DONE [hardware]: CAD model generated. Commit: abc1234"
```

The script:
1. Sends notification via `openclaw message send` (primary)
2. Falls back to direct Telegram Bot API if gateway unavailable
3. Optionally notifies Walker agent session for context (non-blocking)
4. Logs the callback to `ops/team-log.md`

### Configuration

| Setting | Value |
|---------|-------|
| Gateway | `localhost:18789` |
| Gateway Token | `$OPENCLAW_GATEWAY_TOKEN` (from `~/.openclaw/.env`) |
| Telegram Bot | BrainsCEOBot (`accountId: ceo`) |
| Telegram Chat ID | `1602324097` (Maurits) |
| Session Key | `agent:cleanwalker:main` |

---

## CLAUDE.md Update Required

Replace the callback instruction:
```bash
# OLD (unreliable):
openclaw agent --agent cleanwalker --message "TEAM_DONE: ..." --session-id agent:cleanwalker:main

# NEW (reliable):
./scripts/team-callback.sh "TEAM_DONE [team-name]: Summary. Commit: hash"
```

---

## Gateway Documentation Reference

Key docs consulted from `/home/deploy/.npm-global/lib/node_modules/openclaw/docs/`:

- `docs/cli/message-send.md` — `openclaw message send` command
- `docs/cli/agent.md` — `openclaw agent` command
- `docs/gateway/configuration.md` — Gateway config, bindings, hooks
- `docs/automation/cron.md` — Cron jobs and delivery
- `docs/automation/webhooks.md` — Webhook endpoints (/hooks/wake, /hooks/agent)
- `docs/concepts/session.md` — Session lifecycle, compaction, queuing
- `docs/hooks.md` — Hook system, event handlers

---

## Appendix: Test Results

### Test 1: Direct Telegram Bot API
```
POST https://api.telegram.org/bot[TOKEN]/sendMessage
→ 200 OK, message_id: 917, delivered to Maurits (chat_id: 1602324097)
```

### Test 2: `openclaw message send`
```
openclaw message send --channel telegram --account ceo --target 1602324097 --message "test"
→ ✅ Sent via Telegram. Message ID: 925
```

### Test 3: `/hooks/wake`
```
POST http://localhost:18789/hooks/wake
→ {"ok":true,"mode":"now"}
```

### Test 4: `/hooks/agent`
```
POST http://localhost:18789/hooks/agent
→ {"ok":true,"runId":"5d332812-6ee3-440a-a594-2557ce3a9b0d"}
```

### Test 5: `/tools/invoke`
```
POST http://localhost:18789/tools/invoke (all tool names tested)
→ {"ok":false,"error":{"type":"not_found","message":"Tool not available: send_message"}}
```
