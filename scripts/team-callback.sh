#!/usr/bin/env bash
# ============================================================================
# team-callback.sh — Reliable notification for Claude Code sub-teams
# ============================================================================
# Usage:
#   ./scripts/team-callback.sh "TEAM_DONE [hardware]: CAD model complete. Commit: abc1234"
#   ./scripts/team-callback.sh "TEAM_BLOCKED [ml]: Need GPU access"
#   ./scripts/team-callback.sh "TEAM_UPDATE [web]: Homepage redesign 50% done"
#
# Notification flow:
#   1. PRIMARY: openclaw agent --message → Walker CEO session (processes and routes)
#   2. FALLBACK: /tools/invoke → Walker session via HTTP API
#
# Walker (CEO) receives all callbacks, processes them, and decides what to
# forward to Maurits. Raw team callbacks should NOT go directly to Maurits.
# ============================================================================

set -euo pipefail

# --- Configuration -----------------------------------------------------------
AGENT_ID="cleanwalker"
SESSION_KEY="agent:cleanwalker:main"
GATEWAY_URL="http://localhost:18789"

# --- Input validation --------------------------------------------------------
if [ $# -eq 0 ] || [ -z "${1:-}" ]; then
  echo "Usage: $0 \"TEAM_DONE [team-name]: Summary. Commit: hash\""
  exit 1
fi

MESSAGE="$1"
TIMESTAMP=$(date -u +"%Y-%m-%d %H:%M UTC")

# Add timestamp to message
TIMESTAMPED="[${TIMESTAMP}] ${MESSAGE}"

# --- Primary: openclaw agent --message ---------------------------------------
primary_send() {
  echo "[callback] Attempting primary: openclaw agent --message..."
  if command -v openclaw &>/dev/null; then
    if openclaw agent \
      --agent "$AGENT_ID" \
      --session-id "$SESSION_KEY" \
      --message "$TIMESTAMPED" 2>&1; then
      echo "[callback] ✅ Delivered to Walker session"
      return 0
    else
      echo "[callback] ⚠️  openclaw agent delivery failed"
      return 1
    fi
  else
    echo "[callback] ⚠️  openclaw CLI not found"
    return 1
  fi
}

# --- Fallback: HTTP API to gateway -------------------------------------------
fallback_send() {
  echo "[callback] Attempting fallback: gateway /tools/invoke..."

  # Load gateway token
  local TOKEN="${OPENCLAW_GATEWAY_TOKEN:-}"
  if [ -z "$TOKEN" ] && [ -f ~/.openclaw/.env ]; then
    TOKEN=$(grep -m1 'GATEWAY_TOKEN\|OPENCLAW_GATEWAY_TOKEN' ~/.openclaw/.env 2>/dev/null | cut -d= -f2 || true)
  fi

  if [ -z "$TOKEN" ]; then
    echo "[callback] ⚠️  No gateway token available"
    return 1
  fi

  RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
    "${GATEWAY_URL}/tools/invoke" \
    -H "Authorization: Bearer ${TOKEN}" \
    -H "Content-Type: application/json" \
    -d "{\"tool\":\"message\",\"args\":{\"action\":\"send\",\"channel\":\"telegram\",\"target\":\"1602324097\",\"accountId\":\"ceo\",\"message\":\"${TIMESTAMPED}\"}}" \
    --connect-timeout 10 \
    --max-time 15 2>&1)

  HTTP_CODE=$(echo "$RESPONSE" | tail -1)

  if [ "$HTTP_CODE" = "200" ]; then
    echo "[callback] ✅ Fallback delivery successful"
    return 0
  else
    echo "[callback] ❌ Fallback failed: HTTP $HTTP_CODE"
    return 1
  fi
}

# --- Execute notification chain ----------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[callback] Sending: ${MESSAGE:0:80}..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Try primary (to Walker session)
if primary_send; then
  exit 0
fi

# If primary failed, try fallback (HTTP API)
if fallback_send; then
  exit 0
fi

# --- All failed --------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[callback] ❌ ALL delivery methods failed!"
echo "[callback] Message: '$MESSAGE'"
exit 1
