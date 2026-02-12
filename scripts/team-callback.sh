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
#   1. PRIMARY:  openclaw message send → Telegram (via gateway, with retry)
#   2. FALLBACK: Direct Telegram Bot API → curl (if gateway unavailable)
#   3. CONTEXT:  openclaw agent --message → Walker session (non-blocking, best-effort)
#
# Requirements:
#   - openclaw CLI installed and gateway running (for primary path)
#   - Internet access (for fallback path)
# ============================================================================

set -euo pipefail

# --- Configuration -----------------------------------------------------------
TELEGRAM_CHAT_ID="1602324097"           # Maurits
TELEGRAM_ACCOUNT="ceo"                  # OpenClaw account binding
AGENT_ID="cleanwalker"
SESSION_KEY="agent:cleanwalker:main"

# Bot token for direct fallback (sourced from env or hardcoded)
# The CEO bot: @BrainsCeoBot
TELEGRAM_BOT_TOKEN="${TELEGRAM_BOT_TOKEN:-8597718324:AAGAoANJm-mKhZQWxRMO-ImocQPD1Y7dNIM}"

# --- Input validation --------------------------------------------------------
if [ $# -eq 0 ] || [ -z "${1:-}" ]; then
  echo "Usage: $0 \"TEAM_DONE [team-name]: message\""
  exit 1
fi

MESSAGE="$1"
TIMESTAMP=$(date -u +"%Y-%m-%d %H:%M UTC")
HOSTNAME=$(hostname -s)

# Format for Telegram (add metadata)
FORMATTED="🤖 CleanWalker Team Callback\n━━━━━━━━━━━━━━━━━━━━━━━━━━\n${MESSAGE}\n\n🕐 ${TIMESTAMP}\n🖥️ ${HOSTNAME}"

# --- Primary: openclaw message send ------------------------------------------
primary_send() {
  echo "[callback] Attempting primary: openclaw message send..."
  if command -v openclaw &>/dev/null; then
    if openclaw message send \
      --channel telegram \
      --account "$TELEGRAM_ACCOUNT" \
      --target "$TELEGRAM_CHAT_ID" \
      --message "$FORMATTED" 2>&1; then
      echo "[callback] ✅ Primary delivery successful (openclaw message send)"
      return 0
    else
      echo "[callback] ⚠️  Primary delivery failed"
      return 1
    fi
  else
    echo "[callback] ⚠️  openclaw CLI not found"
    return 1
  fi
}

# --- Fallback: Direct Telegram Bot API ---------------------------------------
fallback_send() {
  echo "[callback] Attempting fallback: direct Telegram Bot API..."

  # Escape special characters for JSON
  ESCAPED_MSG=$(printf '%s' "$FORMATTED" | python3 -c 'import sys,json; print(json.dumps(sys.stdin.read()))' 2>/dev/null || printf '"%s"' "$FORMATTED")

  RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
    "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
    -H "Content-Type: application/json" \
    -d "{\"chat_id\": \"${TELEGRAM_CHAT_ID}\", \"text\": ${ESCAPED_MSG}}" \
    --connect-timeout 10 \
    --max-time 15 2>&1)

  HTTP_CODE=$(echo "$RESPONSE" | tail -1)
  BODY=$(echo "$RESPONSE" | head -n -1)

  if [ "$HTTP_CODE" = "200" ]; then
    echo "[callback] ✅ Fallback delivery successful (Telegram Bot API)"
    return 0
  else
    echo "[callback] ❌ Fallback delivery failed: HTTP $HTTP_CODE"
    echo "[callback] Response: $BODY"
    return 1
  fi
}

# --- Context: Notify Walker agent session (best-effort) ----------------------
context_send() {
  echo "[callback] Sending context to Walker agent session (best-effort)..."
  if command -v openclaw &>/dev/null; then
    # Run in background, don't block on agent turn
    (openclaw agent \
      --agent "$AGENT_ID" \
      --session-id "$SESSION_KEY" \
      --message "$MESSAGE" &>/dev/null &)
    echo "[callback] ✅ Context message queued for Walker session"
  fi
}

# --- Execute notification chain ----------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "[callback] Sending: ${MESSAGE:0:80}..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

DELIVERED=false

# Try primary
if primary_send; then
  DELIVERED=true
fi

# If primary failed, try fallback
if [ "$DELIVERED" = false ]; then
  if fallback_send; then
    DELIVERED=true
  fi
fi

# Always send context to Walker session (non-blocking, best-effort)
context_send

# --- Final status ------------------------------------------------------------
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ "$DELIVERED" = true ]; then
  echo "[callback] ✅ Notification delivered to Maurits via Telegram"
  exit 0
else
  echo "[callback] ❌ ALL delivery methods failed!"
  echo "[callback] Manual check: message was '$MESSAGE'"
  exit 1
fi
