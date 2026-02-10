# Inbound Email Webhook Setup

Receives emails sent to `walker@cleanwalkerrobotics.com` via Resend webhook,
normalizes them, and forwards to the OpenClaw gateway for triage.

## Architecture

```
Sender → walker@cleanwalkerrobotics.com → Resend inbound
  → POST https://cleanwalkerrobotics.com/api/email/webhook (Vercel)
  → Fetch full email body from Resend API
  → POST {gateway}/hooks/gmail (OpenClaw hook)
```

## Required Environment Variables (Vercel)

Set these in the Vercel project dashboard → Settings → Environment Variables:

| Variable | Value | Notes |
|----------|-------|-------|
| `RESEND_API_KEY` | `re_...` | Resend API key (from resend.com/api-keys) |
| `RESEND_WEBHOOK_SECRET` | `whsec_...` | Svix signing secret (from Resend webhook config) |
| `OPENCLAW_GATEWAY_URL` | See blocker below | Public URL to the OpenClaw gateway |
| `OPENCLAW_HOOK_TOKEN` | `16d6ed044c39eec7ea4c748dbf025b9f8a9304052f83e633` | Hook auth token |

## Setup Steps

### 1. Configure Resend Inbound Email

1. Go to [Resend Dashboard → Domains](https://resend.com/domains)
2. Ensure `cleanwalkerrobotics.com` is verified
3. Add MX records for inbound email (Resend provides these):
   - Priority 10: `inbound-smtp.resend.com`
   - (Or use the auto-generated `*.resend.app` address for testing)

### 2. Create Webhook in Resend

1. Go to [Resend Dashboard → Webhooks](https://resend.com/webhooks)
2. Add endpoint: `https://cleanwalkerrobotics.com/api/email/webhook`
3. Select event: `email.received`
4. Copy the signing secret → set as `RESEND_WEBHOOK_SECRET` in Vercel

### 3. Set Vercel Environment Variables

```bash
vercel env add RESEND_API_KEY
vercel env add RESEND_WEBHOOK_SECRET
vercel env add OPENCLAW_GATEWAY_URL
vercel env add OPENCLAW_HOOK_TOKEN
```

### 4. Deploy

```bash
vercel --yes --prod --token "$VERCEL_TOKEN"
```

### 5. Test

Send a test email to `walker@cleanwalkerrobotics.com` and check:
- Resend dashboard → Received Emails (should show the email)
- Vercel logs → Function logs for `/api/email/webhook`
- OpenClaw gateway logs for the forwarded message

## BLOCKER: Gateway URL

The OpenClaw hook runs at `POST http://localhost:3033/hooks/gmail` on the VPS
(`46.224.145.42`). This is **not publicly accessible** — Vercel serverless
functions cannot reach `localhost:3033` on the VPS.

### Options to resolve (pick one):

1. **Cloudflare Tunnel (recommended)** — Expose the gateway via `cloudflared`:
   ```bash
   # On the VPS:
   cloudflared tunnel create openclaw
   cloudflared tunnel route dns openclaw gateway.cleanwalkerrobotics.com
   # Config: route localhost:3033 → gateway.cleanwalkerrobotics.com
   ```
   Then set `OPENCLAW_GATEWAY_URL=https://gateway.cleanwalkerrobotics.com`

2. **Traefik reverse proxy** — Add a route in the existing Traefik config
   to expose the gateway on a subdomain with auth:
   ```yaml
   # In docker-compose.yml, add labels to the openclaw container:
   - "traefik.http.routers.openclaw.rule=Host(`gateway.cleanwalkerrobotics.com`)"
   - "traefik.http.services.openclaw.loadbalancer.server.port=3033"
   ```
   Then set `OPENCLAW_GATEWAY_URL=https://gateway.cleanwalkerrobotics.com`

3. **Direct IP + firewall rule** — Open port 3033 on the VPS firewall
   and point the webhook at `http://46.224.145.42:3033`. Not recommended
   (no TLS, exposes the service publicly).

4. **ngrok/bore** — Quick tunnel for testing:
   ```bash
   ngrok http 3033
   ```

**Until the gateway is publicly reachable, the webhook will receive emails
but fail to forward them to OpenClaw.** Resend will retry delivery
automatically, so emails won't be lost.

## Hook Payload Format

The webhook forwards to OpenClaw in this format:

```json
{
  "messages": [
    {
      "id": "resend-email-id",
      "from": "sender@example.com",
      "to": "walker@cleanwalkerrobotics.com",
      "subject": "Re: Partnership inquiry",
      "date": "2026-02-10T12:00:00Z",
      "body": "Plain text email content..."
    }
  ]
}
```

Authorization header: `Bearer <OPENCLAW_HOOK_TOKEN>`

## Security

- Webhook signature verification via Svix HMAC-SHA256 (when `RESEND_WEBHOOK_SECRET` is set)
- Replay attack protection: rejects events with timestamps > 5 minutes old
- Hook token auth on the gateway side
- No secrets in source code — all via environment variables
