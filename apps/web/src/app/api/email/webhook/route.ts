// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { type NextRequest, NextResponse } from "next/server";

/**
 * POST /api/email/webhook — Resend inbound email webhook receiver.
 *
 * Resend sends an `email.received` event with metadata only.
 * We fetch the full email body via the Resend API, normalize it
 * to the OpenClaw hook format, and forward it to the gateway.
 *
 * Required env vars:
 *   RESEND_API_KEY          — Resend API key (for fetching full email content)
 *   RESEND_WEBHOOK_SECRET   — Svix webhook signing secret (for verification)
 *   OPENCLAW_GATEWAY_URL    — e.g. https://openclaw.cleanwalkerrobotics.com
 *   OPENCLAW_HOOK_TOKEN     — Hook auth token
 */

interface ResendWebhookEvent {
	type: string;
	created_at: string;
	data: {
		email_id: string;
		created_at: string;
		from: string;
		to: string[];
		cc: string[];
		bcc: string[];
		subject: string;
		message_id: string;
		attachments: Array<{
			id: string;
			filename: string;
			content_type: string;
		}>;
	};
}

interface ResendEmailContent {
	html?: string;
	text?: string;
}

interface OpenClawMessage {
	id: string;
	from: string;
	to: string;
	subject: string;
	date: string;
	body: string;
}

export async function POST(request: NextRequest) {
	const resendApiKey = process.env.RESEND_API_KEY;
	const webhookSecret = process.env.RESEND_WEBHOOK_SECRET;
	const gatewayUrl = process.env.OPENCLAW_GATEWAY_URL;
	const hookToken = process.env.OPENCLAW_HOOK_TOKEN;

	if (!resendApiKey || !gatewayUrl || !hookToken) {
		console.error("Email webhook: missing required env vars");
		return NextResponse.json(
			{ error: "Webhook not configured" },
			{ status: 503 },
		);
	}

	// --- Verify Svix signature (if secret is configured) ---
	if (webhookSecret) {
		const svixId = request.headers.get("svix-id");
		const svixTimestamp = request.headers.get("svix-timestamp");
		const svixSignature = request.headers.get("svix-signature");

		if (!svixId || !svixTimestamp || !svixSignature) {
			return NextResponse.json(
				{ error: "Missing webhook signature headers" },
				{ status: 401 },
			);
		}

		// Timestamp replay protection: reject events older than 5 minutes
		const ts = Number(svixTimestamp);
		if (Math.abs(Date.now() / 1000 - ts) > 300) {
			return NextResponse.json(
				{ error: "Webhook timestamp too old" },
				{ status: 401 },
			);
		}

		const body = await request.text();
		const isValid = await verifySvixSignature(
			webhookSecret,
			svixId,
			svixTimestamp,
			body,
			svixSignature,
		);

		if (!isValid) {
			return NextResponse.json(
				{ error: "Invalid webhook signature" },
				{ status: 401 },
			);
		}

		// Parse the already-read body
		return handleEvent(
			JSON.parse(body) as ResendWebhookEvent,
			resendApiKey,
			gatewayUrl,
			hookToken,
		);
	}

	// No webhook secret — parse body directly (dev/testing mode)
	let event: ResendWebhookEvent;
	try {
		event = (await request.json()) as ResendWebhookEvent;
	} catch {
		return NextResponse.json(
			{ error: "Invalid request body" },
			{ status: 400 },
		);
	}

	return handleEvent(event, resendApiKey, gatewayUrl, hookToken);
}

async function handleEvent(
	event: ResendWebhookEvent,
	resendApiKey: string,
	gatewayUrl: string,
	hookToken: string,
): Promise<NextResponse> {
	if (event.type !== "email.received") {
		// Acknowledge non-email events without processing
		return NextResponse.json({ ok: true, skipped: event.type });
	}

	const { email_id, from, to, subject, created_at } = event.data;

	// --- Fetch full email content from Resend API ---
	let emailContent: ResendEmailContent;
	try {
		const resp = await fetch(
			`https://api.resend.com/emails/${email_id}/content`,
			{
				headers: { Authorization: `Bearer ${resendApiKey}` },
			},
		);

		if (!resp.ok) {
			console.error(
				`Resend content fetch failed: ${resp.status}`,
				await resp.text(),
			);
			return NextResponse.json(
				{ error: "Failed to fetch email content" },
				{ status: 502 },
			);
		}

		emailContent = (await resp.json()) as ResendEmailContent;
	} catch (err) {
		console.error("Resend content fetch error:", err);
		return NextResponse.json(
			{ error: "Failed to reach Resend API" },
			{ status: 502 },
		);
	}

	// Prefer plain text, fall back to HTML
	const body = emailContent.text || emailContent.html || "";

	// --- Normalize to OpenClaw hook format ---
	const message: OpenClawMessage = {
		id: email_id,
		from,
		to: to[0] || "walker@cleanwalkerrobotics.com",
		subject: subject || "(no subject)",
		date: created_at,
		body,
	};

	// --- Forward to OpenClaw gateway ---
	try {
		const hookResp = await fetch(`${gatewayUrl}/hooks/gmail`, {
			method: "POST",
			headers: {
				"Content-Type": "application/json",
				Authorization: `Bearer ${hookToken}`,
			},
			body: JSON.stringify({ messages: [message] }),
		});

		if (!hookResp.ok) {
			const text = await hookResp.text();
			console.error(`OpenClaw hook error: ${hookResp.status}`, text);
			return NextResponse.json(
				{ error: `Gateway error (${hookResp.status})` },
				{ status: 502 },
			);
		}

		console.log(
			`Email forwarded: ${from} → ${to[0]} | Subject: ${subject}`,
		);
		return NextResponse.json({ ok: true, email_id });
	} catch (err) {
		console.error("OpenClaw gateway error:", err);
		return NextResponse.json(
			{ error: "Failed to reach OpenClaw gateway" },
			{ status: 502 },
		);
	}
}

// --- Svix HMAC-SHA256 signature verification ---
// Resend uses Svix for webhook delivery. The signature is
// base64(HMAC-SHA256(secret, "${svix_id}.${timestamp}.${body}"))

async function verifySvixSignature(
	secret: string,
	msgId: string,
	timestamp: string,
	body: string,
	signatureHeader: string,
): Promise<boolean> {
	// Secret comes as "whsec_<base64>" — strip prefix
	const secretBytes = base64ToUint8Array(secret.replace("whsec_", ""));

	const signedContent = `${msgId}.${timestamp}.${body}`;
	const encoder = new TextEncoder();

	const key = await crypto.subtle.importKey(
		"raw",
		secretBytes.buffer as ArrayBuffer,
		{ name: "HMAC", hash: "SHA-256" },
		false,
		["sign"],
	);

	const signature = await crypto.subtle.sign(
		"HMAC",
		key,
		encoder.encode(signedContent).buffer as ArrayBuffer,
	);

	const expected = uint8ArrayToBase64(new Uint8Array(signature));

	// Svix sends multiple signatures separated by spaces: "v1,<sig1> v1,<sig2>"
	const signatures = signatureHeader.split(" ");
	for (const sig of signatures) {
		const [version, value] = sig.split(",", 2);
		if (version === "v1" && value === expected) {
			return true;
		}
	}

	return false;
}

function base64ToUint8Array(b64: string): Uint8Array {
	const binary = atob(b64);
	const bytes = new Uint8Array(binary.length);
	for (let i = 0; i < binary.length; i++) {
		bytes[i] = binary.charCodeAt(i);
	}
	return bytes;
}

function uint8ArrayToBase64(bytes: Uint8Array): string {
	let binary = "";
	for (let i = 0; i < bytes.length; i++) {
		binary += String.fromCharCode(bytes[i]);
	}
	return btoa(binary);
}
