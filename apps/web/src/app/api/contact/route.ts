// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { Resend } from "resend";
import { NextResponse } from "next/server";

export async function POST(request: Request) {
	try {
		if (!process.env.RESEND_API_KEY) {
			return NextResponse.json(
				{ error: "Email service is not configured." },
				{ status: 503 },
			);
		}

		const resend = new Resend(process.env.RESEND_API_KEY);

		const body = await request.json();
		const { name, email, organization, role, interest, message } = body;

		if (!name || !email || !interest) {
			return NextResponse.json(
				{ error: "Name, email, and interest are required." },
				{ status: 400 },
			);
		}

		const timestamp = new Date().toISOString();

		const { error } = await resend.emails.send({
			from: "CleanWalker Contact Form <contact@cleanwalkerrobotics.com>",
			to: [
				"walker@cleanwalkerrobotics.com",
				"sales@cleanwalkerrobotics.com",
			],
			replyTo: email,
			subject: `New Contact: ${name} — ${interest}`,
			html: `
				<h2>New Contact Form Submission</h2>
				<table style="border-collapse:collapse;width:100%;max-width:600px;">
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Name</td><td style="padding:8px;border-bottom:1px solid #eee;">${name}</td></tr>
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Email</td><td style="padding:8px;border-bottom:1px solid #eee;"><a href="mailto:${email}">${email}</a></td></tr>
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Organization</td><td style="padding:8px;border-bottom:1px solid #eee;">${organization || "—"}</td></tr>
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Role</td><td style="padding:8px;border-bottom:1px solid #eee;">${role || "—"}</td></tr>
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Interest</td><td style="padding:8px;border-bottom:1px solid #eee;">${interest}</td></tr>
					<tr><td style="padding:8px;font-weight:bold;border-bottom:1px solid #eee;">Message</td><td style="padding:8px;border-bottom:1px solid #eee;">${message || "—"}</td></tr>
					<tr><td style="padding:8px;font-weight:bold;">Timestamp</td><td style="padding:8px;">${timestamp}</td></tr>
				</table>
			`,
		});

		if (error) {
			console.error("Resend error:", error);
			return NextResponse.json(
				{ error: "Failed to send email." },
				{ status: 500 },
			);
		}

		return NextResponse.json({ success: true });
	} catch (err) {
		console.error("Contact API error:", err);
		return NextResponse.json(
			{ error: "Internal server error." },
			{ status: 500 },
		);
	}
}
