// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { type NextRequest, NextResponse } from "next/server";

const REPLICATE_API_URL = "https://api.replicate.com/v1/predictions";

// YOLOv8s-WorldV2: open-vocabulary YOLO that accepts custom class names
const MODEL_VERSION =
	"d28fb092d84b252971af3f98fe3138bf75d69508edb80b5237eec9c4f2f77f27";

const LITTER_CLASSES =
	"plastic bottle, can, cigarette butt, paper cup, plastic bag, " +
	"food wrapper, glass bottle, cardboard, styrofoam, trash, litter, " +
	"garbage, waste, debris, wrapper, disposable cup";

interface DetectionBox {
	x1: number;
	y1: number;
	x2: number;
	y2: number;
}

interface Detection {
	name: string;
	confidence: number;
	box: DetectionBox;
}

/**
 * POST /api/detect — Start a litter detection prediction.
 * Body: { image: string (data URI or public URL), confidence?: number }
 * Returns: { status, predictionId } or { status: "succeeded", detections, ... }
 */
export async function POST(request: NextRequest) {
	const token = process.env.REPLICATE_API_TOKEN;
	if (!token) {
		return NextResponse.json(
			{ error: "Detection API not configured" },
			{ status: 503 },
		);
	}

	let body: { image?: string; confidence?: number };
	try {
		body = await request.json();
	} catch {
		return NextResponse.json(
			{ error: "Invalid request body" },
			{ status: 400 },
		);
	}

	const { image, confidence = 0.2 } = body;
	if (!image || typeof image !== "string") {
		return NextResponse.json(
			{ error: "Missing 'image' field (data URI or URL)" },
			{ status: 400 },
		);
	}

	try {
		const resp = await fetch(REPLICATE_API_URL, {
			method: "POST",
			headers: {
				Authorization: `Bearer ${token}`,
				"Content-Type": "application/json",
				Prefer: "wait",
			},
			body: JSON.stringify({
				version: MODEL_VERSION,
				input: {
					image,
					conf: confidence,
					iou: 0.45,
					imgsz: 640,
					class_names: LITTER_CLASSES,
					return_json: true,
				},
			}),
		});

		if (!resp.ok) {
			const text = await resp.text();
			console.error("Replicate API error:", resp.status, text);
			return NextResponse.json(
				{ error: `Model API error (${resp.status})` },
				{ status: 502 },
			);
		}

		const prediction = await resp.json();

		if (prediction.status === "succeeded") {
			return NextResponse.json({
				status: "succeeded",
				...parseOutput(prediction.output),
			});
		}

		if (prediction.status === "failed") {
			return NextResponse.json(
				{
					status: "failed",
					error: prediction.error || "Prediction failed",
				},
				{ status: 502 },
			);
		}

		// Still starting/processing — return ID for polling
		return NextResponse.json({
			status: prediction.status,
			predictionId: prediction.id,
		});
	} catch (err) {
		console.error("Detection API error:", err);
		return NextResponse.json(
			{ error: "Failed to reach detection service" },
			{ status: 502 },
		);
	}
}

/**
 * GET /api/detect?id=<predictionId> — Poll prediction status.
 */
export async function GET(request: NextRequest) {
	const token = process.env.REPLICATE_API_TOKEN;
	if (!token) {
		return NextResponse.json(
			{ error: "Detection API not configured" },
			{ status: 503 },
		);
	}

	const id = request.nextUrl.searchParams.get("id");
	if (!id || !/^[\w-]+$/.test(id)) {
		return NextResponse.json(
			{ error: "Invalid prediction ID" },
			{ status: 400 },
		);
	}

	try {
		const resp = await fetch(`${REPLICATE_API_URL}/${id}`, {
			headers: { Authorization: `Bearer ${token}` },
		});

		if (!resp.ok) {
			return NextResponse.json(
				{ error: `Poll error (${resp.status})` },
				{ status: 502 },
			);
		}

		const prediction = await resp.json();

		if (prediction.status === "succeeded") {
			return NextResponse.json({
				status: "succeeded",
				...parseOutput(prediction.output),
			});
		}

		if (
			prediction.status === "failed" ||
			prediction.status === "canceled"
		) {
			return NextResponse.json({
				status: "failed",
				error: prediction.error || "Prediction failed",
			});
		}

		return NextResponse.json({ status: prediction.status });
	} catch (err) {
		console.error("Poll error:", err);
		return NextResponse.json(
			{ error: "Failed to check prediction status" },
			{ status: 502 },
		);
	}
}

// --- Output parsing (ported from ml/poc/replicate_detect.py) ---

function parseOutput(output: unknown): {
	detections: Detection[];
	annotatedImage: string | null;
	detectionCount: number;
} {
	let rawDetections: unknown[] = [];
	let annotatedImage: string | null = null;

	// Output format 1: { image: string, json_str: string }
	if (output && typeof output === "object" && !Array.isArray(output)) {
		const obj = output as Record<string, unknown>;
		if (typeof obj.image === "string") annotatedImage = obj.image;
		const jsonStr = obj.json_str;
		if (jsonStr) {
			try {
				const parsed =
					typeof jsonStr === "string" ? JSON.parse(jsonStr) : jsonStr;
				if (Array.isArray(parsed)) {
					rawDetections = parsed;
				} else if (
					parsed &&
					typeof parsed === "object" &&
					Array.isArray(
						(parsed as Record<string, unknown>).detections,
					)
				) {
					rawDetections = (parsed as Record<string, unknown>)
						.detections as unknown[];
				}
			} catch {
				// Invalid JSON — skip
			}
		}
	}
	// Output format 2: [annotated_image_url, json_string]
	else if (Array.isArray(output) && output.length >= 2) {
		if (typeof output[0] === "string") annotatedImage = output[0];
		if (typeof output[1] === "string") {
			try {
				const parsed = JSON.parse(output[1]);
				if (Array.isArray(parsed)) rawDetections = parsed;
				else if (parsed?.detections) rawDetections = parsed.detections;
			} catch {
				// Invalid JSON — skip
			}
		}
	}

	const detections: Detection[] = [];
	for (const det of rawDetections) {
		if (!det || typeof det !== "object") continue;
		const d = det as Record<string, unknown>;
		const name = String(d.name || d.class_name || d.label || "unknown");
		const confidence = Number(d.confidence || d.score || 0);
		const box = extractBox(d.box || d.bbox);
		if (box && confidence > 0) {
			detections.push({ name, confidence, box });
		}
	}

	return { detections, annotatedImage, detectionCount: detections.length };
}

function extractBox(raw: unknown): DetectionBox | null {
	if (!raw) return null;

	if (typeof raw === "object" && !Array.isArray(raw)) {
		const b = raw as Record<string, unknown>;
		if (typeof b.x1 === "number") {
			return {
				x1: b.x1,
				y1: b.y1 as number,
				x2: b.x2 as number,
				y2: b.y2 as number,
			};
		}
		if (typeof b.xmin === "number") {
			return {
				x1: b.xmin,
				y1: b.ymin as number,
				x2: b.xmax as number,
				y2: b.ymax as number,
			};
		}
	}

	if (Array.isArray(raw) && raw.length >= 4) {
		return {
			x1: Number(raw[0]),
			y1: Number(raw[1]),
			x2: Number(raw[2]),
			y2: Number(raw[3]),
		};
	}

	return null;
}
