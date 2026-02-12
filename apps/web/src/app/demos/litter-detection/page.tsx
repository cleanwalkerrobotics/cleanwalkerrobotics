// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useEffect, useRef, useCallback } from "react";

// ---- Types ----

interface Detection {
	label: string;
	confidence: number;
	classId: number;
	cx: number;
	cy: number;
	w: number;
	h: number;
	color: string;
	isLitter: boolean;
}

interface SampleScene {
	title: string;
	gradient: string;
	icon: string;
	detections: {
		label: string;
		confidence: number;
		box: { top: string; left: string; width: string; height: string };
		color: string;
	}[];
}

// ---- Constants ----

const DETECTION_COLORS = [
	"#22c55e",
	"#3b82f6",
	"#f59e0b",
	"#ef4444",
	"#8b5cf6",
	"#ec4899",
	"#14b8a6",
	"#f97316",
];

const LITTER_CLASS_IDS = new Set([
	24, 25, 26, 28, 29, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
	52, 53, 54, 55, 67, 73, 75, 76, 79,
]);

const SAMPLE_IMAGES = [
	{
		label: "Beach Litter",
		icon: "\u{1F3D6}\uFE0F",
		gradient: "from-cyan-800 to-blue-900",
		url: "https://images.unsplash.com/photo-1621451537084-482c73073a0f?w=640&q=80",
	},
	{
		label: "Urban Trash",
		icon: "\u{1F3D9}\uFE0F",
		gradient: "from-slate-700 to-gray-800",
		url: "https://images.unsplash.com/photo-1532996122724-e3c354a0b15b?w=640&q=80",
	},
	{
		label: "Park Debris",
		icon: "\u{1F333}",
		gradient: "from-green-800 to-emerald-900",
		url: "https://images.unsplash.com/photo-1605600659908-0ef719419d41?w=640&q=80",
	},
];

const sampleScenes: SampleScene[] = [
	{
		title: "City Park \u2014 Morning Patrol",
		gradient: "from-green-900/60 via-emerald-900/40 to-cw-dark",
		icon: "\u{1F333}",
		detections: [
			{
				label: "Plastic Bottle",
				confidence: 97,
				box: { top: "15%", left: "10%", width: "18%", height: "35%" },
				color: DETECTION_COLORS[0],
			},
			{
				label: "Cigarette Butt",
				confidence: 94,
				box: { top: "65%", left: "55%", width: "10%", height: "12%" },
				color: DETECTION_COLORS[1],
			},
			{
				label: "Candy Wrapper",
				confidence: 91,
				box: { top: "50%", left: "30%", width: "14%", height: "10%" },
				color: DETECTION_COLORS[2],
			},
			{
				label: "Paper Cup",
				confidence: 89,
				box: { top: "20%", left: "68%", width: "15%", height: "28%" },
				color: DETECTION_COLORS[3],
			},
		],
	},
	{
		title: "Beach Boardwalk \u2014 Afternoon",
		gradient: "from-cyan-900/60 via-blue-900/40 to-cw-dark",
		icon: "\u{1F3D6}\uFE0F",
		detections: [
			{
				label: "Aluminum Can",
				confidence: 98,
				box: { top: "40%", left: "20%", width: "12%", height: "25%" },
				color: DETECTION_COLORS[0],
			},
			{
				label: "Plastic Bag",
				confidence: 93,
				box: { top: "10%", left: "60%", width: "22%", height: "30%" },
				color: DETECTION_COLORS[4],
			},
			{
				label: "Straw",
				confidence: 88,
				box: { top: "70%", left: "45%", width: "8%", height: "15%" },
				color: DETECTION_COLORS[2],
			},
			{
				label: "Food Container",
				confidence: 95,
				box: { top: "55%", left: "8%", width: "20%", height: "18%" },
				color: DETECTION_COLORS[1],
			},
			{
				label: "Bottle Cap",
				confidence: 86,
				box: { top: "60%", left: "75%", width: "8%", height: "8%" },
				color: DETECTION_COLORS[5],
			},
		],
	},
	{
		title: "Parking Lot \u2014 Evening",
		gradient: "from-slate-800/80 via-gray-900/60 to-cw-dark",
		icon: "\u{1F17F}\uFE0F",
		detections: [
			{
				label: "Fast Food Bag",
				confidence: 96,
				box: { top: "35%", left: "15%", width: "20%", height: "25%" },
				color: DETECTION_COLORS[3],
			},
			{
				label: "Cigarette Butt",
				confidence: 92,
				box: { top: "75%", left: "40%", width: "8%", height: "8%" },
				color: DETECTION_COLORS[1],
			},
			{
				label: "Plastic Cup",
				confidence: 90,
				box: { top: "25%", left: "65%", width: "14%", height: "30%" },
				color: DETECTION_COLORS[0],
			},
		],
	},
	{
		title: "University Campus \u2014 Midday",
		gradient: "from-amber-900/50 via-orange-900/30 to-cw-dark",
		icon: "\u{1F393}",
		detections: [
			{
				label: "Coffee Cup",
				confidence: 97,
				box: { top: "20%", left: "25%", width: "14%", height: "30%" },
				color: DETECTION_COLORS[0],
			},
			{
				label: "Napkin",
				confidence: 85,
				box: { top: "60%", left: "50%", width: "12%", height: "10%" },
				color: DETECTION_COLORS[2],
			},
			{
				label: "Plastic Bottle",
				confidence: 96,
				box: { top: "15%", left: "70%", width: "12%", height: "35%" },
				color: DETECTION_COLORS[4],
			},
			{
				label: "Chip Bag",
				confidence: 93,
				box: { top: "45%", left: "8%", width: "16%", height: "18%" },
				color: DETECTION_COLORS[5],
			},
			{
				label: "Straw Wrapper",
				confidence: 82,
				box: { top: "70%", left: "30%", width: "10%", height: "6%" },
				color: DETECTION_COLORS[6],
			},
		],
	},
	{
		title: "Sidewalk \u2014 Rush Hour",
		gradient: "from-zinc-800/70 via-neutral-900/50 to-cw-dark",
		icon: "\u{1F6B6}",
		detections: [
			{
				label: "Receipt",
				confidence: 87,
				box: { top: "55%", left: "35%", width: "10%", height: "14%" },
				color: DETECTION_COLORS[2],
			},
			{
				label: "Plastic Bottle",
				confidence: 98,
				box: { top: "10%", left: "55%", width: "13%", height: "38%" },
				color: DETECTION_COLORS[0],
			},
			{
				label: "Cigarette Pack",
				confidence: 95,
				box: { top: "40%", left: "12%", width: "14%", height: "16%" },
				color: DETECTION_COLORS[3],
			},
			{
				label: "Gum Wrapper",
				confidence: 81,
				box: { top: "72%", left: "65%", width: "8%", height: "6%" },
				color: DETECTION_COLORS[6],
			},
		],
	},
	{
		title: "Playground \u2014 Weekend",
		gradient: "from-rose-900/40 via-pink-900/30 to-cw-dark",
		icon: "\u{1F3AA}",
		detections: [
			{
				label: "Juice Box",
				confidence: 96,
				box: { top: "30%", left: "20%", width: "14%", height: "25%" },
				color: DETECTION_COLORS[0],
			},
			{
				label: "Snack Wrapper",
				confidence: 94,
				box: { top: "55%", left: "60%", width: "16%", height: "12%" },
				color: DETECTION_COLORS[5],
			},
			{
				label: "Tissue",
				confidence: 88,
				box: { top: "65%", left: "15%", width: "10%", height: "10%" },
				color: DETECTION_COLORS[2],
			},
			{
				label: "Plastic Fork",
				confidence: 91,
				box: { top: "20%", left: "72%", width: "8%", height: "22%" },
				color: DETECTION_COLORS[4],
			},
			{
				label: "Paper Plate",
				confidence: 93,
				box: { top: "45%", left: "40%", width: "18%", height: "14%" },
				color: DETECTION_COLORS[1],
			},
			{
				label: "Candy Wrapper",
				confidence: 84,
				box: { top: "78%", left: "50%", width: "9%", height: "7%" },
				color: DETECTION_COLORS[7],
			},
		],
	},
];

// ---- Utilities ----

function drawDetections(
	ctx: CanvasRenderingContext2D,
	detections: Detection[],
	canvasWidth: number,
	canvasHeight: number,
) {
	ctx.clearRect(0, 0, canvasWidth, canvasHeight);

	for (const det of detections) {
		const x = (det.cx - det.w / 2) * canvasWidth;
		const y = (det.cy - det.h / 2) * canvasHeight;
		const w = det.w * canvasWidth;
		const h = det.h * canvasHeight;
		const color = det.color;
		const label = `${det.label} ${Math.round(det.confidence * 100)}%`;

		ctx.strokeStyle = color;
		ctx.lineWidth = 2;
		ctx.strokeRect(x, y, w, h);

		ctx.fillStyle = color + "20";
		ctx.fillRect(x, y, w, h);

		ctx.font = "bold 12px Inter, system-ui, sans-serif";
		const textWidth = ctx.measureText(label).width;
		ctx.fillStyle = color;
		const labelY = y > 22 ? y - 22 : y;
		ctx.fillRect(x, labelY, textWidth + 10, 20);
		ctx.fillStyle = "#ffffff";
		ctx.fillText(label, x + 5, labelY + 14);
	}
}

// ---- Components ----

function DetectionCard({ scene }: { scene: SampleScene }) {
	return (
		<div className="group overflow-hidden rounded-2xl border border-white/10 bg-white/5 transition-all hover:border-cw-green/30 hover:bg-white/[0.07]">
			<div
				className={`relative aspect-video bg-gradient-to-br ${scene.gradient} overflow-hidden`}
			>
				<div className="absolute left-3 top-3 z-10 rounded-full bg-black/50 px-3 py-1 text-xs text-gray-300 backdrop-blur-sm">
					<span className="mr-1">{scene.icon}</span>
					{scene.title}
				</div>
				<div className="absolute right-3 top-3 z-10 rounded-full bg-cw-green/20 px-3 py-1 text-xs font-medium text-cw-green backdrop-blur-sm">
					{scene.detections.length} detected
				</div>
				{scene.detections.map((det) => (
					<div
						key={det.label}
						className="absolute transition-opacity"
						style={{
							top: det.box.top,
							left: det.box.left,
							width: det.box.width,
							height: det.box.height,
						}}
					>
						<div
							className="h-full w-full rounded-sm"
							style={{
								border: `2px solid ${det.color}`,
								backgroundColor: `${det.color}15`,
							}}
						/>
						<span
							className="absolute -top-5 left-0 whitespace-nowrap rounded px-1.5 py-0.5 text-[10px] font-medium text-white"
							style={{ backgroundColor: det.color }}
						>
							{det.label}
						</span>
					</div>
				))}
				<div className="pointer-events-none absolute inset-0 opacity-0 transition-opacity group-hover:opacity-100">
					<div
						className="absolute left-0 h-px w-full bg-cw-green/50"
						style={{
							animation: "scanline 2s ease-in-out infinite",
							top: "0%",
						}}
					/>
				</div>
			</div>
			<div className="p-4">
				<div className="space-y-2">
					{scene.detections.map((det) => (
						<div
							key={det.label}
							className="flex items-center justify-between"
						>
							<div className="flex items-center gap-2">
								<span
									className="inline-block h-2.5 w-2.5 rounded-full"
									style={{ backgroundColor: det.color }}
								/>
								<span className="text-sm text-gray-300">
									{det.label}
								</span>
							</div>
							<span
								className="font-mono text-sm font-medium"
								style={{
									color:
										det.confidence >= 95
											? "#22c55e"
											: det.confidence >= 90
												? "#f59e0b"
												: "#94a3b8",
								}}
							>
								{det.confidence}%
							</span>
						</div>
					))}
				</div>
			</div>
		</div>
	);
}

// ---- Main Page ----

export default function LitterDetectionDemoPage() {
	// Model state
	const [modelStatus, setModelStatus] = useState<
		"idle" | "loading" | "ready" | "error"
	>("idle");
	const [modelProgress, setModelProgress] = useState("");
	const [backend, setBackend] = useState<"webgpu" | "wasm" | null>(null);

	// Mode & detection state
	const [mode, setMode] = useState<"webcam" | "upload">("webcam");
	const [isRunning, setIsRunning] = useState(false);
	const [threshold, setThreshold] = useState(50);
	const [fps, setFps] = useState(0);
	const [liveDetections, setLiveDetections] = useState<Detection[]>([]);
	const [videoDims, setVideoDims] = useState({ w: 640, h: 480 });
	const [cameraError, setCameraError] = useState("");

	// Upload state
	const [uploadState, setUploadState] = useState<
		"idle" | "processing" | "done"
	>("idle");
	const [uploadImage, setUploadImage] = useState<string | null>(null);
	const [uploadDetections, setUploadDetections] = useState<Detection[]>([]);
	const [uploadFileName, setUploadFileName] = useState("");
	const [uploadDims, setUploadDims] = useState({ w: 640, h: 480 });
	const [urlInput, setUrlInput] = useState("");

	// Refs
	const modelRef = useRef<ReturnType<typeof Object> | null>(null);
	const processorRef = useRef<ReturnType<typeof Object> | null>(null);
	const transformersRef = useRef<ReturnType<typeof Object> | null>(null);
	const videoRef = useRef<HTMLVideoElement>(null);
	const canvasRef = useRef<HTMLCanvasElement>(null);
	const offscreenRef = useRef<HTMLCanvasElement | null>(null);
	const isProcessingRef = useRef(false);
	const isRunningRef = useRef(false);
	const thresholdRef = useRef(0.5);
	const animationIdRef = useRef<number | null>(null);

	// Sync threshold ref
	useEffect(() => {
		thresholdRef.current = threshold / 100;
	}, [threshold]);

	// Load model on mount
	useEffect(() => {
		let cancelled = false;

		async function loadModel() {
			setModelStatus("loading");
			setModelProgress("Loading Transformers.js...");

			try {
				// biome-ignore lint/suspicious/noExplicitAny: dynamic import
				const transformers: any = await import(
					"@huggingface/transformers"
				);
				if (cancelled) return;
				transformersRef.current = transformers;

				const modelId = "onnx-community/yolo26n-ONNX";

				// biome-ignore lint/suspicious/noExplicitAny: progress callback
				const progressCallback = (info: any) => {
					if (
						info.status === "progress" &&
						info.file?.endsWith(".onnx")
					) {
						const pct = Math.round(
							(info.loaded / info.total) * 100,
						);
						setModelProgress(`Downloading model... ${pct}%`);
					} else if (info.status === "download") {
						setModelProgress("Downloading model files...");
					}
				};

				// Try WebGPU first, fallback to WASM
				// biome-ignore lint/suspicious/noExplicitAny: model type
				let model: any;
				let usedBackend: "webgpu" | "wasm" = "webgpu";

				// Load model via WebGPU with WASM fallback
				setModelProgress("Loading AI model...");
				try {
					model = await transformers.AutoModel.from_pretrained(
						modelId,
						{
							device: "webgpu",
							dtype: "fp16",
							progress_callback: progressCallback,
						},
					);
				} catch {
					setModelProgress(
						"Optimizing for your device...",
					);
					usedBackend = "wasm";
					model = await transformers.AutoModel.from_pretrained(
						modelId,
						{
							device: "wasm",
							progress_callback: progressCallback,
						},
					);
				}

				if (cancelled) {
					model.dispose?.();
					return;
				}

				setModelProgress("Loading processor...");
				const processor =
					await transformers.AutoProcessor.from_pretrained(modelId);

				if (cancelled) {
					model.dispose?.();
					return;
				}

				modelRef.current = model;
				processorRef.current = processor;
				setBackend(usedBackend);
				setModelStatus("ready");
				setModelProgress("");
			} catch (error) {
				if (!cancelled) {
					console.error("Model loading failed:", error);
					setModelStatus("error");
					setModelProgress(
						`Failed: ${(error as Error).message}`,
					);
				}
			}
		}

		loadModel();
		return () => {
			cancelled = true;
		};
	}, []);

	// Run detection on an image source, returns detections with low threshold
	const runDetection = useCallback(
		async (
			imageSource:
				| HTMLVideoElement
				| HTMLImageElement
				| HTMLCanvasElement,
			width: number,
			height: number,
			minThreshold = 0.05,
		): Promise<Detection[]> => {
			// biome-ignore lint/suspicious/noExplicitAny: dynamic model refs
			const model: any = modelRef.current;
			// biome-ignore lint/suspicious/noExplicitAny: dynamic processor refs
			const processor: any = processorRef.current;
			// biome-ignore lint/suspicious/noExplicitAny: dynamic import
			const transformers: any = transformersRef.current;
			if (!model || !processor || !transformers) return [];

			if (!offscreenRef.current) {
				offscreenRef.current = document.createElement("canvas");
			}
			const offscreen = offscreenRef.current;
			offscreen.width = width;
			offscreen.height = height;
			const offCtx = offscreen.getContext("2d")!;
			offCtx.drawImage(imageSource, 0, 0, width, height);

			const image = transformers.RawImage.fromCanvas(offscreen);
			const inputs = await processor(image);
			const output = await model(inputs);

			const scores = output.logits.sigmoid().data;
			const boxes = output.pred_boxes.data;
			const id2label = model.config.id2label;

			const dets: Detection[] = [];

			for (let i = 0; i < 300; i++) {
				let maxScore = 0;
				let maxClass = 0;
				for (let j = 0; j < 80; j++) {
					const score = scores[i * 80 + j];
					if (score > maxScore) {
						maxScore = score;
						maxClass = j;
					}
				}
				if (maxScore >= minThreshold) {
					dets.push({
						label:
							id2label[maxClass] || `Class ${maxClass}`,
						confidence: maxScore,
						classId: maxClass,
						cx: boxes[i * 4],
						cy: boxes[i * 4 + 1],
						w: boxes[i * 4 + 2],
						h: boxes[i * 4 + 3],
						color: DETECTION_COLORS[
							maxClass % DETECTION_COLORS.length
						],
						isLitter: LITTER_CLASS_IDS.has(maxClass),
					});
				}
			}

			return dets;
		},
		[],
	);

	// Webcam detection loop
	const startDetectionLoop = useCallback(() => {
		const video = videoRef.current;
		const canvas = canvasRef.current;
		if (!video || !canvas) return;

		const ctx = canvas.getContext("2d");
		if (!ctx) return;

		function loop() {
			if (!isRunningRef.current) return;

			if (
				modelRef.current &&
				processorRef.current &&
				!isProcessingRef.current &&
				video!.readyState >= 2
			) {
				isProcessingRef.current = true;
				const startTime = performance.now();
				const currentThreshold = thresholdRef.current;

				runDetection(
					video!,
					video!.videoWidth,
					video!.videoHeight,
					currentThreshold,
				)
					.then((dets) => {
						if (isRunningRef.current) {
							setLiveDetections(dets);
							drawDetections(
								ctx!,
								dets,
								canvas!.width,
								canvas!.height,
							);
							const elapsed =
								performance.now() - startTime;
							setFps(Math.round(1000 / elapsed));
						}
					})
					.catch(console.error)
					.finally(() => {
						isProcessingRef.current = false;
					});
			}

			if (isRunningRef.current) {
				animationIdRef.current = requestAnimationFrame(loop);
			}
		}

		loop();
	}, [runDetection]);

	// Start webcam
	const startWebcam = useCallback(async () => {
		const video = videoRef.current;
		const canvas = canvasRef.current;
		if (!video || !canvas) return;

		setCameraError("");

		try {
			const stream = await navigator.mediaDevices.getUserMedia({
				video: {
					facingMode: "environment",
					width: { ideal: 640 },
					height: { ideal: 480 },
				},
				audio: false,
			});

			video.srcObject = stream;
			video.onloadedmetadata = () => {
				canvas.width = video.videoWidth;
				canvas.height = video.videoHeight;
				setVideoDims({
					w: video.videoWidth,
					h: video.videoHeight,
				});

				isRunningRef.current = true;
				setIsRunning(true);
				startDetectionLoop();
			};
		} catch (error) {
			console.error("Camera error:", error);
			setCameraError(
				"Camera access denied. Please allow camera permissions and try again.",
			);
		}
	}, [startDetectionLoop]);

	// Stop webcam
	const stopWebcam = useCallback(() => {
		if (animationIdRef.current) {
			cancelAnimationFrame(animationIdRef.current);
			animationIdRef.current = null;
		}

		const video = videoRef.current;
		if (video?.srcObject) {
			(video.srcObject as MediaStream)
				.getTracks()
				.forEach((t) => t.stop());
			video.srcObject = null;
		}

		isRunningRef.current = false;
		isProcessingRef.current = false;
		setIsRunning(false);
		setFps(0);
		setLiveDetections([]);

		const canvas = canvasRef.current;
		if (canvas) {
			const ctx = canvas.getContext("2d");
			ctx?.clearRect(0, 0, canvas.width, canvas.height);
		}
	}, []);

	// Handle image file upload
	const handleImageFile = useCallback(
		async (file: File) => {
			if (modelStatus !== "ready") return;

			setUploadState("processing");
			setUploadFileName(file.name);
			setUploadDetections([]);

			const reader = new FileReader();
			reader.onload = async () => {
				const dataUri = reader.result as string;
				setUploadImage(dataUri);

				const img = new Image();
				img.onload = async () => {
					setUploadDims({
						w: img.naturalWidth,
						h: img.naturalHeight,
					});
					try {
						const dets = await runDetection(
							img,
							img.naturalWidth,
							img.naturalHeight,
						);
						setUploadDetections(dets);
					} catch (err) {
						console.error("Detection failed:", err);
					}
					setUploadState("done");
				};
				img.src = dataUri;
			};
			reader.readAsDataURL(file);
		},
		[modelStatus, runDetection],
	);

	// Handle image URL
	const handleImageUrl = useCallback(
		async (url: string) => {
			const trimmed = url.trim();
			if (!trimmed || modelStatus !== "ready") return;

			setUploadState("processing");
			setUploadFileName(
				trimmed.split("/").pop()?.split("?")[0] || "Image URL",
			);
			setUploadImage(trimmed);
			setUploadDetections([]);

			const img = new Image();
			img.crossOrigin = "anonymous";
			img.onload = async () => {
				setUploadDims({
					w: img.naturalWidth,
					h: img.naturalHeight,
				});
				try {
					const dets = await runDetection(
						img,
						img.naturalWidth,
						img.naturalHeight,
					);
					setUploadDetections(dets);
				} catch (err) {
					console.error("Detection failed:", err);
				}
				setUploadState("done");
			};
			img.onerror = () => {
				setUploadState("idle");
				setUploadImage(null);
			};
			img.src = trimmed;
		},
		[modelStatus, runDetection],
	);

	// Reset upload
	const resetUpload = useCallback(() => {
		setUploadState("idle");
		setUploadImage(null);
		setUploadDetections([]);
		setUploadFileName("");
		setUrlInput("");
	}, []);

	// Switch mode
	const switchMode = useCallback(
		(newMode: "webcam" | "upload") => {
			if (newMode === mode) return;
			if (mode === "webcam") stopWebcam();
			if (mode === "upload") resetUpload();
			setMode(newMode);
		},
		[mode, stopWebcam, resetUpload],
	);

	// Cleanup on unmount
	useEffect(() => {
		return () => {
			if (animationIdRef.current) {
				cancelAnimationFrame(animationIdRef.current);
			}
			const video = videoRef.current;
			if (video?.srcObject) {
				(video.srcObject as MediaStream)
					.getTracks()
					.forEach((t) => t.stop());
			}
		};
	}, []);

	// Filtered upload detections based on threshold
	const filteredUploadDetections = uploadDetections.filter(
		(d) => d.confidence >= threshold / 100,
	);

	// Active detections for the sidebar
	const activeDetections =
		mode === "webcam" ? liveDetections : filteredUploadDetections;

	return (
		<div className="min-h-screen bg-cw-dark">
			<style>{`
				@keyframes scanline {
					0%, 100% { top: 0%; opacity: 0; }
					10% { opacity: 1; }
					90% { opacity: 1; }
					50% { top: 100%; }
				}
				@keyframes pulse-dot {
					0%, 100% { opacity: 1; }
					50% { opacity: 0.5; }
				}
				@keyframes fadeIn {
					from { opacity: 0; transform: scale(0.95); }
					to { opacity: 1; transform: scale(1); }
				}
				.animate-fadeIn {
					animation: fadeIn 0.4s ease-out forwards;
				}
			`}</style>

			{/* Hero */}
			<section className="px-6 py-24 text-center">
				<div className="mx-auto max-w-4xl">
					<a
						href="/demos"
						className="mb-6 inline-flex items-center gap-2 text-sm text-gray-400 transition-colors hover:text-cw-green"
					>
						<svg
							className="h-4 w-4"
							fill="none"
							viewBox="0 0 24 24"
							stroke="currentColor"
						>
							<path
								strokeLinecap="round"
								strokeLinejoin="round"
								strokeWidth={2}
								d="M15 19l-7-7 7-7"
							/>
						</svg>
						Back to Demos
					</a>
					<span className="mb-4 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1 text-sm text-cw-green">
						Live Demo
					</span>
					<h1 className="mt-4 text-4xl font-bold tracking-tight text-white md:text-5xl">
						Real-Time AI Litter Detection
					</h1>
					<p className="mt-6 text-lg leading-relaxed text-gray-400">
						Our perception system identifies and classifies
						objects in real-time, directly in your browser.
						No server needed &mdash; your camera feed never
						leaves your device.
					</p>
				</div>
			</section>

			{/* Stats */}
			<section className="px-6 pb-16">
				<div className="mx-auto grid max-w-5xl grid-cols-2 gap-4 md:grid-cols-4">
					{[
						{
							value: "Real-time",
							label: "On-device inference",
						},
						{
							value: "Private",
							label: "No data leaves your device",
						},
						{
							value: "80+",
							label: "Object categories",
						},
						{
							value: "GPU-accelerated",
							label: "Hardware-optimized",
						},
					].map((stat) => (
						<div
							key={stat.label}
							className="rounded-xl border border-white/10 bg-white/5 px-4 py-6 text-center"
						>
							<p className="text-2xl font-bold text-cw-green md:text-3xl">
								{stat.value}
							</p>
							<p className="mt-1 text-xs text-gray-400 md:text-sm">
								{stat.label}
							</p>
						</div>
					))}
				</div>
			</section>

			{/* Detection Section */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="mb-10 text-center">
						<h2 className="text-2xl font-bold text-white md:text-3xl">
							Try It Yourself
						</h2>
						<p className="mt-3 text-gray-400">
							Point your camera at objects or upload an
							image to see real-time AI detection
						</p>
					</div>

					{/* Mode Tabs */}
					<div className="mb-6 flex justify-center gap-2">
						<button
							onClick={() => switchMode("webcam")}
							className={`rounded-lg px-6 py-2.5 text-sm font-medium transition-all ${
								mode === "webcam"
									? "bg-cw-green/20 text-cw-green border border-cw-green/30"
									: "bg-white/5 text-gray-400 border border-white/10 hover:bg-white/10 hover:text-white"
							}`}
						>
							<span className="mr-2">
								{"\u{1F4F9}"}
							</span>
							Webcam
						</button>
						<button
							onClick={() => switchMode("upload")}
							className={`rounded-lg px-6 py-2.5 text-sm font-medium transition-all ${
								mode === "upload"
									? "bg-cw-green/20 text-cw-green border border-cw-green/30"
									: "bg-white/5 text-gray-400 border border-white/10 hover:bg-white/10 hover:text-white"
							}`}
						>
							<span className="mr-2">
								{"\u{1F4C1}"}
							</span>
							Upload Image
						</button>
					</div>

					{/* Model Loading Banner */}
					{modelStatus === "loading" && (
						<div className="mb-6 flex items-center justify-center gap-3 rounded-xl border border-amber-500/20 bg-amber-500/5 px-6 py-4">
							<div className="h-5 w-5 animate-spin rounded-full border-2 border-amber-500/30 border-t-amber-500" />
							<p className="text-sm text-amber-400">
								{modelProgress}
							</p>
						</div>
					)}

					{modelStatus === "error" && (
						<div className="mb-6 rounded-xl border border-red-500/20 bg-red-500/5 px-6 py-4 text-center">
							<p className="text-sm text-red-400">
								{modelProgress}
							</p>
							<button
								onClick={() => window.location.reload()}
								className="mt-2 text-sm text-red-300 underline hover:text-red-200"
							>
								Reload page to retry
							</button>
						</div>
					)}

					{/* Main Content Grid */}
					<div className="grid grid-cols-1 gap-6 lg:grid-cols-[1fr_320px]">
						{/* Left: Video / Image */}
						<div>
							{mode === "webcam" ? (
								<div
									className="relative overflow-hidden rounded-2xl border border-white/10 bg-black/40"
									style={{
										aspectRatio: `${videoDims.w} / ${videoDims.h}`,
									}}
								>
									<video
										ref={videoRef}
										autoPlay
										playsInline
										muted
										className="absolute inset-0 h-full w-full"
									/>
									<canvas
										ref={canvasRef}
										className="absolute inset-0 h-full w-full"
										style={{
											pointerEvents:
												"none",
											zIndex: 10,
										}}
									/>

									{/* Status Badge */}
									<div
										className="absolute left-4 top-4 z-20 flex items-center gap-2 rounded-full border border-white/10 bg-black/60 px-3 py-1.5 text-xs font-medium backdrop-blur-sm"
									>
										<span
											className={`inline-block h-2 w-2 rounded-full ${
												isRunning
													? "bg-green-500"
													: modelStatus ===
														  "ready"
														? "bg-amber-500"
														: "bg-gray-500"
											}`}
											style={
												isRunning
													? {
															animation: "pulse-dot 1.5s infinite",
														}
													: undefined
											}
										/>
										<span className="text-gray-300">
											{isRunning
												? "Running"
												: modelStatus ===
													  "ready"
													? "Ready"
													: modelStatus ===
														  "loading"
														? "Loading..."
														: "Error"}
										</span>
									</div>

									{/* FPS Badge */}
									{isRunning && (
										<div className="absolute right-4 top-4 z-20 flex items-center gap-1.5 rounded-full border border-white/10 bg-black/60 px-3 py-1.5 font-mono text-xs backdrop-blur-sm">
											<span className="font-semibold text-cw-green">
												{fps}
											</span>
											<span className="text-gray-400">
												FPS
											</span>
										</div>
									)}

									{/* Start/Stop Overlay */}
									{!isRunning && (
										<div className="absolute inset-0 z-10 flex flex-col items-center justify-center bg-black/60">
											{cameraError ? (
												<>
													<div className="mb-4 flex h-16 w-16 items-center justify-center rounded-full bg-red-500/20">
														<svg
															className="h-8 w-8 text-red-400"
															fill="none"
															viewBox="0 0 24 24"
															stroke="currentColor"
														>
															<path
																strokeLinecap="round"
																strokeLinejoin="round"
																strokeWidth={2}
																d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L4.082 16.5c-.77.833.192 2.5 1.732 2.5z"
															/>
														</svg>
													</div>
													<p className="mb-2 text-sm text-red-300">
														{cameraError}
													</p>
													<button
														onClick={startWebcam}
														disabled={modelStatus !== "ready"}
														className="rounded-lg bg-white/10 px-4 py-2 text-sm text-gray-300 transition-colors hover:bg-white/20 disabled:opacity-50"
													>
														Try Again
													</button>
												</>
											) : (
												<>
													<div className="mb-4 flex h-16 w-16 items-center justify-center rounded-full bg-cw-green/20">
														<svg
															className="h-8 w-8 text-cw-green"
															fill="none"
															viewBox="0 0 24 24"
															stroke="currentColor"
														>
															<path
																strokeLinecap="round"
																strokeLinejoin="round"
																strokeWidth={2}
																d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z"
															/>
														</svg>
													</div>
													<p className="mb-4 text-sm text-gray-400">
														{modelStatus === "ready"
															? "Camera ready for real-time detection"
															: "Waiting for model to load..."}
													</p>
													<button
														onClick={startWebcam}
														disabled={modelStatus !== "ready"}
														className="rounded-lg bg-cw-green px-6 py-3 text-sm font-semibold text-white transition-all hover:bg-cw-green-dark disabled:opacity-50 disabled:cursor-not-allowed"
													>
														Start Camera
													</button>
												</>
											)}
										</div>
									)}
								</div>
							) : (
								/* Upload Mode */
								<div>
									{uploadState === "idle" && (
										<div className="space-y-6">
											<label
												className="flex cursor-pointer flex-col items-center justify-center rounded-2xl border-2 border-dashed border-white/20 bg-white/5 px-8 py-14 transition-all hover:border-cw-green/50 hover:bg-white/[0.07]"
												onDragOver={(
													e,
												) =>
													e.preventDefault()
												}
												onDrop={(e) => {
													e.preventDefault();
													const file =
														e.dataTransfer
															.files[0];
													if (file)
														handleImageFile(
															file,
														);
												}}
											>
												<input
													type="file"
													accept="image/*"
													className="hidden"
													onChange={(
														e,
													) => {
														const file =
															e.target
																.files?.[0];
														if (file)
															handleImageFile(
																file,
															);
													}}
												/>
												<div className="mb-4 flex h-16 w-16 items-center justify-center rounded-full bg-cw-green/20">
													<svg
														className="h-8 w-8 text-cw-green"
														fill="none"
														viewBox="0 0 24 24"
														stroke="currentColor"
													>
														<path
															strokeLinecap="round"
															strokeLinejoin="round"
															strokeWidth={2}
															d="M4 16l4.586-4.586a2 2 0 012.828 0L16 16m-2-2l1.586-1.586a2 2 0 012.828 0L20 14m-6-6h.01M6 20h12a2 2 0 002-2V6a2 2 0 00-2-2H6a2 2 0 00-2 2v12a2 2 0 002 2z"
														/>
													</svg>
												</div>
												<p className="text-lg font-medium text-white">
													{modelStatus ===
													"ready"
														? "Drop an image here or click to upload"
														: "Model loading... please wait"}
												</p>
												<p className="mt-2 text-sm text-gray-400">
													JPG, PNG,
													or WebP
													&mdash;
													detection
													runs
													locally in
													your
													browser
												</p>
											</label>

											<div className="flex items-center gap-4">
												<div className="h-px flex-1 bg-white/10" />
												<span className="text-xs uppercase tracking-wider text-gray-500">
													or
												</span>
												<div className="h-px flex-1 bg-white/10" />
											</div>

											<div className="flex gap-3">
												<input
													type="url"
													placeholder="Paste an image URL..."
													value={
														urlInput
													}
													onChange={(
														e,
													) =>
														setUrlInput(
															e
																.target
																.value,
														)
													}
													onKeyDown={(
														e,
													) => {
														if (
															e.key ===
															"Enter"
														)
															handleImageUrl(
																urlInput,
															);
													}}
													className="flex-1 rounded-xl border border-white/15 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green/50 focus:bg-white/[0.07]"
												/>
												<button
													onClick={() =>
														handleImageUrl(
															urlInput,
														)
													}
													disabled={
														!urlInput.trim() ||
														modelStatus !==
															"ready"
													}
													className="rounded-xl bg-cw-green/20 px-6 py-3 text-sm font-medium text-cw-green transition-all hover:bg-cw-green/30 disabled:cursor-not-allowed disabled:opacity-40"
												>
													Analyze
												</button>
											</div>

											<div>
												<p className="mb-3 text-center text-sm text-gray-500">
													Or try a
													sample
													image
												</p>
												<div className="grid grid-cols-3 gap-3">
													{SAMPLE_IMAGES.map(
														(
															sample,
														) => (
															<button
																key={
																	sample.label
																}
																onClick={() =>
																	handleImageUrl(
																		sample.url,
																	)
																}
																disabled={
																	modelStatus !==
																	"ready"
																}
																className="group flex flex-col items-center gap-2 rounded-xl border border-white/10 bg-white/5 px-3 py-4 transition-all hover:border-cw-green/30 hover:bg-white/[0.07] disabled:opacity-50"
															>
																<div
																	className={`flex h-10 w-10 items-center justify-center rounded-lg bg-gradient-to-br ${sample.gradient} text-xl`}
																>
																	{
																		sample.icon
																	}
																</div>
																<span className="text-xs text-gray-400 group-hover:text-gray-300">
																	{
																		sample.label
																	}
																</span>
															</button>
														),
													)}
												</div>
											</div>
										</div>
									)}

									{uploadState ===
										"processing" && (
										<div className="overflow-hidden rounded-2xl border border-white/10 bg-white/5">
											<div className="relative aspect-video overflow-hidden">
												{uploadImage ? (
													<img
														src={
															uploadImage
														}
														alt="Processing..."
														className="h-full w-full object-cover opacity-40 blur-sm"
													/>
												) : (
													<div className="h-full w-full bg-gradient-to-br from-emerald-900/40 via-teal-900/30 to-cw-dark" />
												)}
												<div className="absolute inset-0 flex flex-col items-center justify-center">
													<div className="relative mb-6 h-16 w-16">
														<div className="absolute inset-0 animate-spin rounded-full border-4 border-cw-green/20 border-t-cw-green" />
													</div>
													<p className="text-lg font-medium text-white">
														Analyzing
														image...
													</p>
													<p className="mt-2 text-xs text-gray-500">
														Processing
														locally
														on your
														device
													</p>
												</div>
											</div>
										</div>
									)}

									{uploadState === "done" &&
										uploadImage && (
											<div className="overflow-hidden rounded-2xl border border-cw-green/30 bg-white/5">
												<div className="flex items-center justify-between border-b border-white/10 px-6 py-4">
													<div>
														<p className="text-sm text-gray-400">
															{
																uploadFileName
															}
														</p>
														<p className="text-lg font-semibold text-white">
															{
																filteredUploadDetections.length
															}{" "}
															item
															{filteredUploadDetections.length !==
															1
																? "s"
																: ""}{" "}
															detected
														</p>
													</div>
													<button
														onClick={
															resetUpload
														}
														className="rounded-lg bg-white/10 px-4 py-2 text-sm text-gray-300 transition-colors hover:bg-white/20"
													>
														Try
														Another
													</button>
												</div>

												<div className="bg-black/30">
													<div
														className="relative mx-auto w-full overflow-hidden"
														style={{
															aspectRatio: `${uploadDims.w} / ${uploadDims.h}`,
														}}
													>
														<img
															src={
																uploadImage
															}
															alt={
																uploadFileName
															}
															className="h-full w-full"
														/>
														{filteredUploadDetections.map(
															(
																det,
																i,
															) => (
																<div
																	key={`det-${det.label}-${i}`}
																	className="absolute animate-fadeIn"
																	style={{
																		left: `${(det.cx - det.w / 2) * 100}%`,
																		top: `${(det.cy - det.h / 2) * 100}%`,
																		width: `${det.w * 100}%`,
																		height: `${det.h * 100}%`,
																		animationDelay: `${i * 80}ms`,
																	}}
																>
																	<div
																		className="h-full w-full rounded-sm"
																		style={{
																			border: `2px solid ${det.color}`,
																			backgroundColor: `${det.color}15`,
																		}}
																	/>
																	<span
																		className="absolute -top-5 left-0 whitespace-nowrap rounded px-1.5 py-0.5 text-[10px] font-medium text-white shadow-lg"
																		style={{
																			backgroundColor:
																				det.color,
																		}}
																	>
																		{
																			det.label
																		}{" "}
																		{Math.round(
																			det.confidence *
																				100,
																		)}

																		%
																	</span>
																</div>
															),
														)}
													</div>
												</div>

												<div className="p-6">
													{filteredUploadDetections.length >
													0 ? (
														<div className="space-y-3">
															{filteredUploadDetections.map(
																(
																	det,
																	i,
																) => (
																	<div
																		key={`row-${det.label}-${i}`}
																		className="flex items-center justify-between rounded-lg bg-white/5 px-4 py-3"
																	>
																		<div className="flex items-center gap-3">
																			<span
																				className="inline-block h-3 w-3 rounded-full"
																				style={{
																					backgroundColor:
																						det.color,
																				}}
																			/>
																			<span className="text-sm font-medium text-white">
																				{
																					det.label
																				}
																			</span>
																			{det.isLitter && (
																				<span className="rounded-full border border-cw-green/30 bg-cw-green/10 px-2 py-0.5 text-[10px] font-medium text-cw-green">
																					Litter
																				</span>
																			)}
																		</div>
																		<div className="flex items-center gap-4">
																			<div className="h-1.5 w-24 overflow-hidden rounded-full bg-white/10">
																				<div
																					className="h-full rounded-full"
																					style={{
																						width: `${Math.round(det.confidence * 100)}%`,
																						backgroundColor:
																							det.color,
																					}}
																				/>
																			</div>
																			<span className="w-10 text-right font-mono text-sm font-medium text-cw-green">
																				{Math.round(
																					det.confidence *
																						100,
																				)}

																				%
																			</span>
																		</div>
																	</div>
																),
															)}
														</div>
													) : (
														<div className="rounded-lg bg-white/5 px-6 py-8 text-center">
															<p className="text-gray-400">
																No
																objects
																detected
																above
																the
																confidence
																threshold.
															</p>
															<p className="mt-2 text-sm text-gray-500">
																Try
																lowering
																the
																threshold
																or
																uploading
																a
																different
																image.
															</p>
														</div>
													)}
												</div>
											</div>
										)}
								</div>
							)}
						</div>

						{/* Right: Controls Sidebar */}
						<div className="space-y-4">
							{/* Backend Badge */}
							<div className="rounded-xl border border-white/10 bg-white/5 p-4">
								<p className="mb-2 text-xs font-medium uppercase tracking-wider text-gray-500">
									AI Model Status
								</p>
								<div className="flex items-center gap-2">
									<span
										className={`inline-block h-2.5 w-2.5 rounded-full ${
											modelStatus === "ready"
												? "bg-green-500"
												: modelStatus ===
													  "loading"
													? "bg-amber-500"
													: modelStatus ===
														  "error"
														? "bg-red-500"
														: "bg-gray-500"
										}`}
									/>
									<span className="text-sm font-medium text-white">
										{modelStatus === "ready"
											? "Ready — GPU Accelerated"
											: modelStatus ===
												  "loading"
												? "Loading model..."
												: modelStatus ===
													  "error"
													? "Load failed"
													: "Initializing..."}
									</span>
								</div>
								{modelStatus === "ready" && (
									<p className="mt-1 text-xs text-gray-500">
										Object detection model &middot;
										Running locally in your browser
									</p>
								)}
							</div>

							{/* FPS Counter (webcam mode) */}
							{mode === "webcam" && isRunning && (
								<div className="rounded-xl border border-white/10 bg-white/5 p-4">
									<p className="mb-1 text-xs font-medium uppercase tracking-wider text-gray-500">
										Performance
									</p>
									<div className="flex items-baseline gap-1">
										<span className="font-mono text-3xl font-bold text-cw-green">
											{fps}
										</span>
										<span className="text-sm text-gray-400">
											FPS
										</span>
									</div>
								</div>
							)}

							{/* Confidence Threshold */}
							<div className="rounded-xl border border-white/10 bg-white/5 p-4">
								<div className="mb-3 flex items-center justify-between">
									<p className="text-xs font-medium uppercase tracking-wider text-gray-500">
										Confidence Threshold
									</p>
									<span className="rounded bg-white/10 px-2 py-0.5 font-mono text-xs text-cw-green">
										{threshold}%
									</span>
								</div>
								<input
									type="range"
									min="0"
									max="100"
									value={threshold}
									onChange={(e) =>
										setThreshold(
											Number(
												e.target.value,
											),
										)
									}
									className="w-full accent-cw-green"
								/>
								<div className="mt-1 flex justify-between text-[10px] text-gray-600">
									<span>0%</span>
									<span>50%</span>
									<span>100%</span>
								</div>
							</div>

							{/* Webcam Controls */}
							{mode === "webcam" && (
								<button
									onClick={
										isRunning
											? stopWebcam
											: startWebcam
									}
									disabled={
										modelStatus !== "ready"
									}
									className={`w-full rounded-xl px-6 py-3 text-sm font-semibold transition-all disabled:cursor-not-allowed disabled:opacity-50 ${
										isRunning
											? "bg-red-500/20 text-red-400 border border-red-500/30 hover:bg-red-500/30"
											: "bg-cw-green text-white hover:bg-cw-green-dark"
									}`}
								>
									{isRunning
										? "Stop Camera"
										: "Start Camera"}
								</button>
							)}

							{/* Detection List */}
							<div className="rounded-xl border border-white/10 bg-white/5 p-4">
								<p className="mb-3 text-xs font-medium uppercase tracking-wider text-gray-500">
									Detections (
									{activeDetections.length})
								</p>
								{activeDetections.length > 0 ? (
									<div className="max-h-64 space-y-2 overflow-y-auto">
										{activeDetections.map(
											(det, i) => (
												<div
													key={`side-${det.label}-${i}`}
													className="flex items-center justify-between rounded-lg bg-white/5 px-3 py-2"
												>
													<div className="flex items-center gap-2">
														<span
															className="inline-block h-2.5 w-2.5 rounded-full"
															style={{
																backgroundColor:
																	det.color,
															}}
														/>
														<span className="text-xs text-gray-300">
															{
																det.label
															}
														</span>
													</div>
													<span className="font-mono text-xs font-medium text-cw-green">
														{Math.round(
															det.confidence *
																100,
														)}
														%
													</span>
												</div>
											),
										)}
									</div>
								) : (
									<p className="text-center text-xs text-gray-600">
										{mode === "webcam"
											? isRunning
												? "No objects detected"
												: "Start camera to detect objects"
											: "Upload an image to detect objects"}
									</p>
								)}
							</div>
						</div>
					</div>

					{/* Production Note */}
					<div className="mt-8 rounded-xl border border-white/10 bg-white/5 p-6">
						<div className="flex gap-3">
							<div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-lg bg-cw-green/20 text-lg">
								{"\u{1F916}"}
							</div>
							<div>
								<p className="text-sm font-medium text-white">
									On-Robot Vision System
								</p>
								<p className="mt-1 text-sm text-gray-400">
									This browser demo showcases
									general object detection
									across 80+ categories. The
									production CleanWalker robot
									runs our custom-trained model
									optimized for 60+ litter
									categories including
									cigarette butts, bottle
									caps, food wrappers, and
									more &mdash; with 97%+
									accuracy in real-world
									conditions.
								</p>
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* Sample Detection Cards */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="mb-8 text-center">
						<h2 className="text-2xl font-bold text-white md:text-3xl">
							Detection Examples
						</h2>
						<p className="mt-3 text-gray-400">
							How CleanWalker sees different
							environments and lighting conditions
						</p>
					</div>
					<div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
						{sampleScenes.map((scene) => (
							<DetectionCard
								key={scene.title}
								scene={scene}
							/>
						))}
					</div>
				</div>
			</section>

			{/* CTA */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold text-white">
						Ready to Deploy CleanWalker?
					</h2>
					<p className="mt-4 text-lg text-gray-400">
						This detection system runs on every
						CleanWalker robot, enabling autonomous litter
						collection across parks, campuses, and public
						spaces.
					</p>
					<a
						href="/contact"
						className="mt-8 inline-block rounded-lg bg-cw-green px-8 py-4 text-lg font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
				</div>
			</section>
		</div>
	);
}
