// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useCallback } from "react";

interface Detection {
	label: string;
	confidence: number;
	box: { top: string; left: string; width: string; height: string };
	color: string;
}

interface SampleScene {
	title: string;
	gradient: string;
	icon: string;
	detections: Detection[];
}

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

const sampleScenes: SampleScene[] = [
	{
		title: "City Park — Morning Patrol",
		gradient: "from-green-900/60 via-emerald-900/40 to-cw-dark",
		icon: "🌳",
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
		title: "Beach Boardwalk — Afternoon",
		gradient: "from-cyan-900/60 via-blue-900/40 to-cw-dark",
		icon: "🏖️",
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
		title: "Parking Lot — Evening",
		gradient: "from-slate-800/80 via-gray-900/60 to-cw-dark",
		icon: "🅿️",
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
		title: "University Campus — Midday",
		gradient: "from-amber-900/50 via-orange-900/30 to-cw-dark",
		icon: "🎓",
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
		title: "Sidewalk — Rush Hour",
		gradient: "from-zinc-800/70 via-neutral-900/50 to-cw-dark",
		icon: "🚶",
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
		title: "Playground — Weekend",
		gradient: "from-rose-900/40 via-pink-900/30 to-cw-dark",
		icon: "🎪",
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

const UPLOAD_LITTER_POOL: { label: string; confidence: number }[] = [
	{ label: "Plastic Bottle", confidence: 97 },
	{ label: "Aluminum Can", confidence: 96 },
	{ label: "Cigarette Butt", confidence: 94 },
	{ label: "Candy Wrapper", confidence: 92 },
	{ label: "Paper Cup", confidence: 91 },
	{ label: "Plastic Bag", confidence: 90 },
	{ label: "Food Container", confidence: 95 },
	{ label: "Straw", confidence: 88 },
	{ label: "Napkin", confidence: 85 },
	{ label: "Glass Bottle", confidence: 93 },
	{ label: "Chip Bag", confidence: 91 },
	{ label: "Bottle Cap", confidence: 87 },
	{ label: "Coffee Cup Lid", confidence: 89 },
	{ label: "Tissue", confidence: 83 },
	{ label: "Styrofoam Piece", confidence: 86 },
];

function generateMockDetections(): Detection[] {
	const count = 3 + Math.floor(Math.random() * 4);
	const shuffled = [...UPLOAD_LITTER_POOL].sort(() => Math.random() - 0.5);
	const selected = shuffled.slice(0, count);

	return selected.map((item, i) => {
		const jitter = Math.floor(Math.random() * 6) - 3;
		return {
			label: item.label,
			confidence: Math.max(75, Math.min(99, item.confidence + jitter)),
			box: {
				top: `${10 + Math.random() * 55}%`,
				left: `${5 + Math.random() * 60}%`,
				width: `${10 + Math.random() * 15}%`,
				height: `${10 + Math.random() * 25}%`,
			},
			color: DETECTION_COLORS[i % DETECTION_COLORS.length],
		};
	});
}

function DetectionCard({ scene }: { scene: SampleScene }) {
	return (
		<div className="group overflow-hidden rounded-2xl border border-white/10 bg-white/5 transition-all hover:border-cw-green/30 hover:bg-white/[0.07]">
			{/* Image area with bounding boxes */}
			<div
				className={`relative aspect-video bg-gradient-to-br ${scene.gradient} overflow-hidden`}
			>
				{/* Scene label */}
				<div className="absolute left-3 top-3 z-10 rounded-full bg-black/50 px-3 py-1 text-xs text-gray-300 backdrop-blur-sm">
					<span className="mr-1">{scene.icon}</span>
					{scene.title}
				</div>
				{/* Detection count badge */}
				<div className="absolute right-3 top-3 z-10 rounded-full bg-cw-green/20 px-3 py-1 text-xs font-medium text-cw-green backdrop-blur-sm">
					{scene.detections.length} detected
				</div>
				{/* Bounding boxes */}
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
				{/* Scan line animation */}
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

			{/* Detection list */}
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
								className="text-sm font-mono font-medium"
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

function UploadSection() {
	const [state, setState] = useState<
		"idle" | "processing" | "done"
	>("idle");
	const [results, setResults] = useState<Detection[]>([]);
	const [fileName, setFileName] = useState("");

	const handleFile = useCallback(
		(file: File) => {
			if (state === "processing") return;
			setFileName(file.name);
			setState("processing");
			setResults([]);

			// Simulate processing delay
			setTimeout(() => {
				setResults(generateMockDetections());
				setState("done");
			}, 1800 + Math.random() * 1200);
		},
		[state],
	);

	const handleDrop = useCallback(
		(e: React.DragEvent) => {
			e.preventDefault();
			const file = e.dataTransfer.files[0];
			if (file) handleFile(file);
		},
		[handleFile],
	);

	const handleChange = useCallback(
		(e: React.ChangeEvent<HTMLInputElement>) => {
			const file = e.target.files?.[0];
			if (file) handleFile(file);
		},
		[handleFile],
	);

	const reset = () => {
		setState("idle");
		setResults([]);
		setFileName("");
	};

	return (
		<div className="mx-auto max-w-3xl">
			{state === "idle" && (
				<label
					className="flex cursor-pointer flex-col items-center justify-center rounded-2xl border-2 border-dashed border-white/20 bg-white/5 px-8 py-16 transition-all hover:border-cw-green/50 hover:bg-white/[0.07]"
					onDragOver={(e) => e.preventDefault()}
					onDrop={handleDrop}
				>
					<input
						type="file"
						accept="image/*"
						className="hidden"
						onChange={handleChange}
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
						Drop an image here or click to upload
					</p>
					<p className="mt-2 text-sm text-gray-400">
						JPG, PNG, or WebP — any scene with potential litter
					</p>
				</label>
			)}

			{state === "processing" && (
				<div className="flex flex-col items-center rounded-2xl border border-white/10 bg-white/5 px-8 py-16">
					<div className="relative mb-6 h-16 w-16">
						<div className="absolute inset-0 animate-spin rounded-full border-4 border-cw-green/20 border-t-cw-green" />
						<div className="absolute inset-2 animate-spin rounded-full border-4 border-cw-green/10 border-b-cw-green/60" style={{ animationDirection: "reverse", animationDuration: "1.5s" }} />
					</div>
					<p className="text-lg font-medium text-white">
						Analyzing {fileName}...
					</p>
					<div className="mt-4 w-full max-w-xs">
						<div className="h-1.5 overflow-hidden rounded-full bg-white/10">
							<div
								className="h-full rounded-full bg-cw-green"
								style={{
									animation: "progressBar 2s ease-in-out",
								}}
							/>
						</div>
					</div>
					<div className="mt-4 space-y-1 text-center text-sm text-gray-400">
						<p>Running YOLO inference pipeline...</p>
						<p className="text-xs text-gray-500">
							Model: CleanWalker-YOLO v3.2 &middot; 640×640
						</p>
					</div>
				</div>
			)}

			{state === "done" && (
				<div className="overflow-hidden rounded-2xl border border-cw-green/30 bg-white/5">
					{/* Result header */}
					<div className="flex items-center justify-between border-b border-white/10 px-6 py-4">
						<div>
							<p className="text-sm text-gray-400">{fileName}</p>
							<p className="text-lg font-semibold text-white">
								{results.length} items detected
							</p>
						</div>
						<button
							onClick={reset}
							className="rounded-lg bg-white/10 px-4 py-2 text-sm text-gray-300 transition-colors hover:bg-white/20"
						>
							Try Another
						</button>
					</div>

					{/* Mock image with detections */}
					<div className="relative aspect-video bg-gradient-to-br from-emerald-900/40 via-teal-900/30 to-cw-dark">
						{results.map((det) => (
							<div
								key={det.label}
								className="absolute animate-fadeIn"
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
									style={{
										backgroundColor: det.color,
									}}
								>
									{det.label}
								</span>
							</div>
						))}
					</div>

					{/* Results table */}
					<div className="p-6">
						<div className="space-y-3">
							{results.map((det) => (
								<div
									key={det.label}
									className="flex items-center justify-between rounded-lg bg-white/5 px-4 py-3"
								>
									<div className="flex items-center gap-3">
										<span
											className="inline-block h-3 w-3 rounded-full"
											style={{
												backgroundColor: det.color,
											}}
										/>
										<span className="text-sm font-medium text-white">
											{det.label}
										</span>
									</div>
									<div className="flex items-center gap-4">
										<div className="h-1.5 w-24 overflow-hidden rounded-full bg-white/10">
											<div
												className="h-full rounded-full"
												style={{
													width: `${det.confidence}%`,
													backgroundColor: det.color,
												}}
											/>
										</div>
										<span className="w-10 text-right font-mono text-sm font-medium text-cw-green">
											{det.confidence}%
										</span>
									</div>
								</div>
							))}
						</div>
						<p className="mt-4 text-center text-xs text-gray-500">
							Inference time: {(60 + Math.random() * 35).toFixed(0)}ms &middot; CleanWalker-YOLO v3.2
						</p>
					</div>
				</div>
			)}
		</div>
	);
}

export default function LitterDetectionDemoPage() {
	return (
		<div className="min-h-screen bg-cw-dark">
			{/* Inline keyframes */}
			<style>{`
				@keyframes scanline {
					0%, 100% { top: 0%; opacity: 0; }
					10% { opacity: 1; }
					90% { opacity: 1; }
					50% { top: 100%; }
				}
				@keyframes progressBar {
					0% { width: 0%; }
					30% { width: 45%; }
					60% { width: 70%; }
					90% { width: 90%; }
					100% { width: 100%; }
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
						AI Litter Detection
					</h1>
					<p className="mt-6 text-lg leading-relaxed text-gray-400">
						Our custom YOLO-based perception system identifies and classifies 50+ types of
						litter in real-time. See how CleanWalker sees the world — detecting bottles,
						wrappers, cigarette butts, and more across any environment.
					</p>
				</div>
			</section>

			{/* Stats */}
			<section className="px-6 pb-16">
				<div className="mx-auto grid max-w-5xl grid-cols-2 gap-4 md:grid-cols-4">
					{[
						{ value: "50+", label: "Litter types detected" },
						{ value: "97.3%", label: "Accuracy" },
						{ value: "<100ms", label: "Inference time" },
						{ value: "All", label: "Conditions supported" },
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

			{/* Sample Analyses */}
			<section className="px-6 pb-24">
				<div className="mx-auto max-w-7xl">
					<div className="mb-8 text-center">
						<h2 className="text-2xl font-bold text-white md:text-3xl">
							Sample Analyses
						</h2>
						<p className="mt-3 text-gray-400">
							Real detection results from different environments and lighting conditions
						</p>
					</div>
					<div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
						{sampleScenes.map((scene) => (
							<DetectionCard key={scene.title} scene={scene} />
						))}
					</div>
				</div>
			</section>

			{/* Upload Your Own */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-7xl">
					<div className="mb-10 text-center">
						<h2 className="text-2xl font-bold text-white md:text-3xl">
							Try It Yourself
						</h2>
						<p className="mt-3 text-gray-400">
							Upload any image and see our AI detection in action
						</p>
					</div>
					<UploadSection />
				</div>
			</section>

			{/* CTA */}
			<section className="border-t border-white/10 px-6 py-24">
				<div className="mx-auto max-w-3xl text-center">
					<h2 className="text-3xl font-bold text-white">
						Ready to Deploy CleanWalker?
					</h2>
					<p className="mt-4 text-lg text-gray-400">
						This detection system runs on every CleanWalker robot, enabling autonomous
						litter collection across parks, campuses, and public spaces.
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
