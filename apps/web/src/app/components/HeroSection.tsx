// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useEffect, useRef, useState } from "react";
import Image from "next/image";

function AnimatedCounter({
	end,
	suffix = "",
	prefix = "",
	duration = 2000,
	inView,
}: {
	end: number;
	suffix?: string;
	prefix?: string;
	duration?: number;
	inView: boolean;
}) {
	const [count, setCount] = useState(0);
	const hasAnimated = useRef(false);

	useEffect(() => {
		if (!inView || hasAnimated.current) return;
		hasAnimated.current = true;

		const startTime = performance.now();
		const animate = (now: number) => {
			const elapsed = now - startTime;
			const progress = Math.min(elapsed / duration, 1);
			// Ease-out cubic
			const eased = 1 - Math.pow(1 - progress, 3);
			setCount(Math.round(eased * end));
			if (progress < 1) requestAnimationFrame(animate);
		};
		requestAnimationFrame(animate);
	}, [inView, end, duration]);

	return (
		<span className="block text-2xl font-bold text-white">
			{prefix}
			{count}
			{suffix}
		</span>
	);
}

export default function HeroSection() {
	const statsRef = useRef<HTMLDivElement>(null);
	const [statsInView, setStatsInView] = useState(false);

	useEffect(() => {
		if (!statsRef.current) return;
		const observer = new IntersectionObserver(
			([entry]) => {
				if (entry.isIntersecting) setStatsInView(true);
			},
			{ threshold: 0.5 },
		);
		observer.observe(statsRef.current);
		return () => observer.disconnect();
	}, []);

	return (
		<section className="relative min-h-[90vh] overflow-hidden bg-cw-dark">
			<Image
				src="/renders/v3/hero-park.webp"
				alt="CleanWalker autonomous robot picking up a plastic bottle in a sunlit park"
				fill
				className="object-cover"
				priority
				sizes="100vw"
			/>
			<div className="absolute inset-0 bg-gradient-to-r from-cw-dark via-cw-dark/80 to-cw-dark/30" />

			{/* Floating geometric shapes */}
			<div className="absolute inset-0 overflow-hidden" aria-hidden="true">
				<div className="hero-shape hero-shape-1" />
				<div className="hero-shape hero-shape-2" />
				<div className="hero-shape hero-shape-3" />
				<div className="hero-shape hero-shape-4" />
				<div className="hero-shape hero-shape-5" />
			</div>

			<div className="relative z-10 flex min-h-[90vh] items-center px-6">
				<div className="mx-auto w-full max-w-7xl">
					<div className="max-w-2xl">
						<div className="mb-6 inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm font-medium text-cw-green backdrop-blur-sm">
							Smart City Infrastructure
						</div>
						<h1 className="text-4xl font-bold leading-tight tracking-tight text-white md:text-6xl">
							Autonomous Litter Collection for{" "}
							<span className="text-cw-green">Cleaner Spaces</span>
						</h1>
						<p className="mt-6 max-w-lg text-lg leading-relaxed text-gray-300">
							CleanWalker deploys intelligent, autonomous robots that keep parks,
							campuses, and public spaces clean — 24/7, rain or shine. No staffing
							challenges. No coverage gaps. Just cleaner spaces.
						</p>
						<div className="mt-10 flex flex-wrap gap-4">
							<a
								href="/contact"
								className="hero-pulse-btn rounded-lg bg-cw-green px-10 py-4 text-base font-semibold text-white transition-colors hover:bg-cw-green-dark"
							>
								Schedule a Demo
							</a>
							<a
								href="/product"
								className="rounded-lg border border-white/20 bg-white/5 px-10 py-4 text-base font-semibold text-white backdrop-blur-sm transition-colors hover:border-white/40 hover:bg-white/10"
							>
								See How It Works
							</a>
						</div>
						<div
							ref={statsRef}
							className="mt-10 grid grid-cols-3 gap-3 rounded-xl border border-white/10 bg-black/30 px-3 py-4 text-sm text-gray-400 backdrop-blur-sm sm:px-6 md:flex md:items-center md:gap-8"
						>
							<div className="text-center md:text-left">
								<AnimatedCounter end={24} suffix="/7" inView={statsInView} />
								<span className="text-xs sm:text-sm">Autonomous</span>
							</div>
							<div className="hidden h-10 w-px bg-white/20 md:block" />
							<div className="border-x border-white/10 text-center md:border-0 md:text-left">
								<AnimatedCounter end={50} suffix="+" inView={statsInView} />
								<span className="text-xs sm:text-sm">Litter types</span>
							</div>
							<div className="hidden h-10 w-px bg-white/20 md:block" />
							<div className="text-center md:text-left">
								<AnimatedCounter end={10} prefix="5-" inView={statsInView} />
								<span className="text-xs sm:text-sm">Acres/day</span>
							</div>
						</div>
					</div>
				</div>
			</div>
		</section>
	);
}
