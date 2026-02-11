// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState, useMemo } from "react";

const fmt = (n: number) =>
	new Intl.NumberFormat("en-US", {
		style: "currency",
		currency: "USD",
		maximumFractionDigits: 0,
	}).format(n);

export default function CostCalculatorPage() {
	const [acres, setAcres] = useState(20);
	const [workers, setWorkers] = useState(5);
	const [hourlyRate, setHourlyRate] = useState(25);
	const [hoursPerDay, setHoursPerDay] = useState(8);
	const [daysPerWeek, setDaysPerWeek] = useState(5);
	const [complaints, setComplaints] = useState(50);

	const calc = useMemo(() => {
		const annualLaborCost = workers * hourlyRate * hoursPerDay * daysPerWeek * 52;
		const recommendedUnits = Math.max(1, Math.ceil(acres / 7.5));
		const annualRobotCost = recommendedUnits * 3250 * 12;
		const annualSavings = annualLaborCost - annualRobotCost;
		const roiPct = annualRobotCost > 0 ? (annualSavings / annualRobotCost) * 100 : 0;
		const paybackMonths =
			annualSavings > 0 ? (annualRobotCost / annualSavings) * 12 : Infinity;
		const fiveYearLaborCost = annualLaborCost * 5;
		const fiveYearRobotCost = annualRobotCost * 5;
		const fiveYearSavings = annualSavings * 5;
		const complaintReduction = Math.round(complaints * 0.7);
		const remainingComplaints = complaints - complaintReduction;
		const coverageIncreasePct = Math.round(
			((24 * 7) / (hoursPerDay * daysPerWeek)) * 100 - 100,
		);

		return {
			annualLaborCost,
			recommendedUnits,
			annualRobotCost,
			annualSavings,
			roiPct,
			paybackMonths,
			fiveYearLaborCost,
			fiveYearRobotCost,
			fiveYearSavings,
			remainingComplaints,
			coverageIncreasePct,
			isPositive: annualSavings > 0,
		};
	}, [acres, workers, hourlyRate, hoursPerDay, daysPerWeek, complaints]);

	return (
		<div className="min-h-screen bg-cw-dark">
			<style>{`
				.cw-range {
					-webkit-appearance: none;
					appearance: none;
					background: transparent;
					cursor: pointer;
					width: 100%;
					height: 24px;
				}
				.cw-range::-webkit-slider-track {
					background: rgba(255, 255, 255, 0.1);
					border-radius: 4px;
					height: 6px;
				}
				.cw-range::-webkit-slider-thumb {
					-webkit-appearance: none;
					background: #22c55e;
					border-radius: 50%;
					height: 20px;
					width: 20px;
					margin-top: -7px;
					box-shadow: 0 0 8px rgba(34, 197, 94, 0.4);
				}
				.cw-range::-moz-range-track {
					background: rgba(255, 255, 255, 0.1);
					border-radius: 4px;
					height: 6px;
					border: none;
				}
				.cw-range::-moz-range-thumb {
					background: #22c55e;
					border-radius: 50%;
					height: 20px;
					width: 20px;
					border: none;
					box-shadow: 0 0 8px rgba(34, 197, 94, 0.4);
				}
			`}</style>

			{/* Header */}
			<header className="border-b border-white/10 bg-black/40 backdrop-blur-sm">
				<div className="mx-auto flex max-w-[1400px] items-center justify-between px-6 py-3">
					<div className="flex items-center gap-3">
						<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
							<svg
								className="h-4 w-4 text-cw-green"
								viewBox="0 0 24 24"
								fill="none"
								stroke="currentColor"
								strokeWidth={2}
							>
								<path d="M12 8c-1.657 0-3 .895-3 2s1.343 2 3 2 3 .895 3 2-1.343 2-3 2m0-8c1.11 0 2.08.402 2.599 1M12 8V7m0 1v8m0 0v1m0-1c-1.11 0-2.08-.402-2.599-1M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
							</svg>
						</div>
						<div>
							<h1 className="font-mono text-sm font-bold text-white">
								Cost Savings Calculator
							</h1>
							<p className="font-mono text-[10px] text-gray-500">
								ROI analysis for CleanWalker deployment
							</p>
						</div>
					</div>
					<div className="hidden items-center gap-2 sm:flex">
						<span className="rounded-full bg-cw-green/20 px-3 py-1 text-xs font-medium text-cw-green">
							Interactive
						</span>
					</div>
				</div>
			</header>

			<div className="mx-auto max-w-[1400px] px-6 py-6">
				{/* Back link */}
				<a
					href="/demos"
					className="mb-6 inline-flex items-center gap-2 font-mono text-xs text-gray-500 transition-colors hover:text-cw-green"
				>
					<svg
						className="h-3 w-3"
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

				{/* Title */}
				<div className="mb-8">
					<h2 className="text-3xl font-bold text-white md:text-4xl">
						How Much Can You Save?
					</h2>
					<p className="mt-3 max-w-2xl text-gray-400">
						Adjust the inputs below to match your facility. See exactly how
						CleanWalker autonomous litter collection compares to manual labor
						&mdash; with real numbers.
					</p>
				</div>

				{/* Main Grid */}
				<div className="grid gap-8 lg:grid-cols-5">
					{/* Inputs */}
					<div className="flex flex-col gap-4 lg:col-span-2">
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<p className="mb-6 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								Your Facility
							</p>

							<SliderInput
								label="Park / Area Size"
								value={acres}
								min={1}
								max={100}
								suffix=" acres"
								onChange={setAcres}
							/>
							<SliderInput
								label="Manual Workers"
								value={workers}
								min={1}
								max={20}
								onChange={setWorkers}
							/>
							<SliderInput
								label="Avg. Hourly Labor Cost"
								value={hourlyRate}
								min={15}
								max={50}
								prefix="$"
								suffix="/hr"
								onChange={setHourlyRate}
							/>
							<SliderInput
								label="Hours Worked / Day"
								value={hoursPerDay}
								min={4}
								max={12}
								suffix="h"
								onChange={setHoursPerDay}
							/>
							<SliderInput
								label="Days Per Week"
								value={daysPerWeek}
								min={5}
								max={7}
								suffix=" days"
								onChange={setDaysPerWeek}
							/>
							<SliderInput
								label="Annual Litter Complaints"
								value={complaints}
								min={0}
								max={500}
								step={5}
								onChange={setComplaints}
								last
							/>
						</div>
					</div>

					{/* Results */}
					<div className="flex flex-col gap-6 lg:col-span-3">
						{/* Side-by-side comparison */}
						<div className="grid gap-4 md:grid-cols-2">
							{/* Manual */}
							<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
								<div className="mb-4 flex items-center gap-2">
									<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-red-500/20">
										<svg
											className="h-4 w-4 text-red-400"
											viewBox="0 0 24 24"
											fill="none"
											stroke="currentColor"
											strokeWidth={2}
										>
											<path d="M17 21v-2a4 4 0 00-4-4H5a4 4 0 00-4 4v2" />
											<circle cx="9" cy="7" r="4" />
											<path d="M23 21v-2a4 4 0 00-3-3.87" />
											<path d="M16 3.13a4 4 0 010 7.75" />
										</svg>
									</div>
									<div>
										<h3 className="font-mono text-sm font-bold text-white">
											Manual Labor
										</h3>
										<p className="font-mono text-[10px] text-gray-500">
											Current approach
										</p>
									</div>
								</div>
								<div className="space-y-3">
									<div>
										<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
											Annual Cost
										</p>
										<p className="font-mono text-2xl font-bold text-red-400">
											{fmt(calc.annualLaborCost)}
										</p>
									</div>
									<div className="grid grid-cols-2 gap-3">
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Coverage
											</p>
											<p className="font-mono text-sm font-bold text-white">
												{hoursPerDay}h/day
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Schedule
											</p>
											<p className="font-mono text-sm font-bold text-white">
												{daysPerWeek} days/wk
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Workers
											</p>
											<p className="font-mono text-sm font-bold text-white">
												{workers}
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Complaints
											</p>
											<p className="font-mono text-sm font-bold text-white">
												{complaints}/yr
											</p>
										</div>
									</div>
									<div>
										<p className="font-mono text-[10px] text-gray-500">
											Data &amp; Analytics
										</p>
										<p className="font-mono text-xs text-gray-400">
											Manual reporting only
										</p>
									</div>
								</div>
							</div>

							{/* CleanWalker */}
							<div className="rounded-xl border border-cw-green/30 bg-cw-green/5 p-6">
								<div className="mb-4 flex items-center gap-2">
									<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/20">
										<svg
											className="h-4 w-4 text-cw-green"
											viewBox="0 0 24 24"
											fill="none"
											stroke="currentColor"
											strokeWidth={2}
										>
											<rect x="2" y="3" width="20" height="14" rx="2" />
											<path d="M8 21h8M12 17v4" />
											<circle cx="8" cy="10" r="1" fill="currentColor" />
											<circle cx="16" cy="10" r="1" fill="currentColor" />
										</svg>
									</div>
									<div>
										<h3 className="font-mono text-sm font-bold text-white">
											CleanWalker
										</h3>
										<p className="font-mono text-[10px] text-cw-green">
											Recommended
										</p>
									</div>
								</div>
								<div className="space-y-3">
									<div>
										<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
											Annual Cost
										</p>
										<p className="font-mono text-2xl font-bold text-cw-green">
											{fmt(calc.annualRobotCost)}
										</p>
									</div>
									<div className="grid grid-cols-2 gap-3">
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Coverage
											</p>
											<p className="font-mono text-sm font-bold text-cw-green">
												24h/day
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Schedule
											</p>
											<p className="font-mono text-sm font-bold text-cw-green">
												7 days/wk
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Units
											</p>
											<p className="font-mono text-sm font-bold text-white">
												{calc.recommendedUnits} CW-1
												{calc.recommendedUnits > 1 ? "s" : ""}
											</p>
										</div>
										<div>
											<p className="font-mono text-[10px] text-gray-500">
												Complaints
											</p>
											<p className="font-mono text-sm font-bold text-cw-green">
												{calc.remainingComplaints}/yr
											</p>
										</div>
									</div>
									<div>
										<p className="font-mono text-[10px] text-gray-500">
											Data &amp; Analytics
										</p>
										<p className="font-mono text-xs text-cw-green">
											Real-time dashboard included
										</p>
									</div>
								</div>
							</div>
						</div>

						{/* Key metrics */}
						<div className="grid grid-cols-2 gap-4 md:grid-cols-4">
							<MetricCard
								label="Annual Savings"
								value={fmt(calc.annualSavings)}
								positive={calc.isPositive}
								colored
							/>
							<MetricCard
								label="ROI"
								value={`${calc.roiPct > 0 ? "+" : ""}${calc.roiPct.toFixed(0)}%`}
								positive={calc.isPositive}
								colored
							/>
							<MetricCard
								label="Payback Period"
								value={
									calc.paybackMonths === Infinity
										? "N/A"
										: calc.paybackMonths < 1
											? "<1 mo"
											: `${calc.paybackMonths.toFixed(1)} mo`
								}
							/>
							<MetricCard
								label="5-Year Savings"
								value={fmt(calc.fiveYearSavings)}
								positive={calc.isPositive}
								colored
							/>
						</div>

						{/* 5-Year bar chart */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<p className="mb-6 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								5-Year Cumulative Cost Comparison
							</p>
							<div
								className="flex items-end gap-3"
								style={{ height: "200px" }}
							>
								{[1, 2, 3, 4, 5].map((year) => {
									const manualCost = calc.annualLaborCost * year;
									const robotCost = calc.annualRobotCost * year;
									const maxCost = calc.fiveYearLaborCost || 1;
									const manualPct = Math.max(
										2,
										(manualCost / maxCost) * 100,
									);
									const robotPct = Math.max(
										2,
										(robotCost / maxCost) * 100,
									);
									return (
										<div
											key={year}
											className="flex flex-1 flex-col items-center gap-2"
										>
											<div
												className="flex w-full items-end justify-center gap-1"
												style={{ height: "170px" }}
											>
												<div
													className="flex-1 rounded-t bg-red-500/50 transition-all duration-500"
													style={{ height: `${manualPct}%` }}
													title={`Manual: ${fmt(manualCost)}`}
												/>
												<div
													className="flex-1 rounded-t bg-cw-green/50 transition-all duration-500"
													style={{ height: `${robotPct}%` }}
													title={`CleanWalker: ${fmt(robotCost)}`}
												/>
											</div>
											<span className="font-mono text-[10px] text-gray-500">
												Y{year}
											</span>
										</div>
									);
								})}
							</div>
							<div className="mt-4 flex items-center justify-between border-t border-white/5 pt-4">
								<div className="flex gap-4">
									<div className="flex items-center gap-2">
										<div className="h-3 w-3 rounded bg-red-500/50" />
										<span className="font-mono text-[10px] text-gray-400">
											Manual Labor
										</span>
									</div>
									<div className="flex items-center gap-2">
										<div className="h-3 w-3 rounded bg-cw-green/50" />
										<span className="font-mono text-[10px] text-gray-400">
											CleanWalker
										</span>
									</div>
								</div>
								<div className="text-right">
									<p className="font-mono text-[10px] text-gray-500">
										5-Year Totals
									</p>
									<p className="font-mono text-[10px] text-gray-400">
										<span className="text-red-400">
											{fmt(calc.fiveYearLaborCost)}
										</span>
										{" vs "}
										<span className="text-cw-green">
											{fmt(calc.fiveYearRobotCost)}
										</span>
									</p>
								</div>
							</div>
						</div>

						{/* Additional benefits */}
						<div className="rounded-xl border border-white/10 bg-white/[0.03] p-6">
							<p className="mb-4 font-mono text-[10px] uppercase tracking-wider text-gray-400">
								Additional Benefits Included
							</p>
							<div className="grid gap-3 md:grid-cols-3">
								<div className="rounded-lg border border-white/5 bg-white/[0.02] p-4">
									<div className="mb-2 flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/10">
										<svg
											className="h-4 w-4 text-cw-green"
											viewBox="0 0 24 24"
											fill="none"
											stroke="currentColor"
											strokeWidth={2}
										>
											<circle cx="12" cy="12" r="10" />
											<path d="M12 6v6l4 2" />
										</svg>
									</div>
									<h4 className="font-mono text-xs font-bold text-white">
										24/7 Coverage
									</h4>
									<p className="mt-1 font-mono text-[10px] leading-relaxed text-gray-500">
										Continuous operation vs {hoursPerDay}h/day manual.{" "}
										{calc.coverageIncreasePct}% more coverage hours.
									</p>
								</div>
								<div className="rounded-lg border border-white/5 bg-white/[0.02] p-4">
									<div className="mb-2 flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/10">
										<svg
											className="h-4 w-4 text-cw-green"
											viewBox="0 0 24 24"
											fill="none"
											stroke="currentColor"
											strokeWidth={2}
										>
											<path d="M22 11.08V12a10 10 0 11-5.93-9.14" />
											<path d="M22 4L12 14.01l-3-3" />
										</svg>
									</div>
									<h4 className="font-mono text-xs font-bold text-white">
										70% Fewer Complaints
									</h4>
									<p className="mt-1 font-mono text-[10px] leading-relaxed text-gray-500">
										Estimated reduction from {complaints} to{" "}
										{calc.remainingComplaints} complaints/year through
										consistent coverage.
									</p>
								</div>
								<div className="rounded-lg border border-white/5 bg-white/[0.02] p-4">
									<div className="mb-2 flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green/10">
										<svg
											className="h-4 w-4 text-cw-green"
											viewBox="0 0 24 24"
											fill="none"
											stroke="currentColor"
											strokeWidth={2}
										>
											<path d="M21 12c0 1.66-4 3-9 3s-9-1.34-9-3" />
											<path d="M3 5v14c0 1.66 4 3 9 3s9-1.34 9-3V5" />
											<path d="M21 5c0 1.66-4 3-9 3s-9-1.34-9-3 4-3 9-3 9 1.34 9 3" />
											<path d="M3 12c0 1.66 4 3 9 3s9-1.34 9-3" />
										</svg>
									</div>
									<h4 className="font-mono text-xs font-bold text-white">
										Data Analytics
									</h4>
									<p className="mt-1 font-mono text-[10px] leading-relaxed text-gray-500">
										Real-time litter heatmaps, coverage reports, and trend
										analysis. No additional cost.
									</p>
								</div>
							</div>
						</div>
					</div>
				</div>

				{/* CTA */}
				<div className="mt-12 rounded-2xl border border-cw-green/20 bg-gradient-to-br from-cw-green/10 to-transparent p-8 text-center md:p-12">
					<h3 className="text-2xl font-bold text-white md:text-3xl">
						See These Savings at Your Facility
					</h3>
					<p className="mx-auto mt-4 max-w-xl text-gray-400">
						Our team will walk your site, analyze your specific needs, and
						provide a detailed deployment plan with guaranteed cost projections.
					</p>
					<a
						href="/contact"
						className="mt-8 inline-block rounded-lg bg-cw-green px-8 py-4 text-lg font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Schedule a Site Assessment
					</a>
				</div>

				{/* Disclaimer */}
				<div className="mt-8 border-t border-white/5 pt-6 text-center">
					<p className="font-mono text-[10px] text-gray-600">
						Calculations based on estimated averages. Actual savings may vary
						based on facility-specific conditions. CleanWalker pricing at
						$3,250/unit/month (RaaS model). Contact sales for custom quotes.
					</p>
				</div>
			</div>
		</div>
	);
}

function SliderInput({
	label,
	value,
	min,
	max,
	step,
	prefix,
	suffix,
	onChange,
	last,
}: {
	label: string;
	value: number;
	min: number;
	max: number;
	step?: number;
	prefix?: string;
	suffix?: string;
	onChange: (v: number) => void;
	last?: boolean;
}) {
	const display = `${prefix || ""}${value}${suffix || ""}`;
	const minLabel = `${prefix || ""}${min}${suffix || ""}`;
	const maxLabel = `${prefix || ""}${max}${suffix || ""}`;

	return (
		<div className={last ? "" : "mb-6"}>
			<div className="mb-2 flex items-center justify-between">
				<label className="font-mono text-xs text-gray-400">{label}</label>
				<span className="font-mono text-sm font-bold text-white">
					{display}
				</span>
			</div>
			<input
				type="range"
				className="cw-range"
				min={min}
				max={max}
				step={step || 1}
				value={value}
				onChange={(e) => onChange(Number(e.target.value))}
			/>
			<div className="mt-1 flex justify-between">
				<span className="font-mono text-[9px] text-gray-600">{minLabel}</span>
				<span className="font-mono text-[9px] text-gray-600">{maxLabel}</span>
			</div>
		</div>
	);
}

function MetricCard({
	label,
	value,
	positive,
	colored,
}: {
	label: string;
	value: string;
	positive?: boolean;
	colored?: boolean;
}) {
	const borderClass = colored
		? positive
			? "border-cw-green/30 bg-cw-green/5"
			: "border-red-500/30 bg-red-500/5"
		: "border-white/10 bg-white/[0.03]";
	const textClass = colored
		? positive
			? "text-cw-green"
			: "text-red-400"
		: "text-white";

	return (
		<div className={`rounded-xl border p-4 ${borderClass}`}>
			<p className="font-mono text-[10px] uppercase tracking-wider text-gray-500">
				{label}
			</p>
			<p className={`mt-1 font-mono text-xl font-bold ${textClass}`}>
				{value}
			</p>
		</div>
	);
}
