// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState } from "react";

const interestOptions = [
	"Pilot Program",
	"City-Wide Deployment",
	"Campus Solution",
	"Custom Solution",
	"Partnership",
	"Other",
];

const steps = [
	{
		number: "1",
		title: "Initial Consultation Call",
		description:
			"We discuss your specific needs, site conditions, and goals to understand how CleanWalker can help.",
	},
	{
		number: "2",
		title: "Site Assessment",
		description:
			"Our team evaluates your location, terrain, and operational requirements for optimal deployment.",
	},
	{
		number: "3",
		title: "Custom Proposal",
		description:
			"You receive a tailored plan with fleet sizing, coverage maps, and a deployment roadmap.",
	},
	{
		number: "4",
		title: "Pilot Deployment",
		description:
			"We deploy a pilot fleet so you can see measurable results before committing long-term.",
	},
];

const faqs = [
	{
		question: "How quickly can you deploy?",
		answer:
			"From signed agreement to robots on the ground, typical deployment takes 4–6 weeks. This includes site assessment, fleet configuration, mapping, and on-site commissioning. Urgent deployments for events or seasonal needs can be expedited.",
	},
	{
		question: "What size areas do you cover?",
		answer:
			"A single CleanWalker unit effectively covers 5–10 acres per shift. For larger sites — city parks, university campuses, festival grounds — we scale the fleet to match. Our largest deployments cover 200+ acres with coordinated multi-robot fleets.",
	},
	{
		question: "Do you operate in my region?",
		answer:
			"We're currently deploying across the continental United States and Western Europe, with expansion planned for additional markets. Contact us with your location and we'll confirm availability and timeline.",
	},
	{
		question: "What's the minimum commitment?",
		answer:
			"Our pilot programs start at 3 months, giving you enough time to see measurable impact on cleanliness scores and labor costs. After the pilot, you can scale up, adjust, or transition to a longer-term agreement.",
	},
	{
		question: "How does maintenance work?",
		answer:
			"CleanWalker is a fully managed service. We handle all maintenance, software updates, and repairs. Each unit runs remote diagnostics continuously, and our field team handles any physical service within 24 hours of a flag.",
	},
];

export default function ContactPage() {
	const [formData, setFormData] = useState({
		name: "",
		email: "",
		organization: "",
		role: "",
		interest: "",
		message: "",
	});
	const [submitted, setSubmitted] = useState(false);
	const [submitting, setSubmitting] = useState(false);
	const [error, setError] = useState("");
	const [openFaq, setOpenFaq] = useState<number | null>(null);

	function handleChange(
		e: React.ChangeEvent<
			HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement
		>,
	) {
		setFormData((prev) => ({ ...prev, [e.target.name]: e.target.value }));
	}

	async function handleSubmit(e: React.FormEvent) {
		e.preventDefault();
		setSubmitting(true);
		setError("");

		try {
			const res = await fetch("/api/contact", {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: JSON.stringify(formData),
			});

			if (!res.ok) {
				const data = await res.json();
				throw new Error(data.error || "Failed to send message.");
			}

			setSubmitted(true);
		} catch (err) {
			setError(
				err instanceof Error ? err.message : "Something went wrong. Please try again.",
			);
		} finally {
			setSubmitting(false);
		}
	}

	return (
		<div className="bg-cw-dark">
			{/* Hero */}
			<section className="border-b border-white/10 bg-cw-dark">
				<div className="mx-auto max-w-7xl px-6 py-24 text-center">
					<span className="inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm font-medium text-cw-green">
						Get in Touch
					</span>
					<h1 className="mt-6 text-4xl font-bold tracking-tight text-white sm:text-5xl">
						Let&apos;s Transform Your
						<br />
						Litter Management
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg leading-relaxed text-gray-400">
						Schedule a consultation with our team. We&apos;ll assess your
						site, design a custom solution, and show you how autonomous
						litter collection can save time and reduce costs.
					</p>
				</div>
			</section>

			{/* Form + Info */}
			<section className="bg-cw-dark">
				<div className="mx-auto max-w-7xl px-6 py-24">
					<div className="grid gap-16 lg:grid-cols-2">
						{/* LEFT — Contact Form */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								Send Us a Message
							</h2>
							<p className="mt-2 text-sm text-gray-400">
								Fill out the form and our sales team will get back to you.
							</p>

							{submitted ? (
								<div className="mt-8 rounded-2xl border border-cw-green/30 bg-cw-green/5 p-10 text-center">
									<div className="mx-auto flex h-14 w-14 items-center justify-center rounded-full bg-cw-green/20">
										<svg
											className="h-7 w-7 text-cw-green"
											fill="none"
											viewBox="0 0 24 24"
											strokeWidth={2}
											stroke="currentColor"
										>
											<path
												strokeLinecap="round"
												strokeLinejoin="round"
												d="M4.5 12.75l6 6 9-13.5"
											/>
										</svg>
									</div>
									<h3 className="mt-5 text-xl font-bold text-white">
										Message Sent
									</h3>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										Thank you for reaching out. Our team will review your
										message and respond within 24 hours.
									</p>
									<button
										onClick={() => {
											setSubmitted(false);
											setFormData({
												name: "",
												email: "",
												organization: "",
												role: "",
												interest: "",
												message: "",
											});
										}}
										className="mt-6 text-sm font-medium text-cw-green transition-colors hover:text-cw-green-dark"
									>
										Send another message
									</button>
								</div>
							) : (
								<form onSubmit={handleSubmit} className="mt-8 space-y-5">
									<div className="grid gap-5 sm:grid-cols-2">
										<div>
											<label
												htmlFor="name"
												className="mb-1.5 block text-sm font-medium text-gray-300"
											>
												Name
											</label>
											<input
												type="text"
												id="name"
												name="name"
												required
												value={formData.name}
												onChange={handleChange}
												placeholder="Your full name"
												className="w-full rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
											/>
										</div>
										<div>
											<label
												htmlFor="email"
												className="mb-1.5 block text-sm font-medium text-gray-300"
											>
												Email
											</label>
											<input
												type="email"
												id="email"
												name="email"
												required
												value={formData.email}
												onChange={handleChange}
												placeholder="you@organization.com"
												className="w-full rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
											/>
										</div>
									</div>

									<div className="grid gap-5 sm:grid-cols-2">
										<div>
											<label
												htmlFor="organization"
												className="mb-1.5 block text-sm font-medium text-gray-300"
											>
												Organization
											</label>
											<input
												type="text"
												id="organization"
												name="organization"
												value={formData.organization}
												onChange={handleChange}
												placeholder="City / University / Company"
												className="w-full rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
											/>
										</div>
										<div>
											<label
												htmlFor="role"
												className="mb-1.5 block text-sm font-medium text-gray-300"
											>
												Role / Title
											</label>
											<input
												type="text"
												id="role"
												name="role"
												value={formData.role}
												onChange={handleChange}
												placeholder="e.g. Facilities Director"
												className="w-full rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
											/>
										</div>
									</div>

									<div>
										<label
											htmlFor="interest"
											className="mb-1.5 block text-sm font-medium text-gray-300"
										>
											I&apos;m interested in:
										</label>
										<select
											id="interest"
											name="interest"
											required
											value={formData.interest}
											onChange={handleChange}
											className="w-full rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
										>
											<option value="" disabled className="bg-cw-dark">
												Select an option
											</option>
											{interestOptions.map((opt) => (
												<option key={opt} value={opt} className="bg-cw-dark">
													{opt}
												</option>
											))}
										</select>
									</div>

									<div>
										<label
											htmlFor="message"
											className="mb-1.5 block text-sm font-medium text-gray-300"
										>
											Message
										</label>
										<textarea
											id="message"
											name="message"
											rows={5}
											value={formData.message}
											onChange={handleChange}
											placeholder="Tell us about your site, goals, or questions..."
											className="w-full resize-none rounded-lg border border-white/10 bg-white/5 px-4 py-3 text-sm text-white placeholder-gray-500 outline-none transition-colors focus:border-cw-green focus:ring-1 focus:ring-cw-green"
										/>
									</div>

									{error && (
										<div className="rounded-lg border border-red-500/30 bg-red-500/10 px-4 py-3 text-sm text-red-400">
											{error}
										</div>
									)}

									<button
										type="submit"
										disabled={submitting}
										className="w-full rounded-lg bg-cw-green px-8 py-3.5 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark disabled:opacity-50"
									>
										{submitting ? "Sending..." : "Send Message"}
									</button>
								</form>
							)}
						</div>

						{/* RIGHT — What to Expect + Contact Info */}
						<div className="space-y-10">
							{/* What to Expect */}
							<div>
								<h2 className="text-2xl font-bold tracking-tight text-white">
									What to Expect
								</h2>
								<p className="mt-2 text-sm text-gray-400">
									Our process is designed to get you results quickly.
								</p>
								<div className="mt-8 space-y-6">
									{steps.map((step) => (
										<div key={step.number} className="flex gap-4">
											<div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-lg bg-cw-green/10 text-sm font-bold text-cw-green">
												{step.number}
											</div>
											<div>
												<h3 className="text-sm font-semibold text-white">
													{step.title}
												</h3>
												<p className="mt-1 text-sm leading-relaxed text-gray-400">
													{step.description}
												</p>
											</div>
										</div>
									))}
								</div>
							</div>

							{/* Contact Info */}
							<div className="rounded-2xl border border-white/10 bg-white/[0.03] p-8">
								<h3 className="text-lg font-semibold text-white">
									Contact Information
								</h3>
								<div className="mt-6 space-y-5">
									<div className="flex items-start gap-3">
										<div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-lg bg-cw-green/10">
											<svg
												className="h-5 w-5 text-cw-green"
												fill="none"
												viewBox="0 0 24 24"
												strokeWidth={1.5}
												stroke="currentColor"
											>
												<path
													strokeLinecap="round"
													strokeLinejoin="round"
													d="M21.75 6.75v10.5a2.25 2.25 0 01-2.25 2.25h-15a2.25 2.25 0 01-2.25-2.25V6.75m19.5 0A2.25 2.25 0 0019.5 4.5h-15a2.25 2.25 0 00-2.25 2.25m19.5 0v.243a2.25 2.25 0 01-1.07 1.916l-7.5 4.615a2.25 2.25 0 01-2.36 0L3.32 8.91a2.25 2.25 0 01-1.07-1.916V6.75"
												/>
											</svg>
										</div>
										<div>
											<p className="text-sm font-medium text-gray-300">
												Email
											</p>
											<a
												href="mailto:sales@cleanwalkerrobotics.com"
												className="text-sm text-cw-green transition-colors hover:text-cw-green-dark"
											>
												sales@cleanwalkerrobotics.com
											</a>
										</div>
									</div>

									<div className="flex items-start gap-3">
										<div className="flex h-10 w-10 shrink-0 items-center justify-center rounded-lg bg-cw-green/10">
											<svg
												className="h-5 w-5 text-cw-green"
												fill="none"
												viewBox="0 0 24 24"
												strokeWidth={1.5}
												stroke="currentColor"
											>
												<path
													strokeLinecap="round"
													strokeLinejoin="round"
													d="M12 6v6h4.5m4.5 0a9 9 0 11-18 0 9 9 0 0118 0z"
												/>
											</svg>
										</div>
										<div>
											<p className="text-sm font-medium text-gray-300">
												Response Time
											</p>
											<p className="text-sm text-gray-400">
												We respond within 24 hours
											</p>
										</div>
									</div>
								</div>
							</div>
						</div>
					</div>
				</div>
			</section>

			{/* FAQ */}
			<section className="border-t border-white/10 bg-cw-dark">
				<div className="mx-auto max-w-3xl px-6 py-24">
					<div className="text-center">
						<span className="inline-block rounded-full border border-cw-green/30 bg-cw-green/10 px-4 py-1.5 text-sm font-medium text-cw-green">
							FAQ
						</span>
						<h2 className="mt-6 text-3xl font-bold tracking-tight text-white">
							Common Questions
						</h2>
						<p className="mt-4 text-gray-400">
							Everything you need to know before getting started.
						</p>
					</div>

					<div className="mt-12 space-y-3">
						{faqs.map((faq, i) => (
							<div
								key={i}
								className="rounded-xl border border-white/10 bg-white/[0.03] transition-colors hover:border-white/20"
							>
								<button
									onClick={() => setOpenFaq(openFaq === i ? null : i)}
									aria-expanded={openFaq === i}
									aria-label={`${faq.question} — ${openFaq === i ? "collapse" : "expand"}`}
									className="flex w-full items-center justify-between px-6 py-5 text-left"
								>
									<span className="pr-4 text-sm font-semibold text-white">
										{faq.question}
									</span>
									<svg
										className={`h-5 w-5 shrink-0 text-gray-400 transition-transform ${openFaq === i ? "rotate-180" : ""}`}
										fill="none"
										viewBox="0 0 24 24"
										strokeWidth={2}
										stroke="currentColor"
									>
										<path
											strokeLinecap="round"
											strokeLinejoin="round"
											d="M19.5 8.25l-7.5 7.5-7.5-7.5"
										/>
									</svg>
								</button>
								{openFaq === i && (
									<div className="border-t border-white/5 px-6 pb-5 pt-4">
										<p className="text-sm leading-relaxed text-gray-400">
											{faq.answer}
										</p>
									</div>
								)}
							</div>
						))}
					</div>
				</div>
			</section>
		</div>
	);
}
