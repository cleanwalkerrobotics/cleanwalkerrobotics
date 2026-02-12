// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";

export const metadata: Metadata = {
	title: "Privacy Policy",
	description:
		"CleanWalker Robotics privacy policy. Learn how we collect, use, and protect your personal data.",
	openGraph: {
		title: "Privacy Policy — CleanWalker Robotics",
		description:
			"Learn how CleanWalker Robotics collects, uses, and protects your personal data.",
	},
};

export default function PrivacyPolicyPage() {
	return (
		<div className="bg-cw-dark">
			{/* Hero */}
			<section className="border-b border-white/10 bg-cw-dark">
				<div className="mx-auto max-w-7xl px-6 py-24 text-center">
					<h1 className="text-4xl font-bold tracking-tight text-white sm:text-5xl">
						Privacy Policy
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg leading-relaxed text-gray-400">
						Your privacy matters to us. This policy explains how we collect,
						use, and protect your personal information.
					</p>
					<p className="mt-4 text-sm text-gray-500">
						Last updated: February 12, 2026
					</p>
				</div>
			</section>

			{/* Content */}
			<section className="bg-cw-dark">
				<div className="mx-auto max-w-3xl px-6 py-24">
					<div className="space-y-16">
						{/* Introduction */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								1. Introduction
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									MB Software Studio LLC (&ldquo;CleanWalker
									Robotics&rdquo;, &ldquo;we&rdquo;, &ldquo;us&rdquo;,
									or &ldquo;our&rdquo;) operates the website at
									cleanwalkerrobotics.com (the &ldquo;Site&rdquo;). This
									Privacy Policy describes how we collect, use, disclose,
									and protect your personal information when you visit our
									Site or use our services.
								</p>
								<p>
									By using our Site, you agree to the collection and use
									of information in accordance with this policy. If you do
									not agree with this policy, please do not use our Site.
								</p>
							</div>
						</div>

						{/* Data We Collect */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								2. Information We Collect
							</h2>
							<div className="mt-6 space-y-6">
								<div>
									<h3 className="text-lg font-semibold text-gray-200">
										Information You Provide
									</h3>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										When you use our contact form, you may provide us
										with:
									</p>
									<ul className="mt-3 list-inside list-disc space-y-2 text-sm text-gray-400">
										<li>Your name</li>
										<li>Email address</li>
										<li>Organization name</li>
										<li>Job title or role</li>
										<li>Area of interest (e.g., pilot program, deployment type)</li>
										<li>Any additional information you include in your message</li>
									</ul>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										All contact form fields except name, email, and area
										of interest are optional. We only collect information
										you voluntarily provide.
									</p>
								</div>

								<div>
									<h3 className="text-lg font-semibold text-gray-200">
										Information Collected Automatically
									</h3>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										When you visit our Site, we may automatically
										collect certain technical information, including:
									</p>
									<ul className="mt-3 list-inside list-disc space-y-2 text-sm text-gray-400">
										<li>IP address</li>
										<li>Browser type and version</li>
										<li>Operating system</li>
										<li>Referring URL</li>
										<li>Pages visited and time spent on pages</li>
										<li>Device type (desktop, mobile, tablet)</li>
									</ul>
								</div>
							</div>
						</div>

						{/* How We Use Data */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								3. How We Use Your Information
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>We use the information we collect to:</p>
								<ul className="list-inside list-disc space-y-2">
									<li>Respond to your inquiries and contact form submissions</li>
									<li>Provide information about our products and services</li>
									<li>Improve our Site and user experience</li>
									<li>Analyze Site usage and performance</li>
									<li>Comply with legal obligations</li>
								</ul>
								<p>
									We do not sell your personal information to third
									parties. We do not use your data for automated
									decision-making or profiling.
								</p>
							</div>
						</div>

						{/* Third-Party Services */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								4. Third-Party Services
							</h2>
							<div className="mt-6 space-y-6">
								<div>
									<h3 className="text-lg font-semibold text-gray-200">
										Vercel
									</h3>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										Our Site is hosted on Vercel. Vercel may collect
										standard web server logs including IP addresses,
										browser information, and page requests. Vercel&apos;s
										privacy policy is available at{" "}
										<a
											href="https://vercel.com/legal/privacy-policy"
											target="_blank"
											rel="noopener noreferrer"
											className="text-cw-green transition-colors hover:text-cw-green-dark"
										>
											vercel.com/legal/privacy-policy
										</a>
										.
									</p>
								</div>

								<div>
									<h3 className="text-lg font-semibold text-gray-200">
										Resend
									</h3>
									<p className="mt-3 text-sm leading-relaxed text-gray-400">
										We use Resend to process contact form submissions
										via email. When you submit our contact form, your
										provided information is transmitted through
										Resend&apos;s email delivery service. Resend&apos;s
										privacy policy is available at{" "}
										<a
											href="https://resend.com/legal/privacy-policy"
											target="_blank"
											rel="noopener noreferrer"
											className="text-cw-green transition-colors hover:text-cw-green-dark"
										>
											resend.com/legal/privacy-policy
										</a>
										.
									</p>
								</div>
							</div>
						</div>

						{/* Cookies */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								5. Cookies
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									Our Site uses only essential cookies required for basic
									site functionality. We do not use advertising cookies,
									tracking cookies, or third-party marketing cookies.
								</p>
								<p>
									You can configure your browser to refuse cookies or
									alert you when cookies are being sent. Note that some
									parts of the Site may not function properly without
									cookies.
								</p>
							</div>
						</div>

						{/* Data Retention */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								6. Data Retention
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									Contact form submissions are retained for as long as
									necessary to respond to your inquiry and maintain our
									business relationship. We typically retain contact
									information for up to 24 months after our last
									interaction, unless a longer retention period is
									required by law.
								</p>
								<p>
									Server logs and analytics data are retained for up to
									12 months for performance monitoring and security
									purposes.
								</p>
							</div>
						</div>

						{/* GDPR */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								7. Your Rights (GDPR)
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									If you are located in the European Economic Area (EEA),
									you have the following rights under the General Data
									Protection Regulation (GDPR):
								</p>
								<ul className="list-inside list-disc space-y-2">
									<li>
										<strong className="text-gray-300">Right of access</strong>{" "}
										&mdash; request a copy of the personal data we hold about you
									</li>
									<li>
										<strong className="text-gray-300">Right to rectification</strong>{" "}
										&mdash; request correction of inaccurate or incomplete data
									</li>
									<li>
										<strong className="text-gray-300">Right to erasure</strong>{" "}
										&mdash; request deletion of your personal data
									</li>
									<li>
										<strong className="text-gray-300">Right to restrict processing</strong>{" "}
										&mdash; request limitation of processing of your data
									</li>
									<li>
										<strong className="text-gray-300">Right to data portability</strong>{" "}
										&mdash; receive your data in a structured, machine-readable format
									</li>
									<li>
										<strong className="text-gray-300">Right to object</strong>{" "}
										&mdash; object to processing of your personal data
									</li>
								</ul>
								<p>
									To exercise any of these rights, please contact us
									using the details in Section 10 below. We will respond
									to your request within 30 days.
								</p>
							</div>
						</div>

						{/* Data Security */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								8. Data Security
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									We take appropriate technical and organizational
									measures to protect your personal information against
									unauthorized access, alteration, disclosure, or
									destruction. Our Site uses HTTPS encryption for all
									data transmission.
								</p>
								<p>
									However, no method of transmission over the Internet or
									electronic storage is 100% secure. While we strive to
									protect your personal information, we cannot guarantee
									its absolute security.
								</p>
							</div>
						</div>

						{/* Changes */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								9. Changes to This Policy
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									We may update this Privacy Policy from time to time. We
									will notify you of any changes by posting the new
									Privacy Policy on this page and updating the &ldquo;Last
									updated&rdquo; date at the top.
								</p>
								<p>
									We encourage you to review this Privacy Policy
									periodically for any changes. Changes are effective when
									posted on this page.
								</p>
							</div>
						</div>

						{/* Contact */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								10. Contact Us
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									If you have questions about this Privacy Policy or wish
									to exercise your data rights, please contact us:
								</p>
								<div className="mt-4 rounded-2xl border border-white/10 bg-white/[0.03] p-6">
									<p className="font-semibold text-gray-200">
										MB Software Studio LLC
									</p>
									<p className="mt-2">
										Email:{" "}
										<a
											href="mailto:privacy@cleanwalkerrobotics.com"
											className="text-cw-green transition-colors hover:text-cw-green-dark"
										>
											privacy@cleanwalkerrobotics.com
										</a>
									</p>
									<p className="mt-1">
										Website:{" "}
										<a
											href="https://cleanwalkerrobotics.com"
											className="text-cw-green transition-colors hover:text-cw-green-dark"
										>
											cleanwalkerrobotics.com
										</a>
									</p>
								</div>
							</div>
						</div>
					</div>
				</div>
			</section>
		</div>
	);
}
