// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";

export const metadata: Metadata = {
	title: "Terms of Service",
	description:
		"CleanWalker Robotics terms of service. Read our terms and conditions for using our website and services.",
	openGraph: {
		title: "Terms of Service — CleanWalker Robotics",
		description:
			"Terms and conditions for using the CleanWalker Robotics website and services.",
	},
};

export default function TermsOfServicePage() {
	return (
		<div className="bg-cw-dark">
			{/* Hero */}
			<section className="border-b border-white/10 bg-cw-dark">
				<div className="mx-auto max-w-7xl px-6 py-24 text-center">
					<h1 className="text-4xl font-bold tracking-tight text-white sm:text-5xl">
						Terms of Service
					</h1>
					<p className="mx-auto mt-6 max-w-2xl text-lg leading-relaxed text-gray-400">
						Please read these terms carefully before using our website or
						services.
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
						{/* Agreement */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								1. Agreement to Terms
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									These Terms of Service (&ldquo;Terms&rdquo;) govern your
									access to and use of the website at
									cleanwalkerrobotics.com (the &ldquo;Site&rdquo;) and any
									related services provided by MB Software Studio LLC
									(&ldquo;CleanWalker Robotics&rdquo;, &ldquo;we&rdquo;,
									&ldquo;us&rdquo;, or &ldquo;our&rdquo;).
								</p>
								<p>
									By accessing or using our Site, you agree to be bound by
									these Terms. If you do not agree to these Terms, you
									must not access or use our Site.
								</p>
							</div>
						</div>

						{/* Service Description */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								2. Description of Services
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									CleanWalker Robotics designs, manufactures, and deploys
									autonomous litter-collecting robots for municipalities,
									property management companies, universities, and other
									organizations managing public spaces.
								</p>
								<p>
									Our Site provides information about our products and
									services, interactive demonstrations, and a means to
									contact our sales team. The Site also includes
									educational content, technical specifications, and
									industry insights.
								</p>
								<p>
									Interactive demos and simulations on our Site are
									provided for informational and demonstration purposes
									only. Actual product performance may vary based on
									deployment conditions, configuration, and environmental
									factors.
								</p>
							</div>
						</div>

						{/* Use of Site */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								3. Use of the Site
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>You agree to use the Site only for lawful purposes and in accordance with these Terms. You agree not to:</p>
								<ul className="list-inside list-disc space-y-2">
									<li>Use the Site in any way that violates applicable laws or regulations</li>
									<li>Attempt to gain unauthorized access to any part of the Site or its systems</li>
									<li>Use the Site to transmit malicious software or harmful code</li>
									<li>Interfere with or disrupt the integrity or performance of the Site</li>
									<li>Scrape, data mine, or use automated systems to collect data from the Site without our prior written consent</li>
									<li>Impersonate or attempt to impersonate CleanWalker Robotics, our employees, or other users</li>
								</ul>
							</div>
						</div>

						{/* Intellectual Property */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								4. Intellectual Property
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									The Site and its entire contents, features, and
									functionality &mdash; including but not limited to text,
									graphics, logos, icons, images, audio clips, video,
									data compilations, software, 3D models, simulations,
									and interactive demos &mdash; are owned by MB Software
									Studio LLC or its licensors and are protected by
									international copyright, trademark, patent, trade
									secret, and other intellectual property laws.
								</p>
								<p>
									The CleanWalker name, logo, and all related names,
									logos, product and service names, designs, and slogans
									are trademarks of MB Software Studio LLC. You must not
									use such marks without our prior written permission.
								</p>
								<p>
									You may not reproduce, distribute, modify, create
									derivative works of, publicly display, publicly perform,
									republish, download, store, or transmit any material
									from our Site, except as follows:
								</p>
								<ul className="list-inside list-disc space-y-2">
									<li>Your browser may temporarily store copies of pages for caching purposes</li>
									<li>You may print or download one copy of a reasonable number of pages for personal, non-commercial, informational use</li>
								</ul>
							</div>
						</div>

						{/* Disclaimer */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								5. Disclaimer of Warranties
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									THE SITE IS PROVIDED ON AN &ldquo;AS IS&rdquo; AND
									&ldquo;AS AVAILABLE&rdquo; BASIS, WITHOUT WARRANTIES OF
									ANY KIND, EITHER EXPRESS OR IMPLIED. TO THE FULLEST
									EXTENT PERMITTED BY LAW, MB SOFTWARE STUDIO LLC
									DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING
									BUT NOT LIMITED TO IMPLIED WARRANTIES OF
									MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
									NON-INFRINGEMENT.
								</p>
								<p>
									We do not warrant that the Site will be uninterrupted,
									timely, secure, or error-free, or that any defects will
									be corrected. We do not warrant the accuracy,
									completeness, or usefulness of any information on the
									Site.
								</p>
							</div>
						</div>

						{/* Limitation of Liability */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								6. Limitation of Liability
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									TO THE FULLEST EXTENT PERMITTED BY APPLICABLE LAW, IN
									NO EVENT SHALL MB SOFTWARE STUDIO LLC, ITS AFFILIATES,
									DIRECTORS, EMPLOYEES, AGENTS, OR LICENSORS BE LIABLE FOR
									ANY INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL, OR
									PUNITIVE DAMAGES, INCLUDING BUT NOT LIMITED TO LOSS OF
									PROFITS, DATA, USE, GOODWILL, OR OTHER INTANGIBLE
									LOSSES, ARISING OUT OF OR IN CONNECTION WITH:
								</p>
								<ul className="list-inside list-disc space-y-2">
									<li>Your access to or use of (or inability to access or use) the Site</li>
									<li>Any conduct or content of any third party on the Site</li>
									<li>Any content obtained from the Site</li>
									<li>Unauthorized access, use, or alteration of your transmissions or content</li>
								</ul>
								<p>
									OUR TOTAL LIABILITY TO YOU FOR ALL CLAIMS ARISING OUT OF
									OR RELATING TO THE USE OF THE SITE SHALL NOT EXCEED THE
									AMOUNT YOU PAID TO US IN THE TWELVE (12) MONTHS
									PRECEDING THE CLAIM, OR ONE HUNDRED EUROS (EUR 100),
									WHICHEVER IS GREATER.
								</p>
							</div>
						</div>

						{/* Indemnification */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								7. Indemnification
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									You agree to indemnify, defend, and hold harmless MB
									Software Studio LLC and its officers, directors,
									employees, agents, and affiliates from and against any
									claims, liabilities, damages, losses, and expenses
									(including reasonable legal fees) arising out of or in
									any way connected with your access to or use of the
									Site, or your violation of these Terms.
								</p>
							</div>
						</div>

						{/* Third-Party Links */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								8. Third-Party Links
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									Our Site may contain links to third-party websites or
									services that are not owned or controlled by CleanWalker
									Robotics. We have no control over, and assume no
									responsibility for, the content, privacy policies, or
									practices of any third-party websites or services.
								</p>
								<p>
									We encourage you to review the terms and privacy
									policies of any third-party sites you visit.
								</p>
							</div>
						</div>

						{/* Governing Law */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								9. Governing Law
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									These Terms shall be governed by and construed in
									accordance with the laws of the Netherlands, without
									regard to its conflict of law provisions.
								</p>
								<p>
									Any disputes arising out of or in connection with these
									Terms shall be submitted to the exclusive jurisdiction
									of the competent courts in the Netherlands.
								</p>
								<p>
									If any provision of these Terms is held to be invalid or
									unenforceable, the remaining provisions shall continue
									in full force and effect.
								</p>
							</div>
						</div>

						{/* Changes */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								10. Changes to These Terms
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									We reserve the right to modify these Terms at any time.
									If we make material changes, we will update the
									&ldquo;Last updated&rdquo; date at the top of this page.
								</p>
								<p>
									Your continued use of the Site after changes are posted
									constitutes your acceptance of the revised Terms. We
									encourage you to review these Terms periodically.
								</p>
							</div>
						</div>

						{/* Contact */}
						<div>
							<h2 className="text-2xl font-bold tracking-tight text-white">
								11. Contact Us
							</h2>
							<div className="mt-6 space-y-4 text-sm leading-relaxed text-gray-400">
								<p>
									If you have questions about these Terms of Service,
									please contact us:
								</p>
								<div className="mt-4 rounded-2xl border border-white/10 bg-white/[0.03] p-6">
									<p className="font-semibold text-gray-200">
										MB Software Studio LLC
									</p>
									<p className="mt-2">
										Email:{" "}
										<a
											href="mailto:legal@cleanwalkerrobotics.com"
											className="text-cw-green transition-colors hover:text-cw-green-dark"
										>
											legal@cleanwalkerrobotics.com
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
