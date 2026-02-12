// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import Navbar from "./components/Navbar";

const inter = Inter({
	subsets: ["latin"],
	display: "swap",
	variable: "--font-inter",
});

const siteUrl = "https://cleanwalkerrobotics.com";

export const metadata: Metadata = {
	title: {
		default: "CleanWalker Robotics — Autonomous Litter Collection",
		template: "%s | CleanWalker Robotics",
	},
	description:
		"CleanWalker deploys autonomous litter-collecting robots to parks, campuses, and public spaces. 24/7 autonomous operation, AI-powered detection, zero staffing complexity.",
	keywords: [
		"autonomous robots",
		"litter collection",
		"smart city",
		"municipal robotics",
		"park cleaning",
		"autonomous litter",
		"quadrupedal robot",
		"AI litter detection",
		"fleet management",
		"public space maintenance",
	],
	metadataBase: new URL(siteUrl),
	openGraph: {
		type: "website",
		locale: "en_US",
		url: siteUrl,
		siteName: "CleanWalker Robotics",
		title: "CleanWalker Robotics — Autonomous Litter Collection",
		description:
			"Autonomous litter-collecting robots for parks, campuses, and public spaces. 24/7 operation, AI-powered detection, zero staffing complexity.",
		images: [
			{
				url: "/og-image.jpg",
				width: 1200,
				height: 630,
				alt: "CleanWalker autonomous litter-collecting robot in a sunlit park",
			},
		],
	},
	twitter: {
		card: "summary_large_image",
		title: "CleanWalker Robotics — Autonomous Litter Collection",
		description:
			"Autonomous litter-collecting robots for parks, campuses, and public spaces. 24/7 operation, AI-powered detection, zero staffing complexity.",
		images: ["/og-image.jpg"],
	},
	robots: {
		index: true,
		follow: true,
	},
};


function Footer() {
	return (
		<footer className="border-t border-white/10 bg-cw-dark">
			<div className="mx-auto max-w-7xl px-6 py-16">
				<div className="grid gap-12 md:grid-cols-4">
					{/* Brand */}
					<div className="md:col-span-1">
						<div className="flex items-center gap-2">
							<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green">
								<span className="text-sm font-bold text-white">CW</span>
							</div>
							<span className="text-lg font-bold text-white">CleanWalker</span>
						</div>
						<p className="mt-4 text-sm leading-relaxed text-gray-400">
							Autonomous litter collection robots for cleaner public spaces.
						</p>
					</div>

					{/* Product */}
					<div>
						<h3 className="mb-4 text-sm font-semibold uppercase tracking-wider text-gray-300">
							Product
						</h3>
						<ul className="space-y-3 text-sm text-gray-400">
							<li>
								<a href="/product" className="transition-colors hover:text-cw-green">
									How It Works
								</a>
							</li>
							<li>
								<a href="/demos" className="transition-colors hover:text-cw-green">
									Demos
								</a>
							</li>
							<li>
								<a href="/product#specs" className="transition-colors hover:text-cw-green">
									Specifications
								</a>
							</li>
						</ul>
					</div>

					{/* Company */}
					<div>
						<h3 className="mb-4 text-sm font-semibold uppercase tracking-wider text-gray-300">
							Company
						</h3>
						<ul className="space-y-3 text-sm text-gray-400">
							<li>
								<a href="/about" className="transition-colors hover:text-cw-green">
									About Us
								</a>
							</li>
							<li>
								<a href="/pilot" className="transition-colors hover:text-cw-green">
									Pilot Program
								</a>
							</li>
							<li>
								<a href="/contact" className="transition-colors hover:text-cw-green">
									Contact
								</a>
							</li>
							<li>
								<a href="/insights" className="transition-colors hover:text-cw-green">
									Insights
								</a>
							</li>
						</ul>
					</div>

					{/* Contact */}
					<div>
						<h3 className="mb-4 text-sm font-semibold uppercase tracking-wider text-gray-300">
							Get in Touch
						</h3>
						<ul className="space-y-3 text-sm text-gray-400">
							<li>sales@cleanwalkerrobotics.com</li>
							<li>cleanwalkerrobotics.com</li>
						</ul>
					</div>
				</div>

				<div className="mt-12 flex flex-col items-center gap-4 border-t border-white/10 pt-8 text-sm text-gray-500">
					<div className="flex gap-6">
						<a href="/privacy" className="transition-colors hover:text-cw-green">
							Privacy Policy
						</a>
						<a href="/terms" className="transition-colors hover:text-cw-green">
							Terms of Service
						</a>
					</div>
					<p>
						&copy; {new Date().getFullYear()} MB Software Studio LLC. All rights reserved.
					</p>
				</div>
			</div>
		</footer>
	);
}

export default function RootLayout({
	children,
}: Readonly<{
	children: React.ReactNode;
}>) {
	return (
		<html lang="en" className={inter.variable}>
			<body className="min-h-screen bg-cw-dark font-sans text-gray-900 antialiased">
				<a
					href="#main-content"
					className="sr-only focus:not-sr-only focus:fixed focus:left-4 focus:top-20 focus:z-[60] focus:rounded-lg focus:bg-cw-green focus:px-4 focus:py-2 focus:text-sm focus:font-semibold focus:text-white"
				>
					Skip to content
				</a>
				<Navbar />
				<main id="main-content" className="pt-16">{children}</main>
				<Footer />
			</body>
		</html>
	);
}
