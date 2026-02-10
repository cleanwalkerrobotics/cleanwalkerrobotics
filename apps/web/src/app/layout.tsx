// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
	title: "CleanWalker Robotics — Autonomous Litter Collection",
	description:
		"CleanWalker deploys autonomous litter-collecting robots to parks, campuses, and public spaces. Autonomous litter collection for municipalities and property managers.",
};

function Navbar() {
	return (
		<nav className="fixed top-0 z-50 w-full border-b border-white/10 bg-cw-dark/95 backdrop-blur-sm">
			<div className="mx-auto flex max-w-7xl items-center justify-between px-6 py-4">
				<a href="/" className="flex items-center gap-2">
					<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green">
						<span className="text-sm font-bold text-white">CW</span>
					</div>
					<span className="text-lg font-bold text-white">CleanWalker</span>
				</a>
				<div className="hidden items-center gap-8 md:flex">
					<a
						href="/product"
						className="text-sm text-gray-300 transition-colors hover:text-cw-green"
					>
						Product
					</a>
					<a
						href="/demos"
						className="text-sm text-gray-300 transition-colors hover:text-cw-green"
					>
						Demos
					</a>
					<a
						href="/about"
						className="text-sm text-gray-300 transition-colors hover:text-cw-green"
					>
						About
					</a>
					<a
						href="/contact"
						className="rounded-lg bg-cw-green px-5 py-2 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
				</div>
				{/* Mobile: show key links */}
				<div className="flex items-center gap-4 md:hidden">
					<a href="/product" className="text-sm text-gray-300">
						Product
					</a>
					<a
						href="/contact"
						className="rounded-lg bg-cw-green px-4 py-2 text-sm font-semibold text-white"
					>
						Contact Sales
					</a>
				</div>
			</div>
		</nav>
	);
}

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
								<a href="/contact" className="transition-colors hover:text-cw-green">
									Contact
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

				<div className="mt-12 border-t border-white/10 pt-8 text-center text-sm text-gray-500">
					&copy; {new Date().getFullYear()} MB Software Studio LLC. All rights reserved.
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
		<html lang="en">
			<body className="min-h-screen bg-cw-dark text-gray-900 antialiased">
				<Navbar />
				<main className="pt-16">{children}</main>
				<Footer />
			</body>
		</html>
	);
}
