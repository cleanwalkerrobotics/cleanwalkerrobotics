// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

import { useState } from "react";

const navLinks = [
	{ href: "/product", label: "Product" },
	{ href: "/pilot", label: "Pilot Program" },
	{ href: "/demos", label: "Demos" },
	{ href: "/about", label: "About" },
];

export default function Navbar() {
	const [menuOpen, setMenuOpen] = useState(false);

	return (
		<nav className="fixed top-0 z-50 w-full border-b border-white/10 bg-cw-dark/95 backdrop-blur-sm">
			<div className="mx-auto flex max-w-7xl items-center justify-between px-6 py-4">
				<a href="/" className="flex items-center gap-2">
					<div className="flex h-8 w-8 items-center justify-center rounded-lg bg-cw-green">
						<span className="text-sm font-bold text-white">CW</span>
					</div>
					<span className="text-lg font-bold text-white">CleanWalker</span>
				</a>

				{/* Desktop nav */}
				<div className="hidden items-center gap-8 md:flex">
					{navLinks.map((link) => (
						<a
							key={link.href}
							href={link.href}
							className="text-sm text-gray-300 transition-colors hover:text-cw-green"
						>
							{link.label}
						</a>
					))}
					<a
						href="/contact"
						className="rounded-lg bg-cw-green px-5 py-2 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Contact Sales
					</a>
				</div>

				{/* Mobile: hamburger + CTA */}
				<div className="flex items-center gap-3 md:hidden">
					<a
						href="/contact"
						className="rounded-lg bg-cw-green px-4 py-2 text-sm font-semibold text-white"
					>
						Contact
					</a>
					<button
						type="button"
						onClick={() => setMenuOpen(!menuOpen)}
						className="flex h-10 w-10 items-center justify-center rounded-lg border border-white/10 text-gray-300 transition-colors hover:border-white/20 hover:text-white"
						aria-label={menuOpen ? "Close menu" : "Open menu"}
						aria-expanded={menuOpen}
					>
						{menuOpen ? (
							<svg className="h-5 w-5" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
								<path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
							</svg>
						) : (
							<svg className="h-5 w-5" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
								<path strokeLinecap="round" strokeLinejoin="round" d="M3.75 6.75h16.5M3.75 12h16.5m-16.5 5.25h16.5" />
							</svg>
						)}
					</button>
				</div>
			</div>

			{/* Mobile dropdown menu */}
			{menuOpen && (
				<div className="border-t border-white/10 bg-cw-dark/98 px-6 pb-6 pt-4 md:hidden">
					<div className="flex flex-col gap-1">
						{navLinks.map((link) => (
							<a
								key={link.href}
								href={link.href}
								className="rounded-lg px-4 py-3 text-sm text-gray-300 transition-colors hover:bg-white/5 hover:text-cw-green"
								onClick={() => setMenuOpen(false)}
							>
								{link.label}
							</a>
						))}
						<a
							href="/contact"
							className="mt-2 rounded-lg bg-cw-green px-4 py-3 text-center text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
							onClick={() => setMenuOpen(false)}
						>
							Contact Sales
						</a>
					</div>
				</div>
			)}
		</nav>
	);
}
