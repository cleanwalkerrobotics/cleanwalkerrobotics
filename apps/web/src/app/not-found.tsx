// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import Link from "next/link";

export default function NotFound() {
	return (
		<div className="flex min-h-[80vh] items-center justify-center bg-cw-dark px-6">
			<div className="max-w-lg text-center">
				<div className="mx-auto mb-6 flex h-20 w-20 items-center justify-center rounded-full bg-cw-green/10">
					<span className="text-4xl font-bold text-cw-green">404</span>
				</div>
				<h1 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
					Page Not Found
				</h1>
				<p className="mt-4 text-lg leading-relaxed text-gray-400">
					The page you&apos;re looking for doesn&apos;t exist or has been moved.
				</p>
				<div className="mt-10 flex flex-col items-center gap-4 sm:flex-row sm:justify-center">
					<Link
						href="/"
						className="rounded-lg bg-cw-green px-8 py-3 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Back to Home
					</Link>
					<Link
						href="/contact"
						className="rounded-lg border border-white/20 bg-white/5 px-8 py-3 text-sm font-semibold text-white transition-colors hover:border-white/40 hover:bg-white/10"
					>
						Contact Sales
					</Link>
				</div>
				<div className="mt-12 border-t border-white/10 pt-8">
					<p className="mb-4 text-sm font-medium text-gray-500">
						Useful links
					</p>
					<div className="flex flex-wrap justify-center gap-6 text-sm text-gray-400">
						<Link href="/product" className="transition-colors hover:text-cw-green">
							Product
						</Link>
						<Link href="/demos" className="transition-colors hover:text-cw-green">
							Demos
						</Link>
						<Link href="/pilot" className="transition-colors hover:text-cw-green">
							Pilot Program
						</Link>
						<Link href="/about" className="transition-colors hover:text-cw-green">
							About
						</Link>
					</div>
				</div>
			</div>
		</div>
	);
}
