// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

export default function Home() {
	return (
		<div className="flex min-h-screen flex-col">
			{/* Navigation */}
			<nav className="border-b border-gray-200 px-6 py-4">
				<div className="mx-auto flex max-w-6xl items-center justify-between">
					<span className="text-xl font-bold text-gray-900">CleanWalker</span>
					<div className="flex gap-6 text-sm text-gray-600">
						<a href="/product" className="hover:text-gray-900">
							Product
						</a>
						<a href="/about" className="hover:text-gray-900">
							About
						</a>
						<a href="/contact" className="hover:text-gray-900">
							Contact
						</a>
					</div>
				</div>
			</nav>

			{/* Hero */}
			<main className="flex flex-1 flex-col items-center justify-center px-6 py-24">
				<h1 className="max-w-3xl text-center text-5xl font-bold tracking-tight text-gray-900">
					Autonomous Street Cleaning Robots
				</h1>
				<p className="mt-6 max-w-xl text-center text-lg text-gray-600">
					CleanWalker deploys intelligent, autonomous robots that keep city streets clean around the
					clock — quietly, efficiently, and sustainably.
				</p>
				<div className="mt-10 flex gap-4">
					<a
						href="/contact"
						className="rounded-lg bg-gray-900 px-6 py-3 text-sm font-medium text-white hover:bg-gray-800"
					>
						Learn More
					</a>
					<a
						href="/contact"
						className="rounded-lg border border-gray-300 px-6 py-3 text-sm font-medium text-gray-700 hover:bg-gray-50"
					>
						Request Demo
					</a>
				</div>
			</main>

			{/* Footer */}
			<footer className="border-t border-gray-200 px-6 py-6">
				<div className="mx-auto max-w-6xl text-center text-sm text-gray-500">
					&copy; {new Date().getFullYear()} MB Software Studio LLC. All rights reserved.
				</div>
			</footer>
		</div>
	);
}
