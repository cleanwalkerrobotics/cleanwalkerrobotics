// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

"use client";

export default function Error({
	reset,
}: {
	error: Error & { digest?: string };
	reset: () => void;
}) {
	return (
		<div className="flex min-h-[80vh] items-center justify-center bg-cw-dark px-6">
			<div className="max-w-lg text-center">
				<div className="mx-auto mb-6 flex h-20 w-20 items-center justify-center rounded-full bg-red-500/10">
					<svg
						className="h-10 w-10 text-red-400"
						fill="none"
						viewBox="0 0 24 24"
						stroke="currentColor"
						strokeWidth={1.5}
					>
						<path
							strokeLinecap="round"
							strokeLinejoin="round"
							d="M12 9v3.75m-9.303 3.376c-.866 1.5.217 3.374 1.948 3.374h14.71c1.73 0 2.813-1.874 1.948-3.374L13.949 3.378c-.866-1.5-3.032-1.5-3.898 0L2.697 16.126zM12 15.75h.007v.008H12v-.008z"
						/>
					</svg>
				</div>
				<h1 className="text-3xl font-bold tracking-tight text-white md:text-4xl">
					Something Went Wrong
				</h1>
				<p className="mt-4 text-lg leading-relaxed text-gray-400">
					An unexpected error occurred. Please try again or return to the
					homepage.
				</p>
				<div className="mt-10 flex flex-col items-center gap-4 sm:flex-row sm:justify-center">
					<button
						onClick={reset}
						className="rounded-lg bg-cw-green px-8 py-3 text-sm font-semibold text-white transition-colors hover:bg-cw-green-dark"
					>
						Try Again
					</button>
					<a
						href="/"
						className="rounded-lg border border-white/20 bg-white/5 px-8 py-3 text-sm font-semibold text-white transition-colors hover:border-white/40 hover:bg-white/10"
					>
						Back to Home
					</a>
				</div>
			</div>
		</div>
	);
}
