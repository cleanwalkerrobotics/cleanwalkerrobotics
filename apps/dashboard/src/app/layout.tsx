// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
	title: "CleanWalker Dashboard",
	description: "Fleet Management Dashboard",
};

const navItems = [
	{ label: "Fleet", href: "/" },
	{ label: "Zones", href: "/zones" },
	{ label: "Reports", href: "/reports" },
	{ label: "Settings", href: "/settings" },
];

export default function RootLayout({
	children,
}: Readonly<{
	children: React.ReactNode;
}>) {
	return (
		<html lang="en">
			<body className="min-h-screen bg-gray-50 text-gray-900 antialiased">
				<div className="flex min-h-screen">
					{/* Sidebar */}
					<aside className="flex w-64 flex-col border-r border-gray-200 bg-white">
						<div className="border-b border-gray-200 px-6 py-4">
							<span className="text-lg font-bold">CleanWalker</span>
						</div>
						<nav className="flex-1 px-4 py-4">
							{navItems.map((item) => (
								<a
									key={item.href}
									href={item.href}
									className="block rounded-md px-3 py-2 text-sm text-gray-700 hover:bg-gray-100 hover:text-gray-900"
								>
									{item.label}
								</a>
							))}
						</nav>
					</aside>

					{/* Main area */}
					<div className="flex flex-1 flex-col">
						{/* Top bar */}
						<header className="border-b border-gray-200 bg-white px-6 py-4">
							<h1 className="text-lg font-semibold">CleanWalker Dashboard</h1>
						</header>

						{/* Content */}
						<main className="flex-1 p-6">{children}</main>
					</div>
				</div>
			</body>
		</html>
	);
}
