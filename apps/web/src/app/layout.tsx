// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
	title: "CleanWalker Robotics",
	description: "Autonomous Street Cleaning Robots",
};

export default function RootLayout({
	children,
}: Readonly<{
	children: React.ReactNode;
}>) {
	return (
		<html lang="en">
			<body className="min-h-screen bg-white text-gray-900 antialiased">{children}</body>
		</html>
	);
}
