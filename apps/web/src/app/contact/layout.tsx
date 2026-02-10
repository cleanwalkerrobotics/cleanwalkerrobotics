// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { Metadata } from "next";

export const metadata: Metadata = {
	title: "Contact Sales",
	description:
		"Schedule a consultation with CleanWalker Robotics. Get a site assessment, custom deployment plan, and see how autonomous litter collection can transform your facility.",
	openGraph: {
		title: "Contact Sales — CleanWalker Robotics",
		description:
			"Schedule a consultation and site assessment. See how autonomous litter collection robots can transform your facility.",
	},
};

export default function ContactLayout({
	children,
}: {
	children: React.ReactNode;
}) {
	return children;
}
