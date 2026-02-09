// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { ButtonHTMLAttributes } from "react";

export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
	variant?: "primary" | "secondary" | "outline";
	size?: "sm" | "md" | "lg";
}

export function Button({
	variant = "primary",
	size = "md",
	className = "",
	children,
	...props
}: ButtonProps) {
	return (
		<button type="button" className={`btn btn-${variant} btn-${size} ${className}`} {...props}>
			{children}
		</button>
	);
}
