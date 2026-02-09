// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { HTMLAttributes } from "react";

export interface CardProps extends HTMLAttributes<HTMLDivElement> {
	title?: string;
}

export function Card({ title, className = "", children, ...props }: CardProps) {
	return (
		<div className={`card ${className}`} {...props}>
			{title && <h3 className="card-title">{title}</h3>}
			<div className="card-content">{children}</div>
		</div>
	);
}
