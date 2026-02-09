// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { MiddlewareHandler } from "hono";

export const logger: MiddlewareHandler = async (c, next) => {
	const start = Date.now();
	const method = c.req.method;
	const path = c.req.path;

	await next();

	const duration = Date.now() - start;
	const status = c.res.status;
	console.log(`${method} ${path} ${status} ${duration}ms`);
};
