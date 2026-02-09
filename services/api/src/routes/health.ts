// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { Hono } from "hono";

const health = new Hono();

health.get("/", (c) => {
	return c.json({ status: "ok" });
});

export { health };
