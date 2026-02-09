// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { Hono } from "hono";

const fleet = new Hono();

fleet.get("/", (c) => {
	return c.json({ data: [], pagination: { page: 1, pageSize: 20, total: 0, totalPages: 0 } });
});

fleet.get("/:id", (c) => {
	const id = c.req.param("id");
	return c.json({ data: null, error: `Robot ${id} not found` }, 404);
});

fleet.post("/", (c) => {
	return c.json({ data: null, message: "Not implemented" }, 501);
});

fleet.put("/:id", (c) => {
	return c.json({ data: null, message: "Not implemented" }, 501);
});

fleet.delete("/:id", (c) => {
	return c.json({ data: null, message: "Not implemented" }, 501);
});

export { fleet };
