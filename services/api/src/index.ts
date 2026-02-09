// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { serve } from "@hono/node-server";
import { app } from "./app.js";

const port = Number(process.env.PORT) || 3001;

serve({ fetch: app.fetch, port }, (info) => {
	console.log(`CleanWalker API running on http://localhost:${info.port}`);
});
