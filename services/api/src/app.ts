// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { Hono } from "hono";
import { logger } from "./middleware/logger.js";
import { fleet, health } from "./routes/index.js";

const app = new Hono();

app.use("*", logger);

app.route("/health", health);
app.route("/fleet", fleet);

export { app };
