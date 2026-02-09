// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { defineConfig } from "drizzle-kit";

export default defineConfig({
	schema: "./src/schema.ts",
	out: "./drizzle",
	dialect: "postgresql",
	dbCredentials: {
		url: process.env.DATABASE_URL ?? "",
	},
});
