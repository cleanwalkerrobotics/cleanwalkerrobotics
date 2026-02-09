// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import { jsonb, numeric, pgEnum, pgTable, text, timestamp, uuid } from "drizzle-orm/pg-core";

export const robotStatusEnum = pgEnum("robot_status", [
	"idle",
	"patrolling",
	"charging",
	"maintenance",
]);

export const zoneStatusEnum = pgEnum("zone_status", ["active", "inactive"]);

export const robots = pgTable("robots", {
	id: uuid("id").primaryKey().defaultRandom(),
	name: text("name").notNull(),
	status: robotStatusEnum("status").notNull().default("idle"),
	batteryLevel: numeric("battery_level", { precision: 5, scale: 2 }).notNull().default("100"),
	zoneId: uuid("zone_id").references(() => zones.id),
	lastCheckIn: timestamp("last_check_in", { withTimezone: true }).defaultNow(),
	createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
	updatedAt: timestamp("updated_at", { withTimezone: true }).notNull().defaultNow(),
});

export const zones = pgTable("zones", {
	id: uuid("id").primaryKey().defaultRandom(),
	name: text("name").notNull(),
	boundary: jsonb("boundary").notNull().$type<Record<string, unknown>>(),
	status: zoneStatusEnum("status").notNull().default("active"),
	createdAt: timestamp("created_at", { withTimezone: true }).notNull().defaultNow(),
});

export const collections = pgTable("collections", {
	id: uuid("id").primaryKey().defaultRandom(),
	robotId: uuid("robot_id")
		.notNull()
		.references(() => robots.id),
	zoneId: uuid("zone_id")
		.notNull()
		.references(() => zones.id),
	weightKg: numeric("weight_kg", { precision: 8, scale: 2 }).notNull(),
	startedAt: timestamp("started_at", { withTimezone: true }).notNull().defaultNow(),
	completedAt: timestamp("completed_at", { withTimezone: true }),
	imageBefore: text("image_before"),
	imageAfter: text("image_after"),
});
