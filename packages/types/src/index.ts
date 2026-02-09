// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

export enum RobotStatus {
	Idle = "idle",
	Patrolling = "patrolling",
	Charging = "charging",
	Maintenance = "maintenance",
}

export enum ZoneStatus {
	Active = "active",
	Inactive = "inactive",
}

export interface Robot {
	id: string;
	name: string;
	status: RobotStatus;
	batteryLevel: number;
	zoneId: string | null;
	lastCheckIn: Date;
	createdAt: Date;
	updatedAt: Date;
}

export interface Zone {
	id: string;
	name: string;
	boundary: Record<string, unknown>;
	status: ZoneStatus;
	createdAt: Date;
}

export interface Collection {
	id: string;
	robotId: string;
	zoneId: string;
	weightKg: number;
	startedAt: Date;
	completedAt: Date | null;
	imageBefore: string | null;
	imageAfter: string | null;
}

export interface ApiResponse<T> {
	success: boolean;
	data: T;
	error?: string;
}

export interface PaginatedResponse<T> {
	success: boolean;
	data: T[];
	pagination: {
		page: number;
		pageSize: number;
		total: number;
		totalPages: number;
	};
}
