import type { Group, MeshStandardMaterial } from "three";

export type ViewMode = "full" | "body" | "bag";

export interface LegRef {
	hipPitch: Group;
	knee: Group;
	baseHipPitch: number;
	baseKneePitch: number;
	phase: number;
}

export interface ArmRef {
	shoulderPitch: Group;
	elbowPitch: Group;
	baseShoulderPitch: number;
	baseElbowPitch: number;
}

export interface BodyResult {
	group: Group;
	legs: LegRef[];
	arm: ArmRef;
	ledMaterial: MeshStandardMaterial;
}

export interface BagResult {
	group: Group;
	hingeGroup: Group;
	update: (time: number) => void;
	play: () => void;
	readonly playing: boolean;
}
