// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

export default function FleetOverview() {
	return (
		<div>
			<h2 className="text-2xl font-bold text-gray-900">Fleet Overview</h2>
			<p className="mt-2 text-gray-600">
				Monitor and manage your autonomous cleaning fleet from this dashboard.
			</p>

			<div className="mt-8 grid grid-cols-1 gap-6 sm:grid-cols-2 lg:grid-cols-4">
				{[
					{ label: "Active Robots", value: "0" },
					{ label: "Zones Covered", value: "0" },
					{ label: "Tasks Today", value: "0" },
					{ label: "Uptime", value: "--" },
				].map((stat) => (
					<div key={stat.label} className="rounded-lg border border-gray-200 bg-white p-6">
						<p className="text-sm text-gray-500">{stat.label}</p>
						<p className="mt-1 text-3xl font-bold text-gray-900">{stat.value}</p>
					</div>
				))}
			</div>
		</div>
	);
}
