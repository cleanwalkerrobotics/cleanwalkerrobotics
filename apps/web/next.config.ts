// Copyright (c) MB Software Studio LLC. All rights reserved.
// Licensed under the AGPL-3.0 License. See LICENSE in the project root.

import type { NextConfig } from "next";

const nextConfig: NextConfig = {
	webpack: (config) => {
		config.resolve.alias = {
			...config.resolve.alias,
			"onnxruntime-node$": false,
			sharp$: false,
		};
		return config;
	},
};

export default nextConfig;
