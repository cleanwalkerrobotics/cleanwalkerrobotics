# Copyright (c) 2026 MB Software Studio LLC. All rights reserved.
# SPDX-License-Identifier: AGPL-3.0

"""
Convert CleanWalker CW-1 URDF to USD format for Isaac Sim.

Usage (standalone — requires Isaac Sim Python environment):
    # From IsaacLab root:
    ./isaaclab.sh -p <repo>/ml/locomotion/convert_urdf.py

    # Or directly if Isaac Sim is on PYTHONPATH:
    python ml/locomotion/convert_urdf.py

The script converts hardware/urdf/cleanwalker-cw1/cleanwalker_cw1.urdf
into ml/locomotion/assets/cleanwalker_cw1.usd with instanceable meshes
for efficient parallel simulation (4096 envs).

After conversion, train.py automatically detects and uses the USD file.
"""

from __future__ import annotations

import argparse
import os
import sys


def find_repo_root() -> str:
    """Walk up from this file to find the repo root (contains hardware/)."""
    path = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(path, "hardware")):
            return path
        path = os.path.dirname(path)
    raise FileNotFoundError("Cannot find repo root (no hardware/ directory found)")


def convert(
    urdf_path: str,
    usd_path: str,
    merge_joints: bool = True,
    make_instanceable: bool = True,
    fix_base: bool = False,
) -> str:
    """Convert URDF to USD using Isaac Sim's converter.

    Args:
        urdf_path: Path to input .urdf file.
        usd_path: Path to output .usd file.
        merge_joints: Merge fixed joints into parent links.
        make_instanceable: Create instanceable meshes for parallel envs.
        fix_base: Fix the base link (True for manipulation, False for locomotion).

    Returns:
        Absolute path to the generated .usd file.
    """
    try:
        import isaacsim  # noqa: F401 — triggers Isaac Sim app startup
    except ImportError:
        print(
            "ERROR: Isaac Sim Python packages not found.\n"
            "Install via: pip install isaacsim-rl isaacsim-replicator "
            "isaacsim-extscache-physics isaacsim-extscache-kit-sdk\n"
            "Or run through IsaacLab: ./isaaclab.sh -p convert_urdf.py"
        )
        sys.exit(1)

    from omni.isaac.kit import SimulationApp

    # Start headless Isaac Sim application
    sim_app = SimulationApp({"headless": True})

    from omni.isaac.core.utils.extensions import enable_extension

    enable_extension("omni.importer.urdf")

    from omni.importer.urdf import _urdf as urdf_importer

    # Configure URDF import settings
    import_config = urdf_importer.ImportConfig()
    import_config.merge_fixed_joints = merge_joints
    import_config.make_instanceable = make_instanceable
    import_config.fix_base = fix_base
    import_config.import_inertia_tensor = True
    import_config.self_collision = True
    import_config.default_drive_type = urdf_importer.UrdfJointTargetType.JOINT_DRIVE_NONE
    import_config.default_drive_strength = 0.0
    import_config.default_position_drive_damping = 0.0
    import_config.replace_cylinders_with_capsules = True
    import_config.convex_decomp = False

    # Convert
    urdf_path = os.path.abspath(urdf_path)
    usd_path = os.path.abspath(usd_path)

    os.makedirs(os.path.dirname(usd_path), exist_ok=True)

    print(f"Converting URDF: {urdf_path}")
    print(f"Output USD:      {usd_path}")
    print(f"  merge_joints:      {merge_joints}")
    print(f"  make_instanceable: {make_instanceable}")
    print(f"  fix_base:          {fix_base}")

    result = urdf_importer.import_robot(
        urdf_path,
        import_config,
        usd_path,
    )

    if result:
        print(f"\nConversion successful: {usd_path}")
        file_size = os.path.getsize(usd_path)
        print(f"File size: {file_size / 1024:.1f} KB")
    else:
        print("\nERROR: URDF conversion failed.")
        sim_app.close()
        sys.exit(1)

    sim_app.close()
    return usd_path


def main():
    parser = argparse.ArgumentParser(
        description="Convert CleanWalker CW-1 URDF to USD for Isaac Sim"
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Path to URDF file (default: auto-detect from repo)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output USD path (default: ml/locomotion/assets/cleanwalker_cw1.usd)",
    )
    parser.add_argument(
        "--no-merge-joints",
        action="store_true",
        help="Don't merge fixed joints",
    )
    parser.add_argument(
        "--no-instanceable",
        action="store_true",
        help="Don't create instanceable meshes",
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="Fix the base link (for debugging, not locomotion)",
    )
    args = parser.parse_args()

    repo_root = find_repo_root()

    urdf_path = args.urdf or os.path.join(
        repo_root, "hardware", "urdf", "cleanwalker-cw1", "cleanwalker_cw1.urdf"
    )
    usd_path = args.output or os.path.join(
        repo_root, "ml", "locomotion", "assets", "cleanwalker_cw1.usd"
    )

    if not os.path.isfile(urdf_path):
        print(f"ERROR: URDF not found: {urdf_path}")
        sys.exit(1)

    convert(
        urdf_path=urdf_path,
        usd_path=usd_path,
        merge_joints=not args.no_merge_joints,
        make_instanceable=not args.no_instanceable,
        fix_base=args.fix_base,
    )


if __name__ == "__main__":
    main()
