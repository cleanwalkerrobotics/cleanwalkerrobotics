"""
Mesh validation for STL files.
Checks: watertight, no degenerate triangles, reasonable bounding box.

Usage:
    python scripts/validate_mesh.py <stl_path>

Exit code 0 = PASS, 1 = FAIL
"""
import sys
import json

def validate_mesh(stl_path: str) -> dict:
    import trimesh

    mesh = trimesh.load(stl_path)

    bounds = mesh.bounds  # [[xmin,ymin,zmin],[xmax,ymax,zmax]]
    extents = mesh.extents  # [dx, dy, dz]

    results = {
        "file": stl_path,
        "vertices": int(mesh.vertices.shape[0]),
        "faces": int(mesh.faces.shape[0]),
        "watertight": bool(mesh.is_watertight),
        "volume": float(mesh.volume) if mesh.is_watertight else None,
        "bounding_box_mm": {
            "x": round(float(extents[0]), 2),
            "y": round(float(extents[1]), 2),
            "z": round(float(extents[2]), 2),
        },
        "center_of_mass": [round(float(c), 2) for c in mesh.center_mass] if mesh.is_watertight else None,
        "has_degenerate_faces": bool(len(mesh.nondegenerate_faces()) < len(mesh.faces)),
        "degenerate_face_count": int(len(mesh.faces) - len(mesh.nondegenerate_faces())),
        "checks": {},
    }

    # Check 1: Non-trivial geometry
    results["checks"]["has_geometry"] = bool(mesh.vertices.shape[0] > 3)

    # Check 2: Watertight
    results["checks"]["watertight"] = bool(mesh.is_watertight)

    # Check 3: No degenerate faces (more than 1% is bad)
    degen_count = len(mesh.faces) - len(mesh.nondegenerate_faces())
    degen_ratio = degen_count / max(len(mesh.faces), 1)
    results["checks"]["low_degenerate_faces"] = bool(degen_ratio < 0.01)

    # Check 4: Reasonable size (0.1mm to 10m per axis)
    results["checks"]["reasonable_size"] = bool(all(0.1 <= e <= 10000 for e in extents))

    # Check 5: Positive volume (if watertight)
    if mesh.is_watertight:
        results["checks"]["positive_volume"] = bool(mesh.volume > 0)
    else:
        results["checks"]["positive_volume"] = False

    # Overall
    results["passed"] = all(results["checks"].values())

    return results


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python validate_mesh.py <stl_path>")
        sys.exit(1)

    import os
    stl_path = sys.argv[1]
    if not os.path.exists(stl_path):
        print(f"Error: {stl_path} not found")
        sys.exit(1)

    results = validate_mesh(stl_path)
    print(json.dumps(results, indent=2))

    if results["passed"]:
        print("\n✅ MESH VALIDATION: PASS")
        sys.exit(0)
    else:
        failed = [k for k, v in results["checks"].items() if not v]
        print(f"\n❌ MESH VALIDATION: FAIL — {failed}")
        sys.exit(1)
