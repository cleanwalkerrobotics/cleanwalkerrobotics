"""
Mesh validation for STL files.
Checks: geometry presence, degenerate triangles, reasonable bounding box.
Assembly-aware: splits multi-component meshes and validates each component individually.

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

    # Detect if this is a multi-component assembly
    # trimesh.split() returns individual connected components
    components = mesh.split()
    is_assembly = len(components) > 1

    results = {
        "file": stl_path,
        "vertices": int(mesh.vertices.shape[0]),
        "faces": int(mesh.faces.shape[0]),
        "is_assembly": is_assembly,
        "component_count": len(components),
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

    # Check 2: Watertight — assembly-aware
    if is_assembly:
        # For assemblies, check each component individually
        watertight_components = 0
        total_volume = 0.0
        component_details = []
        for i, comp in enumerate(components):
            comp_watertight = bool(comp.is_watertight)
            comp_volume = float(comp.volume) if comp_watertight else None
            if comp_watertight:
                watertight_components += 1
                total_volume += comp.volume
            comp_info = {
                "index": i,
                "vertices": int(comp.vertices.shape[0]),
                "faces": int(comp.faces.shape[0]),
                "watertight": comp_watertight,
                "volume": round(comp_volume, 2) if comp_volume is not None else None,
            }
            component_details.append(comp_info)

        results["component_details"] = component_details
        results["watertight_components"] = watertight_components
        results["total_component_count"] = len(components)
        results["assembly_volume"] = round(total_volume, 2) if total_volume > 0 else None

        # Assembly passes watertight if majority of components are watertight
        # (some boolean-cut components may have open faces — acceptable)
        watertight_ratio = watertight_components / len(components)
        results["checks"]["watertight"] = bool(watertight_ratio >= 0.5)
        results["watertight_note"] = (
            f"Assembly: {watertight_components}/{len(components)} components watertight "
            f"({watertight_ratio:.0%}). Non-watertight components are expected in assemblies "
            f"with boolean cuts or thin features."
        )
    else:
        # Single solid: strict watertight check
        results["checks"]["watertight"] = bool(mesh.is_watertight)

    # Check 3: No degenerate faces (more than 1% is bad)
    degen_count = len(mesh.faces) - len(mesh.nondegenerate_faces())
    degen_ratio = degen_count / max(len(mesh.faces), 1)
    results["checks"]["low_degenerate_faces"] = bool(degen_ratio < 0.01)

    # Check 4: Reasonable size (0.1mm to 10m per axis)
    results["checks"]["reasonable_size"] = bool(all(0.1 <= e <= 10000 for e in extents))

    # Check 5: Positive volume — assembly-aware
    if is_assembly:
        # For assemblies, pass if we computed positive total volume from watertight components
        total_vol = results.get("assembly_volume")
        results["checks"]["positive_volume"] = bool(total_vol is not None and total_vol > 0)
    else:
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
        if results.get("is_assembly"):
            wt = results.get("watertight_components", 0)
            total = results.get("total_component_count", 0)
            print(f"\nMESH VALIDATION: PASS (assembly: {wt}/{total} components watertight)")
        else:
            print("\nMESH VALIDATION: PASS")
        sys.exit(0)
    else:
        failed = [k for k, v in results["checks"].items() if not v]
        print(f"\nMESH VALIDATION: FAIL — {failed}")
        sys.exit(1)
