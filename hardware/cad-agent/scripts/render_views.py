"""
Multi-angle renderer for STL models using PyVista.
Produces 6 standard views: front, back, left, right, top, isometric.

Usage:
    python scripts/render_views.py <stl_path> <output_dir> [--debug]
"""
import sys
import os
from pathlib import Path

def render_views(stl_path: str, output_dir: str, debug: bool = False):
    import pyvista as pv

    # Headless rendering
    pv.OFF_SCREEN = True
    try:
        pv.start_xvfb()
    except Exception:
        pass  # xvfb may already be running or not needed

    os.makedirs(output_dir, exist_ok=True)
    mesh = pv.read(stl_path)

    if debug:
        print(f"Mesh loaded: {mesh.n_points} points, {mesh.n_cells} cells")
        print(f"Bounds: {mesh.bounds}")
        print(f"Center: {mesh.center}")

    # Camera positions: (position, focal_point, viewup)
    # We compute these relative to the mesh bounds
    bounds = mesh.bounds  # (xmin, xmax, ymin, ymax, zmin, zmax)
    center = mesh.center
    diag = ((bounds[1]-bounds[0])**2 + (bounds[3]-bounds[2])**2 + (bounds[5]-bounds[4])**2) ** 0.5
    dist = diag * 2.0  # Camera distance

    views = {
        "front": {
            "position": (center[0], center[1] - dist, center[2]),
            "viewup": (0, 0, 1),
        },
        "back": {
            "position": (center[0], center[1] + dist, center[2]),
            "viewup": (0, 0, 1),
        },
        "right": {
            "position": (center[0] + dist, center[1], center[2]),
            "viewup": (0, 0, 1),
        },
        "left": {
            "position": (center[0] - dist, center[1], center[2]),
            "viewup": (0, 0, 1),
        },
        "top": {
            "position": (center[0], center[1], center[2] + dist),
            "viewup": (0, 1, 0),
        },
        "isometric": {
            "position": (
                center[0] + dist * 0.577,
                center[1] - dist * 0.577,
                center[2] + dist * 0.577,
            ),
            "viewup": (0, 0, 1),
        },
    }

    for name, cam in views.items():
        plotter = pv.Plotter(off_screen=True, window_size=[1024, 1024])
        plotter.set_background("#F0F0F0")

        plotter.add_mesh(
            mesh,
            color="#4682B4",  # Steel blue
            smooth_shading=True,
            split_sharp_edges=True,
        )

        plotter.camera.position = cam["position"]
        plotter.camera.focal_point = center
        plotter.camera.up = cam["viewup"]
        plotter.reset_camera()

        # Add orientation axes in corner
        plotter.add_axes(
            interactive=False,
            line_width=2,
            labels_off=False,
        )

        out_path = os.path.join(output_dir, f"{name}.png")
        plotter.screenshot(out_path)
        plotter.close()

        size = os.path.getsize(out_path)
        if debug:
            print(f"  {name}.png — {size} bytes")

        if size == 0:
            print(f"WARNING: {name}.png is empty!", file=sys.stderr)

    print(f"Rendered 6 views to {output_dir}/")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python render_views.py <stl_path> <output_dir> [--debug]")
        sys.exit(1)

    stl_path = sys.argv[1]
    output_dir = sys.argv[2]
    debug = "--debug" in sys.argv

    if not os.path.exists(stl_path):
        print(f"Error: {stl_path} not found")
        sys.exit(1)

    render_views(stl_path, output_dir, debug)
