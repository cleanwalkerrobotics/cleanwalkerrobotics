#!/usr/bin/env python3
"""
blender-render.py — Render CleanWalker CW-1 reference perspective image

Usage:
    blender --background --python scripts/blender-render.py

Imports public/models/cleanwalker-cw1.glb, sets up studio lighting and
a 3/4 perspective camera, renders at 2048x2048 with Cycles (CPU).
Output: public/renders/v3/reference-perspective.png
"""

import bpy
import os
import sys
import math

# ── Paths ──────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
GLB_PATH = os.path.join(ROOT, "public", "models", "cleanwalker-cw1.glb")
OUT_DIR = os.path.join(ROOT, "public", "renders", "v3")
OUT_PATH = os.path.join(OUT_DIR, "reference-perspective.png")

os.makedirs(OUT_DIR, exist_ok=True)

# ── Clean default scene ────────────────────────────────
bpy.ops.wm.read_factory_settings(use_empty=True)

# ── Import GLB ─────────────────────────────────────────
print(f"Importing GLB: {GLB_PATH}")
if not os.path.exists(GLB_PATH):
    print(f"ERROR: GLB not found at {GLB_PATH}")
    sys.exit(1)

bpy.ops.import_scene.gltf(filepath=GLB_PATH)

# Get imported objects
imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
print(f"Imported {len(imported_objects)} mesh objects")

# ── Center and measure the robot ───────────────────────
# Find bounding box of all imported objects
min_coords = [float("inf")] * 3
max_coords = [float("-inf")] * 3

for obj in imported_objects:
    for corner in obj.bound_box:
        world_corner = obj.matrix_world @ bpy.app.driver_namespace.get("Vector", __import__("mathutils").Vector)(corner)
        for i in range(3):
            min_coords[i] = min(min_coords[i], world_corner[i])
            max_coords[i] = max(max_coords[i], world_corner[i])

center = [(min_coords[i] + max_coords[i]) / 2 for i in range(3)]
size = [max_coords[i] - min_coords[i] for i in range(3)]
max_dim = max(size)

print(f"Robot bounding box: {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f}")
print(f"Robot center: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")

# ── Camera setup ───────────────────────────────────────
# 3/4 perspective: front-left, slightly above
cam_data = bpy.data.cameras.new("MainCamera")
cam_data.type = "PERSP"
cam_data.lens = 50  # 50mm focal length for natural perspective
cam_data.clip_start = 0.01
cam_data.clip_end = 100

cam_obj = bpy.data.objects.new("MainCamera", cam_data)
bpy.context.scene.collection.objects.link(cam_obj)
bpy.context.scene.camera = cam_obj

# Position camera at front-left, above
# Robot is roughly 0.6m long, 0.3m wide, 0.5m tall in Y-up
# Camera distance ~1.4m for good framing at 2048x2048
cam_distance = max_dim * 2.2
cam_angle_h = math.radians(35)   # 35 degrees from front (front-left view)
cam_angle_v = math.radians(25)   # 25 degrees above horizontal

cam_x = center[0] + cam_distance * math.cos(cam_angle_v) * math.sin(cam_angle_h)
cam_y = center[1] - cam_distance * math.cos(cam_angle_v) * math.cos(cam_angle_h)
cam_z = center[2] + cam_distance * math.sin(cam_angle_v)

cam_obj.location = (cam_x, cam_y, cam_z)

# Point camera at robot center (slightly above geometric center for visual appeal)
look_target = (center[0], center[1], center[2] + size[2] * 0.1)

# Calculate rotation to look at target
from mathutils import Vector, Matrix
direction = Vector(look_target) - Vector(cam_obj.location)
rot_quat = direction.to_track_quat("-Z", "Y")
cam_obj.rotation_euler = rot_quat.to_euler()

print(f"Camera at ({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f})")

# ── Studio lighting (3-point) ─────────────────────────

# 1. Key light — warm, slightly above and to the right
key_data = bpy.data.lights.new("KeyLight", "AREA")
key_data.energy = 150
key_data.color = (1.0, 0.97, 0.92)  # Warm white
key_data.size = 1.0
key_obj = bpy.data.objects.new("KeyLight", key_data)
key_obj.location = (
    center[0] + max_dim * 1.5,
    center[1] - max_dim * 1.2,
    center[2] + max_dim * 1.8,
)
bpy.context.scene.collection.objects.link(key_obj)
# Point at robot
constraint = key_obj.constraints.new("TRACK_TO")
constraint.target = imported_objects[0] if imported_objects else None
constraint.track_axis = "TRACK_NEGATIVE_Z"
constraint.up_axis = "UP_Y"

# 2. Fill light — softer, opposite side
fill_data = bpy.data.lights.new("FillLight", "AREA")
fill_data.energy = 60
fill_data.color = (0.85, 0.9, 1.0)  # Cool fill
fill_data.size = 1.5
fill_obj = bpy.data.objects.new("FillLight", fill_data)
fill_obj.location = (
    center[0] - max_dim * 1.2,
    center[1] + max_dim * 1.0,
    center[2] + max_dim * 1.0,
)
bpy.context.scene.collection.objects.link(fill_obj)

# 3. Rim/back light — highlights edges from behind
rim_data = bpy.data.lights.new("RimLight", "AREA")
rim_data.energy = 100
rim_data.color = (0.9, 0.95, 1.0)  # Slightly cool
rim_data.size = 0.8
rim_obj = bpy.data.objects.new("RimLight", rim_data)
rim_obj.location = (
    center[0] - max_dim * 1.5,
    center[1] + max_dim * 0.5,
    center[2] + max_dim * 2.0,
)
bpy.context.scene.collection.objects.link(rim_obj)

# 4. Bottom fill — subtle, prevents harsh shadows underneath
bottom_data = bpy.data.lights.new("BottomFill", "AREA")
bottom_data.energy = 25
bottom_data.color = (1.0, 1.0, 1.0)
bottom_data.size = 2.0
bottom_obj = bpy.data.objects.new("BottomFill", bottom_data)
bottom_obj.location = (center[0], center[1], center[2] - max_dim * 0.8)
bottom_obj.rotation_euler = (math.pi, 0, 0)  # Point up
bpy.context.scene.collection.objects.link(bottom_obj)

# ── Ground plane (light gray studio floor) ─────────────
bpy.ops.mesh.primitive_plane_add(size=20, location=(center[0], center[1], min_coords[2] - 0.001))
ground = bpy.context.active_object
ground.name = "StudioFloor"

# Create ground material
ground_mat = bpy.data.materials.new("GroundMaterial")
ground_mat.use_nodes = True
nodes = ground_mat.node_tree.nodes
bsdf = nodes.get("Principled BSDF")
if bsdf:
    bsdf.inputs["Base Color"].default_value = (0.85, 0.85, 0.85, 1.0)  # Light gray
    bsdf.inputs["Roughness"].default_value = 0.9
    bsdf.inputs["Metallic"].default_value = 0.0
ground.data.materials.append(ground_mat)

# ── World background (light gradient) ─────────────────
world = bpy.data.worlds.new("StudioWorld")
bpy.context.scene.world = world
world.use_nodes = True
world_nodes = world.node_tree.nodes
world_links = world.node_tree.links

# Clear default nodes
for node in world_nodes:
    world_nodes.remove(node)

# Create gradient background: light gray to white
bg_node = world_nodes.new("ShaderNodeBackground")
bg_node.inputs["Color"].default_value = (0.92, 0.92, 0.94, 1.0)  # Very light blue-gray
bg_node.inputs["Strength"].default_value = 0.8

output_node = world_nodes.new("ShaderNodeOutputWorld")
world_links.new(bg_node.outputs["Background"], output_node.inputs["Surface"])

# ── Render settings ───────────────────────────────────
scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = 256  # Good quality for CPU
scene.cycles.use_denoising = True

# Resolution
scene.render.resolution_x = 2048
scene.render.resolution_y = 2048
scene.render.resolution_percentage = 100

# Output
scene.render.filepath = OUT_PATH
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.render.image_settings.color_depth = "16"

# Film settings for clean background
scene.render.film_transparent = False

# Color management
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "None"

print(f"Rendering at 2048x2048 with Cycles CPU ({scene.cycles.samples} samples)...")
print(f"Output: {OUT_PATH}")

# ── Render ─────────────────────────────────────────────
bpy.ops.render.render(write_still=True)

print(f"Render complete: {OUT_PATH}")
if os.path.exists(OUT_PATH):
    file_size = os.path.getsize(OUT_PATH) / (1024 * 1024)
    print(f"File size: {file_size:.2f} MB")
else:
    print("ERROR: Output file not found!")
    sys.exit(1)
