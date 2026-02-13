"""
CAD Model: CW-1 Body Chassis + Head Module
Description: Main structural frame with mounting interfaces and head sensor housing
Iteration: 1
"""
import cadquery as cq
import math

# ============================================================
# PARAMETERS (all dimensions in mm)
# ============================================================
# Body dimensions
body_length = 600.0
body_width = 150.0
body_height = 120.0

# Frame tube dimensions
tube_size = 20.0

# Panel dimensions
panel_thickness = 3.0
side_panel_thickness = 3.0

# Head dimensions
head_depth = 80.0
head_width_rear = 150.0
head_width_front = 120.0
head_height_rear = 120.0
head_height_front = 80.0

# LED eye panel dimensions
led_panel_size = 30.0
led_panel_thickness = 3.0
led_panel_spacing = 50.0  # center-to-center distance

# LiDAR mount dimensions
lidar_od = 60.0
lidar_height = 15.0

# Antenna dimensions
antenna_od = 10.0
antenna_height = 20.0
antenna_x_offset = 60.0
antenna_y_offset = -50.0

# Leg mount plate dimensions
leg_mount_width = 80.0
leg_mount_depth = 60.0
leg_mount_thickness = 5.0
leg_mount_bolt_pattern_x = 65.0
leg_mount_bolt_pattern_y = 45.0
leg_mount_bolt_dia = 5.0

# Arm turret mount dimensions
turret_od = 90.0
turret_id = 60.0
turret_thickness = 5.0
turret_bolt_circle = 70.0
turret_bolt_dia = 4.0
turret_y_position = 150.0

# Bag hinge mount dimensions
hinge_mount_width = 30.0
hinge_mount_depth = 20.0
hinge_mount_thickness = 5.0
hinge_mount_bolt_dia = 5.0
hinge_mount_x_offset = 45.0

# Cutout dimensions
arm_turret_cutout_dia = 85.0
lidar_cutout_dia = 50.0
battery_cutout_width = 200.0
battery_cutout_depth = 100.0
battery_cutout_y = -100.0
charging_port_dia = 25.0
charging_port_y = -200.0

# Colors
color_frame = (0.5, 0.5, 0.5)
color_top_panel = (0.23, 0.29, 0.25)
color_bottom_panel = (0.2, 0.2, 0.2)
color_mount_plate = (0.3, 0.3, 0.3)
color_head = (0.23, 0.29, 0.25)
color_led = (0.13, 0.77, 0.37)
color_lidar = (0.15, 0.15, 0.15)
color_antenna = (0.1, 0.1, 0.1)
color_side_panel = (0.23, 0.29, 0.25)

# ============================================================
# CONSTRUCTION
# ============================================================

# Step 1: Chassis frame tubes (20×20mm square tubes)
# Bottom longitudinal rails (4×)
half_width = body_width / 2
half_height = body_height / 2

rail_bl_left = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((half_width, 0, -half_height))
rail_bl_right = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((-half_width, 0, -half_height))

# Bottom lateral cross-members (4×)
cross_positions_y = [-body_length/2, -100, 100, body_length/2]
bottom_crosses = []
for y_pos in cross_positions_y:
    cross = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, y_pos, -half_height))
    bottom_crosses.append(cross)

# Vertical posts at corners (4×)
corner_positions = [
    (half_width, body_length/2),
    (half_width, -body_length/2),
    (-half_width, body_length/2),
    (-half_width, -body_length/2)
]
vertical_posts = []
for x_pos, y_pos in corner_positions:
    post = cq.Workplane("XY").box(tube_size, tube_size, body_height).translate((x_pos, y_pos, 0))
    vertical_posts.append(post)

# Top longitudinal rails (2×)
rail_tl_left = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((half_width, 0, half_height))
rail_tl_right = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((-half_width, 0, half_height))

# Top lateral cross-members (2×)
top_cross_front = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, body_length/2, half_height))
top_cross_rear = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, -body_length/2, half_height))

# Combine all frame tubes
frame = rail_bl_left
for component in [rail_bl_right] + bottom_crosses + vertical_posts + [rail_tl_left, rail_tl_right, top_cross_front, top_cross_rear]:
    frame = frame.union(component)

# Step 2: Top panel with cutouts
top_panel = (cq.Workplane("XY")
    .box(body_width, body_length, panel_thickness)
    .translate((0, 0, half_height))
)

# Arm turret cutout
top_panel = (top_panel
    .faces(">Z")
    .workplane()
    .center(0, turret_y_position)
    .circle(arm_turret_cutout_dia / 2)
    .cutThruAll()
)

# LiDAR cutout
top_panel = (top_panel
    .faces(">Z")
    .workplane()
    .center(0, 0)
    .circle(lidar_cutout_dia / 2)
    .cutThruAll()
)

# Battery access cutout
top_panel = (top_panel
    .faces(">Z")
    .workplane()
    .center(0, battery_cutout_y)
    .rect(battery_cutout_width, battery_cutout_depth)
    .cutThruAll()
)

# Step 3: Bottom panel with cutouts
bottom_panel = (cq.Workplane("XY")
    .box(body_width, body_length, panel_thickness)
    .translate((0, 0, -half_height))
)

# Charging port cutout
bottom_panel = (bottom_panel
    .faces("<Z")
    .workplane()
    .center(0, charging_port_y)
    .circle(charging_port_dia / 2)
    .cutThruAll()
)

# Step 4: Leg mount plates (×4)
leg_positions = [
    (half_width, 250, "FL"),
    (-half_width, 250, "FR"),
    (half_width, -250, "RL"),
    (-half_width, -250, "RR")
]

leg_mount_plates = {}
for x_pos, y_pos, label in leg_positions:
    plate = (cq.Workplane("XY")
        .box(leg_mount_width, leg_mount_depth, leg_mount_thickness)
        .translate((x_pos, y_pos, -half_height - leg_mount_thickness/2))
    )

    # Add M5 bolt holes (4× per plate)
    bolt_pattern = [
        (leg_mount_bolt_pattern_x/2, leg_mount_bolt_pattern_y/2),
        (leg_mount_bolt_pattern_x/2, -leg_mount_bolt_pattern_y/2),
        (-leg_mount_bolt_pattern_x/2, leg_mount_bolt_pattern_y/2),
        (-leg_mount_bolt_pattern_x/2, -leg_mount_bolt_pattern_y/2)
    ]

    try:
        plate = (plate
            .faces(">Z")
            .workplane()
            .pushPoints(bolt_pattern)
            .circle(leg_mount_bolt_dia / 2)
            .cutThruAll()
        )
    except Exception as e:
        print(f"Warning: Could not add bolt holes to leg mount {label}: {e}")

    leg_mount_plates[label] = plate

# Step 5: Arm turret mount ring
arm_turret_ring = (cq.Workplane("XY")
    .circle(turret_od / 2)
    .circle(turret_id / 2)
    .extrude(turret_thickness)
    .translate((0, turret_y_position, half_height))
)

# Add M4 bolt holes on 70mm bolt circle
num_bolts = 4
bolt_angles = [i * 360 / num_bolts for i in range(num_bolts)]
bolt_positions = [
    (turret_bolt_circle/2 * math.cos(math.radians(angle)),
     turret_bolt_circle/2 * math.sin(math.radians(angle)))
    for angle in bolt_angles
]

try:
    arm_turret_ring = (arm_turret_ring
        .faces(">Z")
        .workplane()
        .pushPoints(bolt_positions)
        .circle(turret_bolt_dia / 2)
        .cutThruAll()
    )
except Exception as e:
    print(f"Warning: Could not add bolt holes to turret mount: {e}")

# Step 6: Bag system hinge mounts (×2)
hinge_mount_left = (cq.Workplane("XY")
    .box(hinge_mount_width, hinge_mount_depth, hinge_mount_thickness)
    .translate((hinge_mount_x_offset, -body_length/2, half_height + hinge_mount_thickness/2))
)

hinge_mount_right = (cq.Workplane("XY")
    .box(hinge_mount_width, hinge_mount_depth, hinge_mount_thickness)
    .translate((-hinge_mount_x_offset, -body_length/2, half_height + hinge_mount_thickness/2))
)

# Add M5 bolt holes (2× per plate)
hinge_bolt_pattern = [(8, 0), (-8, 0)]

for mount in [hinge_mount_left, hinge_mount_right]:
    try:
        mount = (mount
            .faces(">Z")
            .workplane()
            .pushPoints(hinge_bolt_pattern)
            .circle(hinge_mount_bolt_dia / 2)
            .cutThruAll()
        )
    except Exception as e:
        print(f"Warning: Could not add bolt holes to hinge mount: {e}")

# Step 7: Head module (simple box to avoid loft issues)
head_box = (cq.Workplane("XY")
    .box(head_width_rear, head_depth, head_height_front)
    .translate((0, body_length/2 + head_depth/2, -20))
)

# Step 8: LED eye panels (×2)
led_left = (cq.Workplane("XY")
    .box(led_panel_size, led_panel_thickness, led_panel_size)
    .translate((led_panel_spacing/2, body_length/2 + head_depth, 10))
)

led_right = (cq.Workplane("XY")
    .box(led_panel_size, led_panel_thickness, led_panel_size)
    .translate((-led_panel_spacing/2, body_length/2 + head_depth, 10))
)

# Step 9: LiDAR mount
lidar_mount = (cq.Workplane("XY")
    .circle(lidar_od / 2)
    .extrude(lidar_height)
    .translate((0, 0, half_height + panel_thickness + lidar_height/2))
)

# Step 10: Antenna nub
antenna = (cq.Workplane("XY")
    .circle(antenna_od / 2)
    .extrude(antenna_height)
    .translate((antenna_x_offset, antenna_y_offset, half_height + panel_thickness + antenna_height/2))
)

# Step 11: Side panels (×2)
side_panel_left = (cq.Workplane("YZ")
    .box(body_length, body_height, side_panel_thickness)
    .translate((half_width + side_panel_thickness/2, 0, 0))
)

side_panel_right = (cq.Workplane("YZ")
    .box(body_length, body_height, side_panel_thickness)
    .translate((-half_width - side_panel_thickness/2, 0, 0))
)

# ============================================================
# DIMENSION CHECK
# ============================================================
# Expected total dimensions:
# Length (with head): 600 + 80 = 680mm
# Width: 150mm (plus side panels = 150 + 2*3 = 156mm)
# Height (with LiDAR): 120 + 3 + 15 = 138mm

# Compute bounding box from main body frame
frame_bbox = frame.val().BoundingBox()
print(f"Frame bounding box: X=[{frame_bbox.xmin:.1f}, {frame_bbox.xmax:.1f}], "
      f"Y=[{frame_bbox.ymin:.1f}, {frame_bbox.ymax:.1f}], "
      f"Z=[{frame_bbox.zmin:.1f}, {frame_bbox.zmax:.1f}]")

# Expected frame dimensions: X=[-75-10, 75+10] Y=[-300, 300] Z=[-60, 60]
expected_x_span = body_width + tube_size
expected_y_span = body_length
expected_z_span = body_height

actual_x_span = frame_bbox.xmax - frame_bbox.xmin
actual_y_span = frame_bbox.ymax - frame_bbox.ymin
actual_z_span = frame_bbox.zmax - frame_bbox.zmin

print(f"Expected frame spans: X={expected_x_span:.1f}, Y={expected_y_span:.1f}, Z={expected_z_span:.1f}")
print(f"Actual frame spans: X={actual_x_span:.1f}, Y={actual_y_span:.1f}, Z={actual_z_span:.1f}")

# Check within 10% tolerance
tolerance = 0.10
for name, expected, actual in [("X", expected_x_span, actual_x_span),
                                ("Y", expected_y_span, actual_y_span),
                                ("Z", expected_z_span, actual_z_span)]:
    diff_pct = abs(actual - expected) / expected
    if diff_pct > tolerance:
        print(f"WARNING: {name}-dimension off by {diff_pct*100:.1f}% (expected {expected:.1f}, got {actual:.1f})")

# ============================================================
# ASSEMBLY
# ============================================================
assy = cq.Assembly()

# Add all components
assy.add(frame, name="chassis_frame", color=cq.Color(*color_frame))
assy.add(top_panel, name="top_panel", color=cq.Color(*color_top_panel))
assy.add(bottom_panel, name="bottom_panel", color=cq.Color(*color_bottom_panel))

assy.add(leg_mount_plates["FL"], name="leg_mount_FL", color=cq.Color(*color_mount_plate))
assy.add(leg_mount_plates["FR"], name="leg_mount_FR", color=cq.Color(*color_mount_plate))
assy.add(leg_mount_plates["RL"], name="leg_mount_RL", color=cq.Color(*color_mount_plate))
assy.add(leg_mount_plates["RR"], name="leg_mount_RR", color=cq.Color(*color_mount_plate))

assy.add(arm_turret_ring, name="arm_turret_mount", color=cq.Color(*color_mount_plate))
assy.add(hinge_mount_left, name="hinge_mount_left", color=cq.Color(*color_mount_plate))
assy.add(hinge_mount_right, name="hinge_mount_right", color=cq.Color(*color_mount_plate))

assy.add(head_box, name="head_module", color=cq.Color(*color_head))
assy.add(led_left, name="led_left", color=cq.Color(*color_led))
assy.add(led_right, name="led_right", color=cq.Color(*color_led))

assy.add(lidar_mount, name="lidar_mount", color=cq.Color(*color_lidar))
assy.add(antenna, name="antenna", color=cq.Color(*color_antenna))

assy.add(side_panel_left, name="side_panel_left", color=cq.Color(*color_side_panel))
assy.add(side_panel_right, name="side_panel_right", color=cq.Color(*color_side_panel))

# ============================================================
# EXPORT
# ============================================================
assy.save("models/model.step")
compound = assy.toCompound()
cq.exporters.export(compound, "models/model.stl")
print("Export complete: models/model.step, models/model.stl")
