"""
CAD Model: CW-1 Bag System Assembly — COMPLETE REBUILD
Description: Full bag system with 600mm body plate, arm turret stub,
             roll dispenser, folding frame at 135deg, and clip system
Iteration: 5 (full rebuild)
"""
import cadquery as cq
import math

# ============================================================
# PARAMETERS (all dimensions in mm)
# ============================================================

# Body plate (full-length robot body reference)
body_length = 600.0
body_width = 150.0
body_plate_thickness = 5.0

# Arm turret stub (front-center mounting context)
turret_diameter = 80.0
turret_height = 50.0
turret_y = 100.0

# Bag roll dispenser
roll_diameter = 80.0
roll_width = 130.0
roll_housing_height = 15.0
roll_housing_wall = 3.0
roll_center_y = 450.0  # Centered on rear half (300-600)

# Roll side brackets
roll_bracket_thickness = 3.0
roll_bracket_height = 25.0
roll_bracket_width = 20.0

# Folding frame (TUBE METHOD: 4 cylinders + union, NEVER sweep)
frame_tube_diameter = 10.0
frame_width = 150.0
frame_depth = 220.0
frame_angle_deg = 135.0

# Support bars (hinge to frame)
support_bar_diameter = 10.0
support_bar_length = 91.0

# Hinge assembly
hinge_y = 600.0
hinge_bracket_height = 15.0
hinge_bracket_width = 25.0
hinge_bracket_depth = 20.0
hinge_bracket_thickness = 3.0
hinge_bracket_foot_height = 3.0
hinge_bracket_spacing = 30.0
hinge_pin_diameter = 4.0
hinge_pin_length = 160.0

# Servo clearance
servo_clearance_width = 15.0
servo_clearance_depth = 15.0
servo_clearance_height = 30.0

# Clip system
clip_count = 4
clip_width = 20.0
clip_height = 8.0
clip_thickness = 2.0
clip_standoff = 3.0

# ============================================================
# DIMENSION CHECK
# ============================================================
# Body plate:  Y=0..600, X=-75..75, Z=-5..0
# Turret:      Y=100, Z=0..50, diameter 80mm
# Roll center: Y=450, diameter 80mm -> Y extent ~410..490
# Hinge:       Y=600, Z=0..15 (bracket height)
# Rotation:    135deg - 90deg = 45deg tilt
# Bar end Y:   600 + 91*cos(45deg) = 664
# Bar end Z:   15 + 91*sin(45deg) = 79
# Frame back Y: 664 + 220*cos(45deg) = 820 -> 220mm behind rear edge OK
# Frame back Z: 79 + 220*sin(45deg) = 235 -> ~235mm above body surface
# Width: 150mm (body/frame), 160mm (hinge pin)

# ============================================================
# CONSTRUCTION
# ============================================================

# Step 1: Body plate (600x150mm reference surface)
body_plate = (
    cq.Workplane("XY")
    .move(0, body_length / 2)
    .rect(body_width, body_length)
    .extrude(-body_plate_thickness)
)

# Step 2: Arm turret stub (front-center context)
turret = (
    cq.Workplane("XY")
    .move(0, turret_y)
    .circle(turret_diameter / 2)
    .extrude(turret_height)
)

# Step 3: Roll housing (curved cradle)
try:
    roll_housing = (
        cq.Workplane("XY")
        .move(0, roll_center_y)
        .rect(roll_width + 2 * roll_housing_wall, roll_diameter / 2 + 2 * roll_housing_wall)
        .extrude(roll_housing_height)
    )
    cradle_cutter = (
        cq.Workplane("YZ")
        .move(roll_center_y, roll_housing_height + roll_diameter / 2 - roll_housing_wall)
        .circle(roll_diameter / 2 + 0.5)
        .extrude(roll_width)
        .translate((-roll_width / 2, 0, 0))
    )
    roll_housing = roll_housing.cut(cradle_cutter)
except Exception as e:
    print(f"Warning: Roll housing error: {e}")
    roll_housing = (
        cq.Workplane("XY")
        .move(0, roll_center_y)
        .rect(roll_width + 2 * roll_housing_wall, roll_diameter / 2 + roll_housing_wall)
        .extrude(roll_housing_height)
    )

# Step 4: Roll cylinder (bag roll itself)
roll_cylinder = (
    cq.Workplane("YZ")
    .move(roll_center_y, roll_housing_height + roll_diameter / 2)
    .circle(roll_diameter / 2)
    .extrude(roll_width)
    .translate((-roll_width / 2, 0, 0))
)

# Step 5: Roll side brackets
left_roll_bracket = (
    cq.Workplane("XY")
    .move(-(roll_width / 2 + roll_housing_wall + roll_bracket_thickness / 2), roll_center_y)
    .rect(roll_bracket_thickness, roll_bracket_width)
    .extrude(roll_bracket_height)
)
right_roll_bracket = (
    cq.Workplane("XY")
    .move((roll_width / 2 + roll_housing_wall + roll_bracket_thickness / 2), roll_center_y)
    .rect(roll_bracket_thickness, roll_bracket_width)
    .extrude(roll_bracket_height)
)

# Step 6: Hinge brackets (L-shaped profile)
try:
    left_hinge_bracket = (
        cq.Workplane("XZ")
        .move(-(body_width / 2 - hinge_bracket_spacing), 0)
        .polyline([
            (0, 0),
            (hinge_bracket_width, 0),
            (hinge_bracket_width, hinge_bracket_foot_height),
            (hinge_bracket_thickness, hinge_bracket_foot_height),
            (hinge_bracket_thickness, hinge_bracket_height),
            (0, hinge_bracket_height),
        ])
        .close()
        .extrude(hinge_bracket_depth)
        .translate((0, hinge_y - hinge_bracket_depth / 2, 0))
    )
except Exception as e:
    print(f"Warning: Left hinge bracket error: {e}")
    left_hinge_bracket = (
        cq.Workplane("XY")
        .move(-(body_width / 2 - hinge_bracket_spacing), hinge_y)
        .rect(hinge_bracket_width, hinge_bracket_depth)
        .extrude(hinge_bracket_height)
    )

try:
    right_hinge_bracket = (
        cq.Workplane("XZ")
        .move((body_width / 2 - hinge_bracket_spacing), 0)
        .polyline([
            (0, 0),
            (-hinge_bracket_width, 0),
            (-hinge_bracket_width, hinge_bracket_foot_height),
            (-hinge_bracket_thickness, hinge_bracket_foot_height),
            (-hinge_bracket_thickness, hinge_bracket_height),
            (0, hinge_bracket_height),
        ])
        .close()
        .extrude(hinge_bracket_depth)
        .translate((0, hinge_y - hinge_bracket_depth / 2, 0))
    )
except Exception as e:
    print(f"Warning: Right hinge bracket error: {e}")
    right_hinge_bracket = (
        cq.Workplane("XY")
        .move((body_width / 2 - hinge_bracket_spacing), hinge_y)
        .rect(hinge_bracket_width, hinge_bracket_depth)
        .extrude(hinge_bracket_height)
    )

# Step 6b: Servo clearance void
try:
    servo_void = (
        cq.Workplane("XY")
        .move(-(body_width / 2 - hinge_bracket_spacing) + servo_clearance_width / 2, hinge_y)
        .rect(servo_clearance_width, servo_clearance_depth)
        .extrude(servo_clearance_height)
    )
    left_hinge_bracket = left_hinge_bracket.cut(servo_void)
except Exception as e:
    print(f"Warning: Servo void cut failed: {e}")

# Step 7: Hinge pin
hinge_pin = (
    cq.Workplane("YZ")
    .move(hinge_y, hinge_bracket_height)
    .circle(hinge_pin_diameter / 2)
    .extrude(hinge_pin_length)
    .translate((-hinge_pin_length / 2, 0, 0))
)

# Step 8: Support bars (build at origin -> rotate -> translate ONCE)
frame_angle_rad = math.radians(frame_angle_deg)
rot_angle = frame_angle_deg - 90  # 45 degrees

support_bar_end_y = hinge_y + support_bar_length * math.cos(frame_angle_rad - math.pi / 2)
support_bar_end_z = hinge_bracket_height + support_bar_length * math.sin(frame_angle_rad - math.pi / 2)

left_support_bar = (
    cq.Workplane("XY")
    .move(-frame_width / 4, 0)
    .circle(support_bar_diameter / 2)
    .extrude(support_bar_length)
    .rotate((0, 0, 0), (1, 0, 0), rot_angle)
    .translate((0, hinge_y, hinge_bracket_height))
)

right_support_bar = (
    cq.Workplane("XY")
    .move(frame_width / 4, 0)
    .circle(support_bar_diameter / 2)
    .extrude(support_bar_length)
    .rotate((0, 0, 0), (1, 0, 0), rot_angle)
    .translate((0, hinge_y, hinge_bracket_height))
)

# Step 9: Folding frame rim — TUBE METHOD (4 cylinders + union, NEVER sweep)
frame_attach_y = support_bar_end_y
frame_attach_z = support_bar_end_z

r = frame_tube_diameter / 2

# Front tube (width-spanning at near edge)
frame_front = (
    cq.Workplane("XZ")
    .move(-frame_width / 2 - r, 0)
    .circle(r)
    .extrude(frame_width + 2 * r)
)

# Back tube (width-spanning at far edge)
frame_back = (
    cq.Workplane("XZ")
    .move(-frame_width / 2 - r, 0)
    .circle(r)
    .extrude(frame_width + 2 * r)
    .translate((0, frame_depth, 0))
)

# Left tube (depth-spanning)
frame_left = (
    cq.Workplane("XY")
    .move(-frame_width / 2, -r)
    .circle(r)
    .extrude(frame_depth + 2 * r)
)

# Right tube (depth-spanning)
frame_right = (
    cq.Workplane("XY")
    .move(frame_width / 2, -r)
    .circle(r)
    .extrude(frame_depth + 2 * r)
)

# Union all 4 tubes — solid corner joints from overlap
frame = frame_front.union(frame_back).union(frame_left).union(frame_right)

# Rotate to open position and translate to support bar endpoints
frame = (
    frame
    .rotate((0, 0, 0), (1, 0, 0), rot_angle)
    .translate((0, frame_attach_y, frame_attach_z))
)

# Step 10: Inner clip line (at roll front edge)
inner_clip_spacing = roll_width / (clip_count + 1)
inner_clips = None
for i in range(clip_count):
    x_pos = -roll_width / 2 + inner_clip_spacing * (i + 1)
    clip = (
        cq.Workplane("XY")
        .move(x_pos, roll_center_y - roll_diameter / 2 - roll_housing_wall)
        .rect(clip_width, clip_thickness)
        .extrude(clip_height)
        .translate((0, 0, clip_standoff))
    )
    if inner_clips is None:
        inner_clips = clip
    else:
        inner_clips = inner_clips.union(clip)

# Step 11: Outer clip line (at frame far rim — spacing per frame_width, NOT roll_width)
frame_back_y = frame_attach_y + frame_depth * math.cos(frame_angle_rad - math.pi / 2)
frame_back_z = frame_attach_z + frame_depth * math.sin(frame_angle_rad - math.pi / 2)

outer_clip_spacing = frame_width / (clip_count + 1)
outer_clips = None
for i in range(clip_count):
    x_pos = -frame_width / 2 + outer_clip_spacing * (i + 1)
    clip = (
        cq.Workplane("XY")
        .move(x_pos, 0)
        .rect(clip_width, clip_thickness)
        .extrude(clip_height)
        .rotate((0, 0, 0), (1, 0, 0), rot_angle)
        .translate((0, frame_back_y, frame_back_z))
    )
    if outer_clips is None:
        outer_clips = clip
    else:
        outer_clips = outer_clips.union(clip)

# ============================================================
# ASSEMBLY
# ============================================================

assy = cq.Assembly()
assy.add(body_plate, name="body_plate", color=cq.Color(0.23, 0.29, 0.25))
assy.add(turret, name="arm_turret", color=cq.Color(0.2, 0.2, 0.2))
assy.add(roll_housing, name="roll_housing", color=cq.Color(0.15, 0.15, 0.15))
assy.add(roll_cylinder, name="roll_cylinder", color=cq.Color(0.4, 0.4, 0.4))
assy.add(left_roll_bracket, name="left_roll_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(right_roll_bracket, name="right_roll_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(left_hinge_bracket, name="left_hinge_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(right_hinge_bracket, name="right_hinge_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(hinge_pin, name="hinge_pin", color=cq.Color(0.6, 0.6, 0.6))
assy.add(left_support_bar, name="left_support_bar", color=cq.Color(0.25, 0.25, 0.25))
assy.add(right_support_bar, name="right_support_bar", color=cq.Color(0.25, 0.25, 0.25))
assy.add(frame, name="frame", color=cq.Color(0.25, 0.25, 0.25))
assy.add(inner_clips, name="inner_clips", color=cq.Color(0.6, 0.6, 0.6))
assy.add(outer_clips, name="outer_clips", color=cq.Color(0.6, 0.6, 0.6))

# ============================================================
# EXPORT
# ============================================================

assy.save("models/model.step")
compound = assy.toCompound()
cq.exporters.export(compound, "models/model.stl")

print("Export complete: models/model.step, models/model.stl")
print("COMPLETE REBUILD — Full bag system with 600mm body plate")
print(f"  Body plate: {body_length}x{body_width}mm")
print(f"  Arm turret: dia {turret_diameter}mm at Y={turret_y}")
print(f"  Roll: dia {roll_diameter}mm centered at Y={roll_center_y}")
print(f"  Frame: {frame_depth}mm deep, dia {frame_tube_diameter}mm tube (4-cyl union)")
print(f"  Frame tip: ~{frame_back_y - hinge_y:.0f}mm behind, ~{frame_back_z:.0f}mm above rear edge")
print(f"  Angle: {frame_angle_deg} deg from body surface")
