"""
CAD Model: CW-1 Bag System Assembly
Description: Bag dispensing and collection system with roll dispenser, folding frame, and clip system
Iteration: 1
"""
import cadquery as cq
import math

# ============================================================
# PARAMETERS (all dimensions in mm)
# ============================================================

# Overall system dimensions
system_width = 150.0  # Overall width (matches robot body)
system_depth_base = 150.0  # Distance from roll front to hinge

# Bag roll dispenser
roll_diameter = 80.0  # Outer diameter of bag roll
roll_width = 130.0  # Width of bag roll (narrower than body)
roll_housing_height = 15.0  # Height of cradle above body surface
roll_housing_wall = 3.0  # Thickness of housing walls
roll_bracket_thickness = 3.0  # Side bracket thickness
roll_bracket_height = 25.0  # Height of side brackets
roll_bracket_width = 20.0  # Width of side brackets

# Folding frame
frame_tube_diameter = 10.0  # Diameter of frame tubing
frame_width = 150.0  # Frame width (matches body)
frame_depth = 220.0  # Frame front-to-back depth
frame_angle_deg = 135.0  # Frame open angle from body surface
support_bar_diameter = 10.0  # Diameter of support bars
support_bar_length = 80.0  # Length of support bars

# Hinge assembly
hinge_pin_diameter = 4.0  # Diameter of hinge pin
hinge_pin_length = 160.0  # Length of hinge pin (spans body width)
hinge_bracket_thickness = 3.0  # Hinge bracket material thickness
hinge_bracket_width = 25.0  # Hinge bracket footprint width
hinge_bracket_depth = 20.0  # Hinge bracket footprint depth
hinge_bracket_height = 15.0  # Height of hinge bracket
hinge_bracket_spacing = 30.0  # Distance from each side edge
servo_clearance_width = 15.0  # Servo clearance width
servo_clearance_height = 30.0  # Servo clearance height

# Clip system
clip_count = 4  # Number of clips per line
clip_width = 20.0  # Width of each clip
clip_height = 8.0  # Height of each clip
clip_thickness = 2.0  # Thickness of spring steel
clip_standoff = 3.0  # Height above mounting surface

# Mounting and positioning
roll_center_y = 0.0  # Roll center Y position (front of system)
hinge_y = 150.0  # Hinge Y position (rear edge)

# ============================================================
# CONSTRUCTION
# ============================================================

# Step 1: Bag Roll Dispenser Housing
# Create a shallow cradle that sits low on the body surface
roll_housing = (
    cq.Workplane("XY")
    .move(0, roll_center_y)
    .rect(roll_width + 2 * roll_housing_wall, roll_diameter / 2 + roll_housing_wall)
    .extrude(roll_housing_height)
    # Cut out the roll cavity
    .faces(">Z")
    .workplane()
    .move(0, roll_housing_wall)
    .rect(roll_width, roll_diameter / 2)
    .cutBlind(-roll_housing_height + roll_housing_wall)
)

# Step 2: Roll cylinder (representing the bag roll itself)
roll_cylinder = (
    cq.Workplane("YZ")
    .move(roll_center_y, roll_housing_height + roll_diameter / 2)
    .circle(roll_diameter / 2)
    .extrude(roll_width)
    .translate((-roll_width / 2, 0, 0))
)

# Step 3: Side mounting brackets for roll
# Left bracket
left_bracket = (
    cq.Workplane("XY")
    .move(-(roll_width / 2 + roll_housing_wall + roll_bracket_thickness / 2), roll_center_y)
    .rect(roll_bracket_thickness, roll_bracket_width)
    .extrude(roll_bracket_height)
)

# Right bracket (mirror of left)
right_bracket = (
    cq.Workplane("XY")
    .move((roll_width / 2 + roll_housing_wall + roll_bracket_thickness / 2), roll_center_y)
    .rect(roll_bracket_thickness, roll_bracket_width)
    .extrude(roll_bracket_height)
)

# Step 4: Hinge brackets at rear edge
# Left hinge bracket
left_hinge_bracket = (
    cq.Workplane("XY")
    .move(-(system_width / 2 - hinge_bracket_spacing), hinge_y)
    .rect(hinge_bracket_width, hinge_bracket_depth)
    .extrude(hinge_bracket_height)
    # Drill hole for hinge pin
    .faces(">Z")
    .workplane()
    .hole(hinge_pin_diameter + 0.5)  # Slight clearance
)

# Right hinge bracket (exclude servo clearance area)
right_hinge_bracket = (
    cq.Workplane("XY")
    .move((system_width / 2 - hinge_bracket_spacing), hinge_y)
    .rect(hinge_bracket_width, hinge_bracket_depth)
    .extrude(hinge_bracket_height)
    # Drill hole for hinge pin
    .faces(">Z")
    .workplane()
    .hole(hinge_pin_diameter + 0.5)
)

# Step 5: Hinge pin
hinge_pin = (
    cq.Workplane("YZ")
    .move(hinge_y, hinge_bracket_height)
    .circle(hinge_pin_diameter / 2)
    .extrude(hinge_pin_length)
    .translate((-hinge_pin_length / 2, 0, 0))
)

# Step 6: Support bars connecting frame to hinge
# Calculate support bar end position based on frame angle
frame_angle_rad = math.radians(frame_angle_deg)
support_bar_end_y = hinge_y + support_bar_length * math.cos(frame_angle_rad - math.pi / 2)
support_bar_end_z = hinge_bracket_height + support_bar_length * math.sin(frame_angle_rad - math.pi / 2)

# Left support bar
left_support_start = (-frame_width / 4, hinge_y, hinge_bracket_height)
left_support_end = (-frame_width / 4, support_bar_end_y, support_bar_end_z)

left_support_bar = (
    cq.Workplane("XY")
    .move(left_support_start[0], left_support_start[1])
    .circle(support_bar_diameter / 2)
    .extrude(support_bar_length)
    .rotate((0, 0, 0), (1, 0, 0), frame_angle_deg - 90)
    .translate((0, hinge_y, hinge_bracket_height))
)

# Right support bar
right_support_start = (frame_width / 4, hinge_y, hinge_bracket_height)
right_support_end = (frame_width / 4, support_bar_end_y, support_bar_end_z)

right_support_bar = (
    cq.Workplane("XY")
    .move(right_support_start[0], right_support_start[1])
    .circle(support_bar_diameter / 2)
    .extrude(support_bar_length)
    .rotate((0, 0, 0), (1, 0, 0), frame_angle_deg - 90)
    .translate((0, hinge_y, hinge_bracket_height))
)

# Step 7: Folding frame rim (rectangular frame made of circular tubing)
# Calculate frame position at the end of support bars
frame_attach_y = support_bar_end_y
frame_attach_z = support_bar_end_z

# Frame corner positions in the frame's local coordinate system
# Front edge of frame at attachment point, extends backward/upward at frame angle
frame_front_left = (-frame_width / 2, 0, 0)
frame_front_right = (frame_width / 2, 0, 0)
frame_back_left = (-frame_width / 2, frame_depth, 0)
frame_back_right = (frame_width / 2, frame_depth, 0)

# Create frame as four tubes forming a rectangle
# Front tube (left to right)
frame_front = (
    cq.Workplane("XZ")
    .move(-frame_width / 2, 0)
    .circle(frame_tube_diameter / 2)
    .extrude(frame_width)
)

# Back tube (left to right)
frame_back = (
    cq.Workplane("XZ")
    .move(-frame_width / 2, 0)
    .circle(frame_tube_diameter / 2)
    .extrude(frame_width)
    .translate((0, frame_depth, 0))
)

# Left tube (front to back)
frame_left = (
    cq.Workplane("XY")
    .move(-frame_width / 2, 0)
    .circle(frame_tube_diameter / 2)
    .extrude(frame_depth)
)

# Right tube (front to back)
frame_right = (
    cq.Workplane("XY")
    .move(frame_width / 2, 0)
    .circle(frame_tube_diameter / 2)
    .extrude(frame_depth)
)

# Combine frame tubes
frame = frame_front.union(frame_back).union(frame_left).union(frame_right)

# Rotate frame to open position and position at end of support bars
frame = (
    frame
    .rotate((0, 0, 0), (1, 0, 0), frame_angle_deg - 90)
    .translate((0, frame_attach_y, frame_attach_z))
)

# Step 8: Inner clip line (at roll front edge)
# Create 4 evenly spaced clips along the front of the roll area
clip_spacing = roll_width / (clip_count + 1)

inner_clips = cq.Workplane("XY")
for i in range(clip_count):
    x_pos = -roll_width / 2 + clip_spacing * (i + 1)
    clip = (
        cq.Workplane("XY")
        .move(x_pos, roll_center_y - roll_diameter / 2 - roll_housing_wall)
        .rect(clip_width, clip_thickness)
        .extrude(clip_height)
        .translate((0, 0, clip_standoff))
    )
    if i == 0:
        inner_clips = clip
    else:
        inner_clips = inner_clips.union(clip)

# Step 9: Outer clip line (at frame back edge)
# Calculate position of frame back edge in world coordinates
frame_back_y = frame_attach_y + frame_depth * math.cos(frame_angle_rad - math.pi / 2)
frame_back_z = frame_attach_z + frame_depth * math.sin(frame_angle_rad - math.pi / 2)

outer_clips = cq.Workplane("XY")
for i in range(clip_count):
    x_pos = -frame_width / 2 + clip_spacing * (i + 1)
    clip = (
        cq.Workplane("XY")
        .move(x_pos, 0)
        .rect(clip_width, clip_thickness)
        .extrude(clip_height)
        # Rotate to match frame angle
        .rotate((0, 0, 0), (1, 0, 0), frame_angle_deg - 90)
        # Position at frame back edge
        .translate((0, frame_back_y, frame_back_z))
    )
    if i == 0:
        outer_clips = clip
    else:
        outer_clips = outer_clips.union(clip)

# ============================================================
# ASSEMBLY
# ============================================================

# Create assembly and add all components
assy = cq.Assembly()
assy.add(roll_housing, name="roll_housing", color=cq.Color(0.2, 0.2, 0.2))
assy.add(roll_cylinder, name="roll_cylinder", color=cq.Color(0.8, 0.8, 0.8))
assy.add(left_bracket, name="left_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(right_bracket, name="right_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(left_hinge_bracket, name="left_hinge_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(right_hinge_bracket, name="right_hinge_bracket", color=cq.Color(0.2, 0.2, 0.2))
assy.add(hinge_pin, name="hinge_pin", color=cq.Color(0.7, 0.7, 0.7))
assy.add(left_support_bar, name="left_support_bar", color=cq.Color(0.3, 0.3, 0.3))
assy.add(right_support_bar, name="right_support_bar", color=cq.Color(0.3, 0.3, 0.3))
assy.add(frame, name="frame", color=cq.Color(0.3, 0.3, 0.3))
assy.add(inner_clips, name="inner_clips", color=cq.Color(0.7, 0.7, 0.7))
assy.add(outer_clips, name="outer_clips", color=cq.Color(0.7, 0.7, 0.7))

# ============================================================
# EXPORT
# ============================================================

# Export assembly to STEP
assy.save("/home/deploy/cleanwalkerrobotics/hardware/cad-agent/models/model.step")

# Convert to compound for STL export
compound = assy.toCompound()
cq.exporters.export(compound, "/home/deploy/cleanwalkerrobotics/hardware/cad-agent/models/model.stl")

print("Export complete: models/model.step, models/model.stl")
