"""
CAD Model: CW-1 Front-Left Leg Module
Description: 3-DOF leg with hip yaw, hip pitch, knee pitch actuators
Iteration: 1
"""
import cadquery as cq
import math

# ============================================================
# PARAMETERS (all dimensions in mm)
# ============================================================

# Body mounting plate
body_plate_width = 100.0
body_plate_depth = 100.0
body_plate_thickness = 5.0

# Hip mounting plate
hip_plate_width = 80.0
hip_plate_depth = 60.0
hip_plate_thickness = 5.0
hip_plate_bolt_pattern_width = 65.0
hip_plate_bolt_pattern_depth = 45.0
hip_plate_bolt_hole_diameter = 6.0  # M5 clearance
hip_plate_cable_hole_diameter = 30.0

# Hip yaw actuator (AK60-6)
hip_yaw_outer_diameter = 82.0
hip_yaw_inner_diameter = 77.0
hip_yaw_height = 45.0

# Hip pitch actuator (AK70-10)
hip_pitch_outer_diameter = 104.0
hip_pitch_inner_diameter = 99.0
hip_pitch_housing_length = 55.0
hip_pitch_lateral_offset = 35.0  # Y offset from yaw axis

# U-bracket connecting yaw to pitch
bracket_thickness = 3.0
bracket_width = 40.0
bracket_height = 60.0

# Upper leg (thigh) - parallel tubes
upper_leg_tube_outer_diameter = 15.0
upper_leg_tube_inner_diameter = 12.0
upper_leg_tube_spacing = 30.0  # center-to-center
upper_leg_length = 200.0
upper_leg_cross_brace_positions = [upper_leg_length / 3, 2 * upper_leg_length / 3]

# Knee pitch actuator (AK70-10)
knee_pitch_outer_diameter = 104.0
knee_pitch_inner_diameter = 99.0
knee_pitch_housing_length = 55.0

# Lower leg (calf)
lower_leg_outer_diameter = 30.0
lower_leg_inner_diameter = 25.0
lower_leg_length = 200.0

# Foot
foot_radius = 25.0

# Z-stack positions (measuring from body interface at Z=0, downward is negative)
z_body_interface = 0.0
z_mounting_plate_bottom = z_body_interface - hip_plate_thickness
z_hip_yaw_top = z_mounting_plate_bottom
z_hip_yaw_bottom = z_hip_yaw_top - hip_yaw_height
z_hip_pitch_axis = z_hip_yaw_bottom
z_knee_axis = z_hip_pitch_axis - upper_leg_length
z_foot_center = z_knee_axis - lower_leg_length
z_foot_bottom = z_foot_center - foot_radius

# ============================================================
# CONSTRUCTION
# ============================================================

# Body plate (reference mounting surface)
body_plate = (
    cq.Workplane("XY")
    .box(body_plate_width, body_plate_depth, body_plate_thickness, centered=(True, True, False))
    .translate((0, 0, -body_plate_thickness))
)

# Hip mounting plate with bolt holes and cable pass-through
hip_mounting_plate = (
    cq.Workplane("XY")
    .box(hip_plate_width, hip_plate_depth, hip_plate_thickness, centered=(True, True, False))
    .translate((0, 0, z_body_interface))
)

# Add bolt holes at corners
hip_mounting_plate = (
    hip_mounting_plate
    .faces(">Z")
    .workplane()
    .pushPoints([
        (hip_plate_bolt_pattern_width / 2, hip_plate_bolt_pattern_depth / 2),
        (-hip_plate_bolt_pattern_width / 2, hip_plate_bolt_pattern_depth / 2),
        (hip_plate_bolt_pattern_width / 2, -hip_plate_bolt_pattern_depth / 2),
        (-hip_plate_bolt_pattern_width / 2, -hip_plate_bolt_pattern_depth / 2),
    ])
    .hole(hip_plate_bolt_hole_diameter)
)

# Central cable pass-through hole
hip_mounting_plate = (
    hip_mounting_plate
    .faces(">Z")
    .workplane()
    .hole(hip_plate_cable_hole_diameter)
)

# Hip yaw housing (cylindrical shell)
hip_yaw_outer = (
    cq.Workplane("XY")
    .circle(hip_yaw_outer_diameter / 2)
    .extrude(-hip_yaw_height)
    .translate((0, 0, z_hip_yaw_top))
)

hip_yaw_inner = (
    cq.Workplane("XY")
    .circle(hip_yaw_inner_diameter / 2)
    .extrude(-hip_yaw_height - 1)  # Slightly longer for clean cut
    .translate((0, 0, z_hip_yaw_top))
)

hip_yaw_housing = hip_yaw_outer.cut(hip_yaw_inner)

# U-bracket connecting hip yaw to hip pitch
# Simple vertical plate with offset for hip pitch motor
u_bracket = (
    cq.Workplane("XZ")
    .center(0, z_hip_pitch_axis)
    .rect(bracket_width, bracket_height)
    .extrude(bracket_thickness)
    .translate((0, -bracket_thickness / 2, 0))
)

# Hip pitch housing (cylindrical shell, oriented along Y axis)
# Build at origin along Y, then translate to position
hip_pitch_outer = (
    cq.Workplane("XZ")
    .circle(hip_pitch_outer_diameter / 2)
    .extrude(hip_pitch_housing_length)
    .translate((0, -hip_pitch_housing_length / 2, z_hip_pitch_axis))
)

hip_pitch_inner = (
    cq.Workplane("XZ")
    .circle(hip_pitch_inner_diameter / 2)
    .extrude(hip_pitch_housing_length + 1)
    .translate((0, -hip_pitch_housing_length / 2 - 0.5, z_hip_pitch_axis))
)

hip_pitch_housing = hip_pitch_outer.cut(hip_pitch_inner)

# Upper leg - two parallel tubes with cross braces
# Tube 1: at X = +upper_leg_tube_spacing/2
upper_leg_tube1_outer = (
    cq.Workplane("XY")
    .workplane(offset=z_hip_pitch_axis)
    .circle(upper_leg_tube_outer_diameter / 2)
    .extrude(-upper_leg_length)
    .translate((upper_leg_tube_spacing / 2, 0, 0))
)

upper_leg_tube1_inner = (
    cq.Workplane("XY")
    .workplane(offset=z_hip_pitch_axis)
    .circle(upper_leg_tube_inner_diameter / 2)
    .extrude(-upper_leg_length - 1)
    .translate((upper_leg_tube_spacing / 2, 0, 0))
)

upper_leg_tube1 = upper_leg_tube1_outer.cut(upper_leg_tube1_inner)

# Tube 2: at X = -upper_leg_tube_spacing/2
upper_leg_tube2_outer = (
    cq.Workplane("XY")
    .workplane(offset=z_hip_pitch_axis)
    .circle(upper_leg_tube_outer_diameter / 2)
    .extrude(-upper_leg_length)
    .translate((-upper_leg_tube_spacing / 2, 0, 0))
)

upper_leg_tube2_inner = (
    cq.Workplane("XY")
    .workplane(offset=z_hip_pitch_axis)
    .circle(upper_leg_tube_inner_diameter / 2)
    .extrude(-upper_leg_length - 1)
    .translate((-upper_leg_tube_spacing / 2, 0, 0))
)

upper_leg_tube2 = upper_leg_tube2_outer.cut(upper_leg_tube2_inner)

# Cross braces (horizontal tubes along X direction connecting the two vertical tubes)
cross_brace1_outer = (
    cq.Workplane("YZ")
    .workplane(offset=-upper_leg_tube_spacing / 2)
    .circle(upper_leg_tube_outer_diameter / 2)
    .extrude(upper_leg_tube_spacing)
    .translate((0, 0, z_hip_pitch_axis - upper_leg_cross_brace_positions[0]))
)

cross_brace1_inner = (
    cq.Workplane("YZ")
    .workplane(offset=-upper_leg_tube_spacing / 2 - 0.5)
    .circle(upper_leg_tube_inner_diameter / 2)
    .extrude(upper_leg_tube_spacing + 1)
    .translate((0, 0, z_hip_pitch_axis - upper_leg_cross_brace_positions[0]))
)

cross_brace1 = cross_brace1_outer.cut(cross_brace1_inner)

cross_brace2_outer = (
    cq.Workplane("YZ")
    .workplane(offset=-upper_leg_tube_spacing / 2)
    .circle(upper_leg_tube_outer_diameter / 2)
    .extrude(upper_leg_tube_spacing)
    .translate((0, 0, z_hip_pitch_axis - upper_leg_cross_brace_positions[1]))
)

cross_brace2_inner = (
    cq.Workplane("YZ")
    .workplane(offset=-upper_leg_tube_spacing / 2 - 0.5)
    .circle(upper_leg_tube_inner_diameter / 2)
    .extrude(upper_leg_tube_spacing + 1)
    .translate((0, 0, z_hip_pitch_axis - upper_leg_cross_brace_positions[1]))
)

cross_brace2 = cross_brace2_outer.cut(cross_brace2_inner)

# Combine upper leg components
upper_leg = upper_leg_tube1.union(upper_leg_tube2).union(cross_brace1).union(cross_brace2)

# Knee pitch housing (cylindrical shell, oriented along Y axis)
knee_pitch_outer = (
    cq.Workplane("XZ")
    .circle(knee_pitch_outer_diameter / 2)
    .extrude(knee_pitch_housing_length)
    .translate((0, -knee_pitch_housing_length / 2, z_knee_axis))
)

knee_pitch_inner = (
    cq.Workplane("XZ")
    .circle(knee_pitch_inner_diameter / 2)
    .extrude(knee_pitch_housing_length + 1)
    .translate((0, -knee_pitch_housing_length / 2 - 0.5, z_knee_axis))
)

knee_pitch_housing = knee_pitch_outer.cut(knee_pitch_inner)

# Lower leg (single tube)
lower_leg_outer = (
    cq.Workplane("XY")
    .workplane(offset=z_knee_axis)
    .circle(lower_leg_outer_diameter / 2)
    .extrude(-lower_leg_length)
)

lower_leg_inner = (
    cq.Workplane("XY")
    .workplane(offset=z_knee_axis)
    .circle(lower_leg_inner_diameter / 2)
    .extrude(-lower_leg_length - 1)
)

lower_leg = lower_leg_outer.cut(lower_leg_inner)

# Foot (sphere)
foot = (
    cq.Workplane("XY")
    .sphere(foot_radius)
    .translate((0, 0, z_foot_center))
)

# ============================================================
# DIMENSION CHECK
# ============================================================
# Expected bounding box:
# X: ±(max of: hip_plate_width/2=40, upper_leg_tube_spacing/2 + tube_radius = 15+7.5 = 22.5,
#     knee_diameter/2 = 52, lower_leg_diameter/2 = 15, foot_radius = 25)
#   = ±52mm (from knee/hip pitch housings)
# Y: ±(max of: hip_plate_depth/2=30, hip_pitch_housing_length/2=27.5, knee_housing_length/2=27.5)
#   = ±30mm (from hip plate depth)
# Z: from z_body_interface (0) to z_foot_bottom
#   = 0 to z_foot_bottom

expected_x_extent = max(
    hip_plate_width / 2,
    upper_leg_tube_spacing / 2 + upper_leg_tube_outer_diameter / 2,
    hip_pitch_outer_diameter / 2,
    knee_pitch_outer_diameter / 2,
    lower_leg_outer_diameter / 2,
    foot_radius
)

expected_y_extent = max(
    hip_plate_depth / 2,
    hip_pitch_housing_length / 2,
    knee_pitch_housing_length / 2
)

expected_z_min = z_foot_bottom
expected_z_max = z_body_interface

print(f"Expected X extent: ±{expected_x_extent:.1f} mm")
print(f"Expected Y extent: ±{expected_y_extent:.1f} mm")
print(f"Expected Z range: {expected_z_min:.1f} to {expected_z_max:.1f} mm")
print(f"Expected total height: {abs(expected_z_min - expected_z_max):.1f} mm")
print(f"Target total height: ~455 mm")

total_height = abs(z_foot_bottom - z_body_interface)
if abs(total_height - 455) > 45:  # Allow 10% tolerance
    print(f"WARNING: Height mismatch - got {total_height:.1f} mm, expected ~455 mm")

# ============================================================
# ASSEMBLY
# ============================================================
assy = cq.Assembly()

# Add all components with descriptive names
assy.add(body_plate, name="body_plate", color=cq.Color("gray"))
assy.add(hip_mounting_plate, name="hip_mounting_plate", color=cq.Color("white"))
assy.add(hip_yaw_housing, name="hip_yaw_housing", color=cq.Color("blue"))
assy.add(u_bracket, name="u_bracket", color=cq.Color("gray"))
assy.add(hip_pitch_housing, name="hip_pitch_housing", color=cq.Color("blue"))
assy.add(upper_leg, name="upper_leg", color=cq.Color("orange"))
assy.add(knee_pitch_housing, name="knee_pitch_housing", color=cq.Color("blue"))
assy.add(lower_leg, name="lower_leg", color=cq.Color("yellow"))
assy.add(foot, name="foot", color=cq.Color("black"))

# ============================================================
# EXPORT
# ============================================================
assy.save("/home/deploy/cleanwalkerrobotics/hardware/cad-agent/models/model.step")
compound = assy.toCompound()
cq.exporters.export(compound, "/home/deploy/cleanwalkerrobotics/hardware/cad-agent/models/model.stl")
print("Export complete: models/model.step, models/model.stl")
