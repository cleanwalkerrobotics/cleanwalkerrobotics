"""
CAD Model: CW-1 Full Robot Assembly
Description: Complete robot assembly integrating all 4 subsystems —
             body chassis + head, 4 legs, arm + gripper, bag system.
             All subsystems positioned via body chassis mount interfaces.
Iteration: 1
"""
import cadquery as cq
import math
import os

# Ensure output directory exists
os.makedirs("models/full-assembly", exist_ok=True)

# ============================================================
# ASSEMBLY-LEVEL PARAMETERS (all dimensions in mm)
# ============================================================

# Body chassis reference dimensions
body_length = 600.0
body_width = 150.0
body_height = 120.0
half_width = body_width / 2   # 75
half_height = body_height / 2  # 60

# Leg mount positions (center of mount plates on body chassis)
# Legs attach to bottom of chassis; Z=-60 is the top face of the mount plates
leg_mount_positions = {
    "FL": (half_width, 250, -half_height),
    "FR": (-half_width, 250, -half_height),
    "RL": (half_width, -250, -half_height),
    "RR": (-half_width, -250, -half_height),
}

# Arm transform: rotate 180 deg about Z, then translate
# Maps arm turret (originally at Y=100) to chassis turret (Y=150, Z=60)
arm_rotation_deg = 180.0
arm_translate = (0, 250, half_height)

# Bag transform: rotate 180 deg about Z, then translate
# Maps bag hinge (originally at Y=600) to chassis rear (Y=-300, Z=60)
bag_rotation_deg = 180.0
bag_translate = (0, 300, half_height)

# ============================================================
# SUBSYSTEM 1: BODY CHASSIS + HEAD (18 components at origin)
# ============================================================

def build_body_chassis():
    """Build complete body chassis centered at origin.
    Returns list of (geometry, name, color) tuples."""
    parts = []

    # Parameters
    tube_size = 20.0
    panel_thickness = 3.0
    side_panel_thickness = 3.0

    head_depth = 80.0
    head_width_rear = 150.0
    head_height_front = 80.0

    led_panel_size = 30.0
    led_panel_thickness = 3.0
    led_panel_spacing = 50.0

    lidar_od = 60.0
    lidar_height = 15.0

    antenna_od = 10.0
    antenna_height = 20.0
    antenna_x_offset = 60.0
    antenna_y_offset = -50.0

    leg_mount_width = 80.0
    leg_mount_depth = 60.0
    leg_mount_thickness = 5.0
    leg_mount_bolt_pattern_x = 65.0
    leg_mount_bolt_pattern_y = 45.0
    leg_mount_bolt_dia = 5.0

    turret_od = 90.0
    turret_id = 60.0
    turret_thickness = 5.0
    turret_bolt_circle = 70.0
    turret_bolt_dia = 4.0
    turret_y_position = 150.0

    hinge_mount_width = 30.0
    hinge_mount_depth = 20.0
    hinge_mount_thickness = 5.0

    hinge_mount_x_offset = 45.0

    arm_turret_cutout_dia = 85.0
    lidar_cutout_dia = 50.0
    battery_cutout_width = 200.0
    battery_cutout_depth = 100.0
    battery_cutout_y = -100.0
    charging_port_dia = 25.0
    charging_port_y = -200.0

    hw = half_width
    hh = half_height

    # --- Chassis frame (union of 16 tubes) ---
    rail_bl_left = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((hw, 0, -hh))
    rail_bl_right = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((-hw, 0, -hh))

    cross_positions_y = [-body_length / 2, -100, 100, body_length / 2]
    bottom_crosses = []
    for y_pos in cross_positions_y:
        cross = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, y_pos, -hh))
        bottom_crosses.append(cross)

    corner_positions = [
        (hw, body_length / 2), (hw, -body_length / 2),
        (-hw, body_length / 2), (-hw, -body_length / 2)
    ]
    vertical_posts = []
    for x_pos, y_pos in corner_positions:
        post = cq.Workplane("XY").box(tube_size, tube_size, body_height).translate((x_pos, y_pos, 0))
        vertical_posts.append(post)

    rail_tl_left = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((hw, 0, hh))
    rail_tl_right = cq.Workplane("XY").box(tube_size, body_length, tube_size).translate((-hw, 0, hh))
    top_cross_front = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, body_length / 2, hh))
    top_cross_rear = cq.Workplane("XY").box(body_width, tube_size, tube_size).translate((0, -body_length / 2, hh))

    frame = rail_bl_left
    for c in [rail_bl_right] + bottom_crosses + vertical_posts + [
        rail_tl_left, rail_tl_right, top_cross_front, top_cross_rear
    ]:
        frame = frame.union(c)
    parts.append((frame, "chassis_frame", cq.Color(0.5, 0.5, 0.5)))

    # --- Top panel with cutouts ---
    top_panel = cq.Workplane("XY").box(body_width, body_length, panel_thickness).translate((0, 0, hh))
    try:
        top_panel = top_panel.faces(">Z").workplane().center(0, turret_y_position).circle(arm_turret_cutout_dia / 2).cutThruAll()
        top_panel = top_panel.faces(">Z").workplane().center(0, 0).circle(lidar_cutout_dia / 2).cutThruAll()
        top_panel = top_panel.faces(">Z").workplane().center(0, battery_cutout_y).rect(battery_cutout_width, battery_cutout_depth).cutThruAll()
    except Exception as e:
        print(f"Warning: Top panel cutouts: {e}")
    parts.append((top_panel, "top_panel", cq.Color(0.23, 0.29, 0.25)))

    # --- Bottom panel ---
    bottom_panel = cq.Workplane("XY").box(body_width, body_length, panel_thickness).translate((0, 0, -hh))
    try:
        bottom_panel = bottom_panel.faces("<Z").workplane().center(0, charging_port_y).circle(charging_port_dia / 2).cutThruAll()
    except Exception as e:
        print(f"Warning: Bottom panel cutout: {e}")
    parts.append((bottom_panel, "bottom_panel", cq.Color(0.2, 0.2, 0.2)))

    # --- Leg mount plates (x4) ---
    for label, (x_pos, y_pos) in [("FL", (hw, 250)), ("FR", (-hw, 250)),
                                    ("RL", (hw, -250)), ("RR", (-hw, -250))]:
        plate = (cq.Workplane("XY")
                 .box(leg_mount_width, leg_mount_depth, leg_mount_thickness)
                 .translate((x_pos, y_pos, -hh - leg_mount_thickness / 2)))
        try:
            bolt_pts = [
                (leg_mount_bolt_pattern_x / 2, leg_mount_bolt_pattern_y / 2),
                (leg_mount_bolt_pattern_x / 2, -leg_mount_bolt_pattern_y / 2),
                (-leg_mount_bolt_pattern_x / 2, leg_mount_bolt_pattern_y / 2),
                (-leg_mount_bolt_pattern_x / 2, -leg_mount_bolt_pattern_y / 2),
            ]
            plate = plate.faces(">Z").workplane().pushPoints(bolt_pts).circle(leg_mount_bolt_dia / 2).cutThruAll()
        except Exception as e:
            print(f"Warning: Leg mount {label} bolts: {e}")
        parts.append((plate, f"leg_mount_{label}", cq.Color(0.3, 0.3, 0.3)))

    # --- Arm turret mount ring ---
    turret_ring = (cq.Workplane("XY")
                   .circle(turret_od / 2).circle(turret_id / 2)
                   .extrude(turret_thickness)
                   .translate((0, turret_y_position, hh)))
    try:
        bolt_angles = [i * 90 for i in range(4)]
        bolt_pts = [(turret_bolt_circle / 2 * math.cos(math.radians(a)),
                     turret_bolt_circle / 2 * math.sin(math.radians(a))) for a in bolt_angles]
        turret_ring = turret_ring.faces(">Z").workplane().pushPoints(bolt_pts).circle(turret_bolt_dia / 2).cutThruAll()
    except Exception as e:
        print(f"Warning: Turret bolt holes: {e}")
    parts.append((turret_ring, "arm_turret_mount", cq.Color(0.3, 0.3, 0.3)))

    # --- Bag hinge mounts (x2) ---
    for label, x_off in [("left", hinge_mount_x_offset), ("right", -hinge_mount_x_offset)]:
        hm = (cq.Workplane("XY")
              .box(hinge_mount_width, hinge_mount_depth, hinge_mount_thickness)
              .translate((x_off, -body_length / 2, hh + hinge_mount_thickness / 2)))
        parts.append((hm, f"hinge_mount_{label}", cq.Color(0.3, 0.3, 0.3)))

    # --- Head module ---
    head_box = (cq.Workplane("XY")
                .box(head_width_rear, head_depth, head_height_front)
                .translate((0, body_length / 2 + head_depth / 2, -20)))
    parts.append((head_box, "head_module", cq.Color(0.23, 0.29, 0.25)))

    # --- LED eye panels (x2) ---
    for label, x_sign in [("left", 1), ("right", -1)]:
        led = (cq.Workplane("XY")
               .box(led_panel_size, led_panel_thickness, led_panel_size)
               .translate((x_sign * led_panel_spacing / 2, body_length / 2 + head_depth, 10)))
        parts.append((led, f"led_{label}", cq.Color(0.13, 0.77, 0.37)))

    # --- LiDAR mount ---
    lidar = (cq.Workplane("XY")
             .circle(lidar_od / 2).extrude(lidar_height)
             .translate((0, 0, hh + panel_thickness + lidar_height / 2)))
    parts.append((lidar, "lidar_mount", cq.Color(0.15, 0.15, 0.15)))

    # --- Antenna nub ---
    ant = (cq.Workplane("XY")
           .circle(antenna_od / 2).extrude(antenna_height)
           .translate((antenna_x_offset, antenna_y_offset, hh + panel_thickness + antenna_height / 2)))
    parts.append((ant, "antenna", cq.Color(0.1, 0.1, 0.1)))

    # --- Side panels (x2) ---
    for label, x_sign in [("left", 1), ("right", -1)]:
        sp = (cq.Workplane("YZ")
              .box(body_length, body_height, side_panel_thickness)
              .translate((x_sign * (hw + side_panel_thickness / 2), 0, 0)))
        parts.append((sp, f"side_panel_{label}", cq.Color(0.23, 0.29, 0.25)))

    return parts


# ============================================================
# SUBSYSTEM 2: LEG MODULE (single front-left leg at origin)
# ============================================================

def build_leg():
    """Build one leg module with body interface at Z=0.
    Extends downward to Z ~ -475. Excludes body_plate reference.
    Returns list of (geometry, name, color) tuples."""
    parts = []

    # Parameters
    hip_plate_width = 80.0
    hip_plate_depth = 60.0
    hip_plate_thickness = 5.0
    hip_plate_bolt_pattern_width = 65.0
    hip_plate_bolt_pattern_depth = 45.0
    hip_plate_bolt_hole_diameter = 6.0
    hip_plate_cable_hole_diameter = 30.0

    hip_yaw_outer_diameter = 82.0
    hip_yaw_inner_diameter = 77.0
    hip_yaw_height = 45.0

    hip_pitch_outer_diameter = 104.0
    hip_pitch_inner_diameter = 99.0
    hip_pitch_housing_length = 55.0

    bracket_thickness = 3.0
    bracket_width = 40.0
    bracket_height = 60.0

    upper_leg_tube_outer_diameter = 15.0
    upper_leg_tube_inner_diameter = 12.0
    upper_leg_tube_spacing = 30.0
    upper_leg_length = 200.0

    knee_pitch_outer_diameter = 104.0
    knee_pitch_inner_diameter = 99.0
    knee_pitch_housing_length = 55.0

    lower_leg_outer_diameter = 30.0
    lower_leg_inner_diameter = 25.0
    lower_leg_length = 200.0

    foot_radius = 25.0

    # Z-stack positions (body interface at Z=0, downward is negative)
    z_body_interface = 0.0
    z_mounting_plate_bottom = z_body_interface - hip_plate_thickness
    z_hip_yaw_top = z_mounting_plate_bottom
    z_hip_yaw_bottom = z_hip_yaw_top - hip_yaw_height
    z_hip_pitch_axis = z_hip_yaw_bottom
    z_knee_axis = z_hip_pitch_axis - upper_leg_length
    z_foot_center = z_knee_axis - lower_leg_length

    # --- Hip mounting plate ---
    hip_plate = (cq.Workplane("XY")
                 .box(hip_plate_width, hip_plate_depth, hip_plate_thickness,
                      centered=(True, True, False))
                 .translate((0, 0, z_body_interface)))
    try:
        hip_plate = (hip_plate.faces(">Z").workplane().pushPoints([
            (hip_plate_bolt_pattern_width / 2, hip_plate_bolt_pattern_depth / 2),
            (-hip_plate_bolt_pattern_width / 2, hip_plate_bolt_pattern_depth / 2),
            (hip_plate_bolt_pattern_width / 2, -hip_plate_bolt_pattern_depth / 2),
            (-hip_plate_bolt_pattern_width / 2, -hip_plate_bolt_pattern_depth / 2),
        ]).hole(hip_plate_bolt_hole_diameter))
        hip_plate = hip_plate.faces(">Z").workplane().hole(hip_plate_cable_hole_diameter)
    except Exception as e:
        print(f"Warning: Hip plate holes: {e}")
    parts.append((hip_plate, "hip_plate", cq.Color(0.8, 0.8, 0.8)))

    # --- Hip yaw housing (cylindrical shell) ---
    hip_yaw_outer = (cq.Workplane("XY")
                     .circle(hip_yaw_outer_diameter / 2)
                     .extrude(-hip_yaw_height)
                     .translate((0, 0, z_hip_yaw_top)))
    hip_yaw_inner = (cq.Workplane("XY")
                     .circle(hip_yaw_inner_diameter / 2)
                     .extrude(-hip_yaw_height - 1)
                     .translate((0, 0, z_hip_yaw_top)))
    hip_yaw_housing = hip_yaw_outer.cut(hip_yaw_inner)
    parts.append((hip_yaw_housing, "hip_yaw", cq.Color(0.2, 0.3, 0.8)))

    # --- U-bracket ---
    u_bracket = (cq.Workplane("XZ")
                 .center(0, z_hip_pitch_axis)
                 .rect(bracket_width, bracket_height)
                 .extrude(bracket_thickness)
                 .translate((0, -bracket_thickness / 2, 0)))
    parts.append((u_bracket, "u_bracket", cq.Color(0.5, 0.5, 0.5)))

    # --- Hip pitch housing (Y-oriented cylinder) ---
    hp_outer = (cq.Workplane("XZ")
                .circle(hip_pitch_outer_diameter / 2)
                .extrude(hip_pitch_housing_length)
                .translate((0, -hip_pitch_housing_length / 2, z_hip_pitch_axis)))
    hp_inner = (cq.Workplane("XZ")
                .circle(hip_pitch_inner_diameter / 2)
                .extrude(hip_pitch_housing_length + 1)
                .translate((0, -hip_pitch_housing_length / 2 - 0.5, z_hip_pitch_axis)))
    hip_pitch_housing = hp_outer.cut(hp_inner)
    parts.append((hip_pitch_housing, "hip_pitch", cq.Color(0.2, 0.3, 0.8)))

    # --- Upper leg (2 parallel tubes + 2 cross braces) ---
    ul_tube1_o = (cq.Workplane("XY").workplane(offset=z_hip_pitch_axis)
                  .circle(upper_leg_tube_outer_diameter / 2)
                  .extrude(-upper_leg_length)
                  .translate((upper_leg_tube_spacing / 2, 0, 0)))
    ul_tube1_i = (cq.Workplane("XY").workplane(offset=z_hip_pitch_axis)
                  .circle(upper_leg_tube_inner_diameter / 2)
                  .extrude(-upper_leg_length - 1)
                  .translate((upper_leg_tube_spacing / 2, 0, 0)))
    ul_tube1 = ul_tube1_o.cut(ul_tube1_i)

    ul_tube2_o = (cq.Workplane("XY").workplane(offset=z_hip_pitch_axis)
                  .circle(upper_leg_tube_outer_diameter / 2)
                  .extrude(-upper_leg_length)
                  .translate((-upper_leg_tube_spacing / 2, 0, 0)))
    ul_tube2_i = (cq.Workplane("XY").workplane(offset=z_hip_pitch_axis)
                  .circle(upper_leg_tube_inner_diameter / 2)
                  .extrude(-upper_leg_length - 1)
                  .translate((-upper_leg_tube_spacing / 2, 0, 0)))
    ul_tube2 = ul_tube2_o.cut(ul_tube2_i)

    brace_z_offsets = [upper_leg_length / 3, 2 * upper_leg_length / 3]
    braces = []
    for bz in brace_z_offsets:
        cb_o = (cq.Workplane("YZ").workplane(offset=-upper_leg_tube_spacing / 2)
                .circle(upper_leg_tube_outer_diameter / 2)
                .extrude(upper_leg_tube_spacing)
                .translate((0, 0, z_hip_pitch_axis - bz)))
        cb_i = (cq.Workplane("YZ").workplane(offset=-upper_leg_tube_spacing / 2 - 0.5)
                .circle(upper_leg_tube_inner_diameter / 2)
                .extrude(upper_leg_tube_spacing + 1)
                .translate((0, 0, z_hip_pitch_axis - bz)))
        braces.append(cb_o.cut(cb_i))

    upper_leg = ul_tube1.union(ul_tube2)
    for b in braces:
        upper_leg = upper_leg.union(b)
    parts.append((upper_leg, "upper_leg", cq.Color(0.9, 0.5, 0.1)))

    # --- Knee pitch housing ---
    kp_outer = (cq.Workplane("XZ")
                .circle(knee_pitch_outer_diameter / 2)
                .extrude(knee_pitch_housing_length)
                .translate((0, -knee_pitch_housing_length / 2, z_knee_axis)))
    kp_inner = (cq.Workplane("XZ")
                .circle(knee_pitch_inner_diameter / 2)
                .extrude(knee_pitch_housing_length + 1)
                .translate((0, -knee_pitch_housing_length / 2 - 0.5, z_knee_axis)))
    knee_pitch_housing = kp_outer.cut(kp_inner)
    parts.append((knee_pitch_housing, "knee", cq.Color(0.2, 0.3, 0.8)))

    # --- Lower leg (single tube) ---
    ll_outer = (cq.Workplane("XY").workplane(offset=z_knee_axis)
                .circle(lower_leg_outer_diameter / 2)
                .extrude(-lower_leg_length))
    ll_inner = (cq.Workplane("XY").workplane(offset=z_knee_axis)
                .circle(lower_leg_inner_diameter / 2)
                .extrude(-lower_leg_length - 1))
    lower_leg = ll_outer.cut(ll_inner)
    parts.append((lower_leg, "lower_leg", cq.Color(0.8, 0.7, 0.1)))

    # --- Foot (sphere) ---
    foot = cq.Workplane("XY").sphere(foot_radius).translate((0, 0, z_foot_center))
    parts.append((foot, "foot", cq.Color(0.1, 0.1, 0.1)))

    return parts


# ============================================================
# SUBSYSTEM 3: ARM + GRIPPER (at original coords, no body plate)
# ============================================================

def build_arm():
    """Build 5-DOF arm at original coordinates (turret at Y=100).
    Excludes body plate reference. Will be rotated+translated for assembly.
    Returns list of (geometry, name, color) tuples."""
    parts = []

    # Parameters
    turret_od = 80.0
    turret_id = 60.0
    turret_height = 50.0
    turret_y = 100.0

    shoulder_width = 50.0
    shoulder_height = 40.0
    shoulder_depth = 30.0
    shoulder_wall = 3.0

    upper_arm_length = 180.0
    arm_width = 30.0
    arm_depth = 30.0
    arm_wall = 2.0

    elbow_width = 50.0
    elbow_height = 35.0
    elbow_depth = 25.0
    elbow_wall = 3.0

    forearm_length = 180.0

    wrist_od = 40.0
    wrist_height = 50.0

    gripper_width = 40.0
    gripper_depth = 40.0
    gripper_height = 30.0

    finger_length = 70.0
    finger_width = 15.0
    finger_thickness = 5.0
    finger_open_angle = 30.0

    pad_length = 15.0
    pad_width = 10.0
    pad_thickness = 3.0

    shoulder_pitch = 90.0
    elbow_pitch = 30.0
    wrist_pitch = -30.0

    xm430_w = 46.5
    xm430_d = 36.0
    xm430_h = 34.0
    xl430_w = 28.5
    xl430_d = 46.5
    xl430_h = 34.0

    # --- Turret base (hollow cylinder) ---
    t_outer = cq.Workplane("XY").circle(turret_od / 2).extrude(turret_height)
    t_inner = cq.Workplane("XY").circle(turret_id / 2).extrude(turret_height)
    turret = t_outer.cut(t_inner).translate((0, turret_y, 0))
    parts.append((turret, "turret", cq.Color(0.2, 0.2, 0.2)))

    # --- Shoulder bracket (U-bracket) ---
    sb_solid = cq.Workplane("XY").box(shoulder_width, shoulder_depth, shoulder_height)
    sb_channel = (cq.Workplane("XY")
                  .box(shoulder_width - 2 * shoulder_wall, shoulder_depth + 2,
                       shoulder_height - shoulder_wall)
                  .translate((0, 0, -shoulder_wall / 2)))
    shoulder_bracket = sb_solid.cut(sb_channel)
    shoulder_bracket = shoulder_bracket.translate((0, turret_y, turret_height + shoulder_height / 2))
    parts.append((shoulder_bracket, "shoulder_bracket", cq.Color(0.25, 0.25, 0.25)))

    # Joint positions
    shoulder_joint_z = turret_height + shoulder_height
    shoulder_joint_pos = (0, turret_y, shoulder_joint_z)

    # --- Upper arm (rectangular tube, rotated to shoulder pitch) ---
    ua_outer = (cq.Workplane("XY")
                .box(arm_width, arm_depth, upper_arm_length)
                .translate((0, 0, upper_arm_length / 2)))
    ua_inner = (cq.Workplane("XY")
                .box(arm_width - 2 * arm_wall, arm_depth - 2 * arm_wall, upper_arm_length)
                .translate((0, 0, upper_arm_length / 2)))
    upper_arm = ua_outer.cut(ua_inner)
    upper_arm = upper_arm.rotate((0, 0, 0), (1, 0, 0), -shoulder_pitch)
    upper_arm = upper_arm.translate(shoulder_joint_pos)
    parts.append((upper_arm, "upper_arm", cq.Color(0.23, 0.29, 0.25)))

    ua_end_y = shoulder_joint_pos[1] - upper_arm_length * math.sin(math.radians(shoulder_pitch))
    ua_end_z = shoulder_joint_pos[2] + upper_arm_length * math.cos(math.radians(shoulder_pitch))
    upper_arm_end = (0, ua_end_y, ua_end_z)

    # --- Elbow bracket ---
    eb_solid = cq.Workplane("XY").box(elbow_width, elbow_depth, elbow_height)
    eb_channel = (cq.Workplane("XY")
                  .box(elbow_width - 2 * elbow_wall, elbow_depth + 2,
                       elbow_height - elbow_wall)
                  .translate((0, 0, -elbow_wall / 2)))
    cumulative_pitch_elbow = shoulder_pitch + elbow_pitch
    elbow_bracket = eb_solid.cut(eb_channel)
    elbow_bracket = elbow_bracket.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_elbow)
    elbow_bracket = elbow_bracket.translate(upper_arm_end)
    parts.append((elbow_bracket, "elbow_bracket", cq.Color(0.25, 0.25, 0.25)))

    # --- Forearm ---
    fa_outer = (cq.Workplane("XY")
                .box(arm_width, arm_depth, forearm_length)
                .translate((0, 0, forearm_length / 2)))
    fa_inner = (cq.Workplane("XY")
                .box(arm_width - 2 * arm_wall, arm_depth - 2 * arm_wall, forearm_length)
                .translate((0, 0, forearm_length / 2)))
    forearm = fa_outer.cut(fa_inner)
    forearm = forearm.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_elbow)
    forearm = forearm.translate(upper_arm_end)
    parts.append((forearm, "forearm", cq.Color(0.23, 0.29, 0.25)))

    fa_dy = -forearm_length * math.sin(math.radians(cumulative_pitch_elbow))
    fa_dz = forearm_length * math.cos(math.radians(cumulative_pitch_elbow))
    forearm_end = (0, upper_arm_end[1] + fa_dy, upper_arm_end[2] + fa_dz)

    # --- Wrist cylinder ---
    cumulative_pitch_wrist = cumulative_pitch_elbow + wrist_pitch
    wrist_cyl = cq.Workplane("XY").circle(wrist_od / 2).extrude(wrist_height)
    wrist_cyl = wrist_cyl.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
    wrist_cyl = wrist_cyl.translate(forearm_end)
    parts.append((wrist_cyl, "wrist", cq.Color(0.2, 0.2, 0.2)))

    w_dy = -wrist_height * math.sin(math.radians(cumulative_pitch_wrist))
    w_dz = wrist_height * math.cos(math.radians(cumulative_pitch_wrist))
    wrist_end = (0, forearm_end[1] + w_dy, forearm_end[2] + w_dz)

    # --- Gripper body ---
    gb = (cq.Workplane("XY")
          .box(gripper_width, gripper_depth, gripper_height)
          .translate((0, 0, gripper_height / 2)))
    gb = gb.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
    gb = gb.translate(wrist_end)
    parts.append((gb, "gripper_body", cq.Color(0.2, 0.2, 0.2)))

    gb_dy = -gripper_height * math.sin(math.radians(cumulative_pitch_wrist))
    gb_dz = gripper_height * math.cos(math.radians(cumulative_pitch_wrist))
    gripper_base = (0, wrist_end[1] + gb_dy, wrist_end[2] + gb_dz)

    # --- Fingers + pads ---
    half_angle = finger_open_angle / 2
    for label, x_sign in [("left", 1), ("right", -1)]:
        finger = (cq.Workplane("XY")
                  .box(finger_width, finger_thickness, finger_length)
                  .translate((0, 0, finger_length / 2)))
        finger = finger.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
        finger = finger.rotate((0, 0, 0), (0, 1, 0), x_sign * half_angle)
        finger = finger.translate(gripper_base)
        parts.append((finger, f"finger_{label}", cq.Color(0.6, 0.6, 0.6)))

        pad = (cq.Workplane("XY")
               .box(pad_width, pad_thickness, pad_length)
               .translate((0, 0, finger_length - pad_length / 2)))
        pad = pad.rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
        pad = pad.rotate((0, 0, 0), (0, 1, 0), x_sign * half_angle)
        pad = pad.translate(gripper_base)
        parts.append((pad, f"pad_{label}", cq.Color(0.1, 0.1, 0.1)))

    # --- Actuator reference boxes ---
    xm_shoulder = (cq.Workplane("XY")
                   .box(xm430_w, xm430_d, xm430_h)
                   .translate((0, turret_y, turret_height + shoulder_height / 2)))
    parts.append((xm_shoulder, "servo_shoulder", cq.Color(0.8, 0.3, 0.3)))

    xm_elbow = (cq.Workplane("XY")
                .box(xm430_w, xm430_d, xm430_h)
                .rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_elbow)
                .translate(upper_arm_end))
    parts.append((xm_elbow, "servo_elbow", cq.Color(0.8, 0.3, 0.3)))

    xl_wrist = (cq.Workplane("XY")
                .box(xl430_w, xl430_d, xl430_h)
                .rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
                .translate(forearm_end))
    parts.append((xl_wrist, "servo_wrist", cq.Color(0.3, 0.3, 0.8)))

    xl_gripper = (cq.Workplane("XY")
                  .box(xl430_w, xl430_d, xl430_h)
                  .rotate((0, 0, 0), (1, 0, 0), -cumulative_pitch_wrist)
                  .translate(wrist_end))
    parts.append((xl_gripper, "servo_gripper", cq.Color(0.3, 0.8, 0.3)))

    return parts


# ============================================================
# SUBSYSTEM 4: BAG SYSTEM (at original coords, no body plate/turret)
# ============================================================

def build_bag():
    """Build bag system at original coordinates (hinge at Y=600).
    Excludes body plate and turret stub (context references only).
    Will be rotated+translated for assembly.
    Returns list of (geometry, name, color) tuples."""
    parts = []

    # Parameters
    body_w = 150.0

    roll_diameter = 80.0
    roll_width = 130.0
    roll_housing_height = 15.0
    roll_housing_wall = 3.0
    roll_center_y = 450.0

    roll_bracket_thickness = 3.0
    roll_bracket_height = 25.0
    roll_bracket_width = 20.0

    frame_tube_diameter = 10.0
    frame_width = 150.0
    frame_depth = 220.0
    frame_angle_deg = 135.0

    support_bar_diameter = 10.0
    support_bar_length = 91.0

    hinge_y = 600.0
    hinge_bracket_height = 15.0
    hinge_bracket_width = 25.0
    hinge_bracket_depth = 20.0
    hinge_bracket_thickness = 3.0
    hinge_bracket_foot_height = 3.0
    hinge_bracket_spacing = 30.0
    hinge_pin_diameter = 4.0
    hinge_pin_length = 160.0

    servo_clearance_width = 15.0
    servo_clearance_depth = 15.0
    servo_clearance_height = 30.0

    clip_count = 4
    clip_width = 20.0
    clip_height = 8.0
    clip_thickness = 2.0
    clip_standoff = 3.0

    # --- Roll housing (curved cradle) ---
    try:
        roll_housing = (cq.Workplane("XY")
                        .move(0, roll_center_y)
                        .rect(roll_width + 2 * roll_housing_wall,
                              roll_diameter / 2 + 2 * roll_housing_wall)
                        .extrude(roll_housing_height))
        cradle_cutter = (cq.Workplane("YZ")
                         .move(roll_center_y, roll_housing_height + roll_diameter / 2 - roll_housing_wall)
                         .circle(roll_diameter / 2 + 0.5)
                         .extrude(roll_width)
                         .translate((-roll_width / 2, 0, 0)))
        roll_housing = roll_housing.cut(cradle_cutter)
    except Exception as e:
        print(f"Warning: Roll housing cradle: {e}")
        roll_housing = (cq.Workplane("XY")
                        .move(0, roll_center_y)
                        .rect(roll_width + 2 * roll_housing_wall,
                              roll_diameter / 2 + roll_housing_wall)
                        .extrude(roll_housing_height))
    parts.append((roll_housing, "roll_housing", cq.Color(0.15, 0.15, 0.15)))

    # --- Roll cylinder ---
    roll_cyl = (cq.Workplane("YZ")
                .move(roll_center_y, roll_housing_height + roll_diameter / 2)
                .circle(roll_diameter / 2)
                .extrude(roll_width)
                .translate((-roll_width / 2, 0, 0)))
    parts.append((roll_cyl, "roll_cylinder", cq.Color(0.4, 0.4, 0.4)))

    # --- Roll side brackets ---
    for label, x_sign in [("left", -1), ("right", 1)]:
        bracket = (cq.Workplane("XY")
                   .move(x_sign * (roll_width / 2 + roll_housing_wall + roll_bracket_thickness / 2),
                         roll_center_y)
                   .rect(roll_bracket_thickness, roll_bracket_width)
                   .extrude(roll_bracket_height))
        parts.append((bracket, f"roll_bracket_{label}", cq.Color(0.2, 0.2, 0.2)))

    # --- Hinge brackets (L-shaped) ---
    try:
        left_hb = (cq.Workplane("XZ")
                   .move(-(body_w / 2 - hinge_bracket_spacing), 0)
                   .polyline([
                       (0, 0), (hinge_bracket_width, 0),
                       (hinge_bracket_width, hinge_bracket_foot_height),
                       (hinge_bracket_thickness, hinge_bracket_foot_height),
                       (hinge_bracket_thickness, hinge_bracket_height),
                       (0, hinge_bracket_height),
                   ]).close().extrude(hinge_bracket_depth)
                   .translate((0, hinge_y - hinge_bracket_depth / 2, 0)))
    except Exception as e:
        print(f"Warning: Left hinge bracket polyline: {e}")
        left_hb = (cq.Workplane("XY")
                   .move(-(body_w / 2 - hinge_bracket_spacing), hinge_y)
                   .rect(hinge_bracket_width, hinge_bracket_depth)
                   .extrude(hinge_bracket_height))

    try:
        servo_void = (cq.Workplane("XY")
                      .move(-(body_w / 2 - hinge_bracket_spacing) + servo_clearance_width / 2,
                            hinge_y)
                      .rect(servo_clearance_width, servo_clearance_depth)
                      .extrude(servo_clearance_height))
        left_hb = left_hb.cut(servo_void)
    except Exception as e:
        print(f"Warning: Servo void cut: {e}")
    parts.append((left_hb, "hinge_bracket_left", cq.Color(0.2, 0.2, 0.2)))

    try:
        right_hb = (cq.Workplane("XZ")
                    .move((body_w / 2 - hinge_bracket_spacing), 0)
                    .polyline([
                        (0, 0), (-hinge_bracket_width, 0),
                        (-hinge_bracket_width, hinge_bracket_foot_height),
                        (-hinge_bracket_thickness, hinge_bracket_foot_height),
                        (-hinge_bracket_thickness, hinge_bracket_height),
                        (0, hinge_bracket_height),
                    ]).close().extrude(hinge_bracket_depth)
                    .translate((0, hinge_y - hinge_bracket_depth / 2, 0)))
    except Exception as e:
        print(f"Warning: Right hinge bracket polyline: {e}")
        right_hb = (cq.Workplane("XY")
                    .move((body_w / 2 - hinge_bracket_spacing), hinge_y)
                    .rect(hinge_bracket_width, hinge_bracket_depth)
                    .extrude(hinge_bracket_height))
    parts.append((right_hb, "hinge_bracket_right", cq.Color(0.2, 0.2, 0.2)))

    # --- Hinge pin ---
    hinge_pin = (cq.Workplane("YZ")
                 .move(hinge_y, hinge_bracket_height)
                 .circle(hinge_pin_diameter / 2)
                 .extrude(hinge_pin_length)
                 .translate((-hinge_pin_length / 2, 0, 0)))
    parts.append((hinge_pin, "hinge_pin", cq.Color(0.6, 0.6, 0.6)))

    # --- Support bars ---
    frame_angle_rad = math.radians(frame_angle_deg)
    rot_angle = frame_angle_deg - 90  # 45 degrees

    support_bar_end_y = hinge_y + support_bar_length * math.cos(frame_angle_rad - math.pi / 2)
    support_bar_end_z = hinge_bracket_height + support_bar_length * math.sin(frame_angle_rad - math.pi / 2)

    for label, x_frac in [("left", -0.25), ("right", 0.25)]:
        bar = (cq.Workplane("XY")
               .move(x_frac * frame_width, 0)
               .circle(support_bar_diameter / 2)
               .extrude(support_bar_length))
        bar = bar.rotate((0, 0, 0), (1, 0, 0), rot_angle)
        bar = bar.translate((0, hinge_y, hinge_bracket_height))
        parts.append((bar, f"support_bar_{label}", cq.Color(0.25, 0.25, 0.25)))

    # --- Folding frame (4 cylinder tubes + union) ---
    frame_attach_y = support_bar_end_y
    frame_attach_z = support_bar_end_z
    r = frame_tube_diameter / 2

    frame_front = (cq.Workplane("XZ")
                   .move(-frame_width / 2 - r, 0)
                   .circle(r).extrude(frame_width + 2 * r))
    frame_back = (cq.Workplane("XZ")
                  .move(-frame_width / 2 - r, 0)
                  .circle(r).extrude(frame_width + 2 * r)
                  .translate((0, frame_depth, 0)))
    frame_left = (cq.Workplane("XY")
                  .move(-frame_width / 2, -r)
                  .circle(r).extrude(frame_depth + 2 * r))
    frame_right = (cq.Workplane("XY")
                   .move(frame_width / 2, -r)
                   .circle(r).extrude(frame_depth + 2 * r))

    frame = frame_front.union(frame_back).union(frame_left).union(frame_right)
    frame = frame.rotate((0, 0, 0), (1, 0, 0), rot_angle)
    frame = frame.translate((0, frame_attach_y, frame_attach_z))
    parts.append((frame, "frame", cq.Color(0.25, 0.25, 0.25)))

    # --- Inner clips (at roll front edge) ---
    inner_clip_spacing = roll_width / (clip_count + 1)
    inner_clips = None
    for i in range(clip_count):
        x_pos = -roll_width / 2 + inner_clip_spacing * (i + 1)
        clip = (cq.Workplane("XY")
                .move(x_pos, roll_center_y - roll_diameter / 2 - roll_housing_wall)
                .rect(clip_width, clip_thickness)
                .extrude(clip_height)
                .translate((0, 0, clip_standoff)))
        inner_clips = clip if inner_clips is None else inner_clips.union(clip)
    parts.append((inner_clips, "inner_clips", cq.Color(0.6, 0.6, 0.6)))

    # --- Outer clips (at frame far rim) ---
    frame_back_y = frame_attach_y + frame_depth * math.cos(frame_angle_rad - math.pi / 2)
    frame_back_z = frame_attach_z + frame_depth * math.sin(frame_angle_rad - math.pi / 2)
    outer_clip_spacing = frame_width / (clip_count + 1)
    outer_clips = None
    for i in range(clip_count):
        x_pos = -frame_width / 2 + outer_clip_spacing * (i + 1)
        clip = (cq.Workplane("XY")
                .move(x_pos, 0)
                .rect(clip_width, clip_thickness)
                .extrude(clip_height))
        clip = clip.rotate((0, 0, 0), (1, 0, 0), rot_angle)
        clip = clip.translate((0, frame_back_y, frame_back_z))
        outer_clips = clip if outer_clips is None else outer_clips.union(clip)
    parts.append((outer_clips, "outer_clips", cq.Color(0.6, 0.6, 0.6)))

    return parts


# ============================================================
# BUILD ALL SUBSYSTEMS
# ============================================================

print("=" * 60)
print("CW-1 Full Robot Assembly")
print("=" * 60)

print("\n[1/4] Building body chassis + head...")
body_parts = build_body_chassis()
print(f"       {len(body_parts)} components")

print("[2/4] Building leg module (template)...")
leg_parts = build_leg()
print(f"       {len(leg_parts)} components per leg, x4 = {4 * len(leg_parts)}")

print("[3/4] Building arm + gripper...")
arm_parts = build_arm()
print(f"       {len(arm_parts)} components")

print("[4/4] Building bag system...")
bag_parts = build_bag()
print(f"       {len(bag_parts)} components")

total_components = len(body_parts) + 4 * len(leg_parts) + len(arm_parts) + len(bag_parts)
print(f"\nTotal components: {total_components}")

# ============================================================
# ASSEMBLY
# ============================================================

print("\nAssembling...")
assy = cq.Assembly()

# --- Body chassis (at origin) ---
for geom, name, color in body_parts:
    assy.add(geom, name=name, color=color)

# --- 4 Legs (translated to mount points) ---
for label, (tx, ty, tz) in leg_mount_positions.items():
    for geom, name, color in leg_parts:
        placed = geom.translate((tx, ty, tz))
        assy.add(placed, name=f"leg_{label}_{name}", color=color)

# --- Arm + gripper (rotate 180 deg Z, then translate) ---
for geom, name, color in arm_parts:
    placed = (geom
              .rotate((0, 0, 0), (0, 0, 1), arm_rotation_deg)
              .translate(arm_translate))
    assy.add(placed, name=f"arm_{name}", color=color)

# --- Bag system (rotate 180 deg Z, then translate) ---
for geom, name, color in bag_parts:
    placed = (geom
              .rotate((0, 0, 0), (0, 0, 1), bag_rotation_deg)
              .translate(bag_translate))
    assy.add(placed, name=f"bag_{name}", color=color)

print("Assembly complete.")

# ============================================================
# DIMENSION CHECK
# ============================================================

compound = assy.toCompound()
bbox = compound.BoundingBox()

print(f"\n{'='*60}")
print("DIMENSION CHECK")
print(f"{'='*60}")
print(f"Bounding box:")
print(f"  X: [{bbox.xmin:.1f}, {bbox.xmax:.1f}] mm  (width:  {bbox.xmax - bbox.xmin:.1f} mm)")
print(f"  Y: [{bbox.ymin:.1f}, {bbox.ymax:.1f}] mm  (length: {bbox.ymax - bbox.ymin:.1f} mm)")
print(f"  Z: [{bbox.zmin:.1f}, {bbox.zmax:.1f}] mm  (height: {bbox.zmax - bbox.zmin:.1f} mm)")

# Expected ranges:
# Width (X):  ~260mm (body 150 + side panels + legs extend ±52mm from mount at ±75)
# Length (Y): ~1100mm (arm gripper at ~Y=566, bag frame back at ~Y=-520)
# Height (Z): ~830mm (feet at Z~-535, bag frame tip at Z~295)
#   Standing height (feet to body top): ~595mm
#   Body chassis: 120mm tall

print(f"\nSubsystem summary:")
print(f"  Body chassis: {len(body_parts)} parts")
print(f"  Legs:         4 x {len(leg_parts)} = {4*len(leg_parts)} parts")
print(f"  Arm+gripper:  {len(arm_parts)} parts")
print(f"  Bag system:   {len(bag_parts)} parts")
print(f"  TOTAL:        {total_components} parts")

# ============================================================
# EXPORT
# ============================================================

print(f"\nExporting...")
assy.save("models/full-assembly/model.step")
cq.exporters.export(compound, "models/full-assembly/model.stl")

step_size = os.path.getsize("models/full-assembly/model.step")
stl_size = os.path.getsize("models/full-assembly/model.stl")

print(f"\n{'='*60}")
print("EXPORT COMPLETE")
print(f"{'='*60}")
print(f"  STEP: models/full-assembly/model.step ({step_size/1024:.0f} KB)")
print(f"  STL:  models/full-assembly/model.stl ({stl_size/1024:.0f} KB)")
print(f"  Components: {total_components}")
print(f"{'='*60}")
