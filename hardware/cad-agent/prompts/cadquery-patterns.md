# CadQuery Patterns Reference

Proven patterns and known pitfalls for the CAD agent pipeline. **Read this before generating any CadQuery code.**

---

## Critical Rules (NEVER violate)

### 1. NEVER sweep() on a closed wire path

```python
# ❌ WRONG — creates a SOLID SLAB, not a pipe
path = cq.Workplane("XY").rect(width, depth)  # closed rectangle
profile = cq.Workplane("YZ").circle(tube_r)
frame = path.sweep(profile)  # FILLS THE INTERIOR → solid block

# ✅ CORRECT — build individual tubes and union
r = tube_diameter / 2
front = cq.Workplane("XZ").move(-w/2 - r, 0).circle(r).extrude(w + 2*r)
back  = cq.Workplane("XZ").move(-w/2 - r, 0).circle(r).extrude(w + 2*r).translate((0, d, 0))
left  = cq.Workplane("XY").move(-w/2, -r).circle(r).extrude(d + 2*r)
right = cq.Workplane("XY").move(w/2, -r).circle(r).extrude(d + 2*r)
frame = front.union(back).union(left).union(right)
```

**Why:** CadQuery's `sweep()` along a closed wire creates a solid volume filling the enclosed area. This is by design (it's a loft-like operation), not a bug. For tubular frames, always use individual tubes.

**Corner connections:** Extend each tube by `tube_radius` at both ends. The overlapping cylinder ends fuse during `union()`, creating solid corner joints.

### 2. NEVER sketch at offset, rotate around origin, then translate by offset

```python
# ❌ WRONG — double-offset: Y=150 applied twice
bar = (
    cq.Workplane("XY")
    .move(x_pos, 150)          # sketch at Y=hinge_y
    .circle(r).extrude(length)
    .rotate((0,0,0), (1,0,0), angle)  # rotates the Y=150 offset
    .translate((0, 150, height))       # adds Y=150 again!
)

# ✅ CORRECT — build at origin, rotate, translate ONCE
bar = (
    cq.Workplane("XY")
    .move(x_pos, 0)            # Y=0, only X offset
    .circle(r).extrude(length)
    .rotate((0,0,0), (1,0,0), angle)
    .translate((0, 150, height))  # single translation
)
```

**Why:** `.rotate()` rotates ALL coordinates around the given origin. If the geometry is at Y=150 and you rotate around (0,0,0), the Y=150 offset gets rotated into Y and Z components. Then `.translate()` adds another Y=150, resulting in a displaced part.

### 3. NEVER define a parameter without using it

Every variable in the PARAMETERS section MUST appear in the CONSTRUCTION section. If you define `servo_clearance_width = 15.0`, there MUST be a construction step that references it. Unused parameters = missing features.

**Self-check:** After writing the code, Ctrl+F each parameter name. If it only appears in the PARAMETERS section, you forgot to build something.

---

## Proven Patterns

### Rectangular tubular frames

For any frame made of circular-cross-section tubing:

```python
r = tube_diameter / 2  # extend each tube by radius for corner overlap

# Front tube (along X)
front = cq.Workplane("XZ").move(-width/2 - r, 0).circle(r).extrude(width + 2*r)

# Back tube (along X, offset in Y)
back = cq.Workplane("XZ").move(-width/2 - r, 0).circle(r).extrude(width + 2*r).translate((0, depth, 0))

# Left tube (along Y)
left = cq.Workplane("XY").move(-width/2, -r).circle(r).extrude(depth + 2*r)

# Right tube (along Y)
right = cq.Workplane("XY").move(width/2, -r).circle(r).extrude(depth + 2*r)

# Union creates solid corners
frame = front.union(back).union(left).union(right)
```

### L-shaped brackets

Use polyline for any non-rectangular profile:

```python
bracket = (
    cq.Workplane("XZ")
    .move(x_center, 0)
    .polyline([
        (0, 0),                              # base origin
        (foot_width, 0),                     # foot extends right
        (foot_width, foot_height),           # inner corner
        (wall_thickness, foot_height),       # junction
        (wall_thickness, total_height),      # tab top
        (0, total_height),                   # outer edge
    ])
    .close()
    .extrude(depth)
    .translate((0, y_offset, 0))
)
```

**Mirror for opposite side:** Negate the X coordinates in the polyline.

### Curved cradles / recesses

Create a box, then cut a cylinder to create a curved channel:

```python
# Rectangular base
housing = (
    cq.Workplane("XY")
    .move(0, center_y)
    .rect(housing_width, housing_depth)
    .extrude(housing_height)
)

# Cylindrical cutter (slightly larger than roll for clearance)
cutter = (
    cq.Workplane("YZ")
    .move(center_y, housing_height + roll_radius - wall_thickness)
    .circle(roll_radius + 0.5)  # 0.5mm clearance
    .extrude(roll_width)
    .translate((-roll_width/2, 0, 0))
)

housing = housing.cut(cutter)
```

### Angled components (support bars, tilted frames)

Always build at origin, rotate, translate:

```python
import math

# Build bar at origin pointing up along Z
bar = (
    cq.Workplane("XY")
    .move(x_offset, 0)  # only lateral offset, Y=0
    .circle(bar_diameter / 2)
    .extrude(bar_length)
)

# Rotate around X axis to tilt backward
bar = bar.rotate((0, 0, 0), (1, 0, 0), angle_deg - 90)

# Translate to mounting point (single translation)
bar = bar.translate((0, hinge_y, hinge_height))

# Compute endpoint for next component attachment
end_y = hinge_y + bar_length * math.cos(math.radians(angle_deg) - math.pi/2)
end_z = hinge_height + bar_length * math.sin(math.radians(angle_deg) - math.pi/2)
```

### Multi-component assemblies

Use `cq.Assembly()` for models with distinct parts:

```python
assy = cq.Assembly()
assy.add(body_plate, name="body_plate", color=cq.Color(0.15, 0.15, 0.15))
assy.add(housing, name="housing", color=cq.Color(0.2, 0.2, 0.2))
assy.add(roll, name="roll", color=cq.Color(0.8, 0.8, 0.8))
# ... more components

# Export
assy.save("models/model.step")       # preserves component structure + colors
compound = assy.toCompound()
cq.exporters.export(compound, "models/model.stl")  # single mesh for rendering
```

### Evenly spaced features (clips, holes, mounting points)

**Always compute spacing per target width:**

```python
# ❌ WRONG — reusing spacing from a different-width rail
clip_spacing = roll_width / (count + 1)  # 26mm
# ... later, on a 150mm-wide frame:
x = -frame_width/2 + clip_spacing * (i + 1)  # off-center!

# ✅ CORRECT — compute per rail
inner_spacing = roll_width / (count + 1)    # for 130mm rail
outer_spacing = frame_width / (count + 1)   # for 150mm rail
```

### Body plate for mounting context

Always include a reference plate when the component mounts to the robot body:

```python
body_plate = (
    cq.Workplane("XY")
    .move(0, plate_length/2 - offset)
    .rect(body_width, plate_length)
    .extrude(-5)  # 5mm thick below Z=0
)
assy.add(body_plate, name="body_plate", color=cq.Color(0.15, 0.15, 0.15))
```

This helps the evaluator see component positioning relative to the robot body.

### Boolean cuts (voids, clearances)

**Always verify the cutter overlaps the target:**

```python
# Before cutting, check coordinates overlap
# Target bracket: X = -45 to -20
# Cutter center: X = -37.5, width 15 → X = -45 to -30
# Overlap: X = -45 to -30 ✓

void = (
    cq.Workplane("XY")
    .move(target_x + clearance_width/2, target_y)  # positioned TO OVERLAP
    .rect(clearance_width, clearance_depth)
    .extrude(clearance_height)
)
target = target.cut(void)
```

### Dimension pre-validation

Include a comment block before the assembly section:

```python
# DIMENSION CHECK
# Front extent: Y ≈ roll_center_y - roll_diameter/2 - wall = -43mm
# Rear extent:  Y ≈ hinge_y + bar_length*cos(45°) + frame_depth*cos(45°) = 334mm
# Total depth:  334 - (-43) = 377mm → target 370mm ✓ (within 2%)
# Height:       Z ≈ hinge_height + bar_length*sin(45°) + frame_depth*sin(45°) = 259mm
# Width:        X ≈ max(frame_width, hinge_pin_length) = 160mm
```

---

## try/except Best Practices

Wrap complex geometry operations but keep fallbacks simple:

```python
try:
    # Attempt the correct geometry
    bracket = (
        cq.Workplane("XZ")
        .polyline([...]).close().extrude(depth)
    )
except Exception as e:
    print(f"Warning: Bracket construction error: {e}")
    # Fallback to simpler geometry (will fail eval but won't crash)
    bracket = (
        cq.Workplane("XY")
        .rect(width, depth).extrude(height)
    )
```

**Never silently swallow errors.** Always `print()` the warning so it appears in the execution report.

---

## Common CadQuery Errors and Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| `ValueError: No pending wires` | Face selector returned nothing | Try different selector (e.g., `">Y"` instead of `">Z"`) |
| `StdFail_NotDone` | Boolean op failed (fillet too large, bad geometry) | Reduce fillet radius, simplify geometry |
| `Standard_NullObject` | Operation on empty/invalid workplane | Check previous operations completed |
| `sweep() fills interior` | Closed wire path | Use individual extrusions + union |
| `.hole()` drills wrong direction | Selected wrong face | Verify face normal matches intended hole direction |

---

## Checklist Before Submitting model.py

1. [ ] All parameters from design_description.md have corresponding variables
2. [ ] All parameter variables are used in construction (grep check)
3. [ ] No `sweep()` on closed wire paths
4. [ ] All `.move()` coordinates are correct (no double-offset with rotate+translate)
5. [ ] Spacing variables computed per target width (not reused from different context)
6. [ ] DIMENSION CHECK comment block with expected bounding box
7. [ ] Both STEP and STL exports present
8. [ ] Body plate included (if component mounts to robot body)
9. [ ] try/except around complex geometry operations
10. [ ] Print statement confirming export success
