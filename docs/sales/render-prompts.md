# CleanWalker Robot — AI Render Prompts

**Purpose:** Generate photorealistic marketing renders for website, pitch deck, and sales materials.
**Tool:** Seedream 4.5 (bytedance/seedream-4.5) via Replicate API at 16:9 aspect ratio.
**Style:** Clean, professional, photorealistic product photography. Modern industrial robot marketing meets premium product shots.
**Design Spec:** See `docs/design/robot-design-spec.md` for canonical robot design (V2.2).

---

## Core Robot Description (MUST start every prompt)

Every render prompt begins with this exact canonical description (from V2.2 spec), then appends scene-specific details:

```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.
```

---

## Hero Shots (Website Homepage)

### 1. Hero — Robot in Park Setting
**File:** `hero-park.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. The robot is in a clean urban park, its mechanical gripper arm reaching down and picking up a plastic bottle from the grass. The heavy-duty black trash bag hanging from the rear folding frame is partially filled with collected litter. Morning golden hour lighting, shallow depth of field, green grass and trees in background. Photorealistic product photography, 8K, ultra detailed.
```

### 2. Hero — Robot on City Sidewalk
**File:** `hero-sidewalk.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. The robot is walking on a European city sidewalk with cobblestone street. Four legs in dynamic walking pose. Modern architecture in the background. Overcast sky, soft diffused lighting. Professional product photography, studio quality, 8K.
```

### 3. Hero — Fleet of Robots Working
**File:** `hero-fleet.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Wide-angle photo showing three of these identical robots working together in a large urban park, picking up litter. One robot is picking up a crushed can with its mechanical gripper arm, another is walking with its bag nearly full of litter, and the third has its folding frame folded down flat against the body, a sealed full bag dropped neatly at the curb behind it. Drone perspective, morning light, professional marketing photo. Ultra detailed, 8K.
```

---

## Product Detail Shots (Product Page)

### 4. Close-up — Gripper Picking Up Litter
**File:** `detail-gripper.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Extreme close-up focused on the robot's mechanical gripper with 2-3 metal fingers firmly grasping a crushed aluminum can. The gripper is attached to the single articulated robotic arm. In the background, slightly out of focus, the rear folding frame holds the open trash bag with collected litter visible inside. Shallow depth of field, studio lighting, product photography style, 8K macro photography.
```

### 5. Close-up — Sensor Array (Front Face)
**File:** `detail-sensors.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Front view close-up of the robot's sensor housing head module showing two square bright green LED eye panels on the flat front face. The matte dark olive-green body panels have clean industrial design with softly rounded chamfered edges. Behind and above, the rear folding frame is partially visible holding the open trash bag. Studio product photography with soft rim lighting, 8K.
```

### 6. Side Profile — Full Robot
**File:** `detail-side-profile.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Clean side-profile product photo on a white/light gray studio background. Standing pose, four articulated legs visible with all joints and actuator housings, the single robotic arm with mechanical gripper visible at front top, and the rectangular folding frame hinged open at 135 degrees from the rear of the body with the heavy-duty black trash bag hanging freely — the robot's most distinctive feature. On the back, the horizontal bag roll dispenser is visible between the arm mount and the frame hinge. Professional product photography, even lighting, no harsh shadows, 8K.
```

### 7. Robot at Charging Dock
**File:** `detail-charging-dock.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. The robot is parked on a small charging dock platform in an urban park setting. The dock is a simple weatherproof platform with a small rain canopy. Bright green LED strips glowing on the upper legs as a charging indicator. The rectangular folding frame at the rear is folded down flat against the body in compact transport mode, giving the robot a low sleek profile. Evening golden hour lighting, 8K.
```

---

## Lifestyle / Context Shots (About Page, Social Media)

### 8. Robot and City Worker
**File:** `lifestyle-city-worker.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. A city parks maintenance worker in a high-visibility vest stands next to the robot, checking a tablet showing fleet status. The robot is about knee-height, with its folding frame open and the heavy-duty black trash bag half-full of collected litter hanging behind it. They are in a well-maintained urban park. The worker looks pleased. Natural daylight, editorial photography style, 8K.
```

### 9. Night Operations
**File:** `lifestyle-night-ops.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. The robot is operating on a city street at night. The bright green LED strips on the upper legs illuminate the sidewalk around it. Two square green LED eye panels on the sensor housing head glow visibly. Street lamps in the background. The robot is mid-stride with its folding frame holding the open trash bag behind the body. The mechanical gripper arm is pressing a piece of litter down into the bag. Cinematic lighting, shallow depth of field, moody atmosphere, 8K.
```

### 10. Before/After — Park Clean-up
**File:** `lifestyle-before-after.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Split image composition: Left side shows a park area with scattered litter (plastic bottles, cans, food wrappers) on the ground. Right side shows the same park area pristine and clean, with the robot visible in the distance, its folding frame folded down flat against the body, a sealed full bag dropped neatly at the curb behind it. Before/after comparison, bright daylight, professional editorial photography, 8K.
```

---

## Technical / Pitch Deck Shots

### 11. Exploded View
**File:** `tech-exploded-view.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Technical exploded view diagram on a white background. The robot's components are floating in space showing: dark olive-green weatherproof enclosure panels with rounded edges, aluminum frame chassis, 12 servo actuator modules for the four legs, stereo camera system in the sensor housing head, LiDAR sensor puck, compute module, 48V battery pack, single mechanical gripper arm assembly, horizontal black cylindrical bag roll dispenser, rectangular dark metal folding bag frame with hinge mechanism, bright green LED strip modules for the upper legs, and rubber foot pads. Clean technical illustration style with thin leader lines. Professional, minimalist, 8K.
```

### 12. Dashboard Mockup with Robot
**File:** `tech-dashboard-mockup.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Split composition: Left side shows the robot in a park with its folding frame holding an open trash bag, its mechanical gripper arm actively picking up a piece of litter. Right side shows a fleet management dashboard on a large monitor displaying a map with robot positions, bags collected statistics, and battery levels. Modern office environment. Professional product marketing photo, 8K.
```

---

## Component Highlight Renders

### 13. Actuator Close-up (CubeMars Style)
**File:** `component-actuator.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance. A tall multi-joint robotic arm with mechanical gripper rises from the front top. At the rear, a rectangular dark metal tube frame is hinged open with a heavy-duty black trash bag hanging freely. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Close-up detail shot focusing on one of the robot's compact brushless DC servo actuator modules, removed from a leg joint and placed on a reflective dark surface. Cylindrical black anodized aluminum housing, about 70mm diameter, with CAN bus connector cable. The full robot is visible in soft focus in the background, showing its olive-green body and the folding frame with trash bag behind. Subtle rim lighting, clean industrial premium feel, studio product photography, 8K.
```

### 14. Custom PCB
**File:** `component-pcb.png`
```
A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance. A tall multi-joint robotic arm with mechanical gripper rises from the front top. At the rear, a rectangular dark metal tube frame is hinged open with a heavy-duty black trash bag hanging freely. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks. Close-up detail shot focusing on the robot's custom green PCB (printed circuit board) with SMD components, CAN bus connectors, and power distribution rails. Clean solder joints, professional assembly. On a dark matte surface with dramatic side lighting. The full robot is visible in soft focus in the background showing its dark olive-green body and the folding frame with trash bag. 8K macro photography.
```

---

## Brand Guidelines for Renders

- **Body color:** Dark matte olive-green (#3B4A3F) with smooth molded panels and softly rounded edges
- **Accent color:** CleanWalker Green (#22c55e) — LED strips on upper legs and LED eye panels
- **Style:** Smooth matte finishes, no chrome/glossy surfaces, injection-molded ABS look
- **Mood:** Professional, trustworthy, capable, modern
- **Key silhouette feature:** Rectangular folding frame hinged at 135 degrees from rear with heavy-duty black trash bag hanging freely
- **Bag roll:** Black cylindrical dispenser mounted horizontally (left-to-right axis) on center of back
- **Head:** Compact sensor housing module with flat face and two square green LED eyes — NOT an organic dog head
- **Arm:** Single articulated mechanical gripper (NOT soft/silicone), tallest point of robot
- **LEDs:** Bright green strips on outward-facing sides of UPPER LEG segments ONLY — NOT on body sides, NOT ring accents on joints
- **NEVER reference competitor brands** (no Unitree, Go2, Boston Dynamics, Spot) in any prompt — AI models embed their branding
- **Avoid:** Sci-fi/futuristic vibes, aggressive poses, pure black body, multiple arms, soft grippers, X-shaped frames, baskets/cages around bag, rough/tactical textures, glossy surfaces
