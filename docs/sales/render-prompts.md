# CleanWalker Robot — AI Render Prompts

**Purpose:** Generate photorealistic marketing renders for website, pitch deck, and sales materials.
**Tool:** Seedream 4.5 (bytedance/seedream-4.5) via Replicate API at 16:9 aspect ratio.
**Style:** Clean, professional, photorealistic product photography. Think Boston Dynamics Spot marketing meets Apple product shots.
**Design Spec:** See `docs/design/robot-design-spec.md` for canonical robot design.

---

## Core Robot Description (MUST start every prompt)

Every render prompt begins with this exact canonical description, then appends scene-specific details:

```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic.
```

---

## Hero Shots (Website Homepage)

### 1. Hero — Robot in Park Setting
**File:** `hero-park.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. The robot is in a clean urban park, its mechanical gripper arm reaching down and picking up a plastic bottle from the grass. The semi-transparent trash bag on its back is partially filled with collected litter visible through the material. Morning golden hour lighting, shallow depth of field, green grass and trees in background. Photorealistic product photography, 8K, ultra detailed.
```

### 2. Hero — Robot on City Sidewalk
**File:** `hero-sidewalk.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. The robot is walking on a European city sidewalk with cobblestone street. Four legs in dynamic walking pose. Modern architecture in the background. Overcast sky, soft diffused lighting. Professional product photography, studio quality, 8K.
```

### 3. Hero — Fleet of Robots Working
**File:** `hero-fleet.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Wide-angle photo showing three of these identical robots working together in a large urban park, picking up litter. One robot is picking up a crushed can with its mechanical gripper arm, another is walking with its bag nearly full of litter, and the third has its scissor frame lowered flat, dropping a sealed full bag at the curb. Drone perspective, morning light, professional marketing photo. Ultra detailed, 8K.
```

---

## Product Detail Shots (Product Page)

### 4. Close-up — Gripper Picking Up Litter
**File:** `detail-gripper.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Extreme close-up focused on the robot's mechanical gripper with 2-3 metal fingers firmly grasping a crushed aluminum can. The gripper is attached to the single articulated robotic arm. In the background, slightly out of focus, the raised X-shaped scissor bag frame holds the open trash bag with collected litter visible inside. Shallow depth of field, studio lighting, product photography style, 8K macro photography.
```

### 5. Close-up — Sensor Array (Front Face)
**File:** `detail-sensors.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Front view close-up of the robot's face showing two square bright green LED eyes (camera/sensor windows) on the front panel. The matte dark olive-green body panels have clean industrial design with slightly rounded chamfered edges. Behind and above, the raised X-shaped scissor bag frame is partially visible holding the open trash bag. Studio product photography with soft rim lighting, 8K.
```

### 6. Side Profile — Full Robot
**File:** `detail-side-profile.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Clean side-profile product photo on a white/light gray studio background. Standing pose, four articulated legs visible with all joints and servo motors, the single robotic arm with mechanical gripper visible at front top, and the X-shaped scissor bag frame raised on its back holding the open semi-transparent trash bag — the robot's most distinctive feature. Professional product photography, even lighting, no harsh shadows, 8K.
```

### 7. Robot at Charging Dock
**File:** `detail-charging-dock.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. The robot is parked on a small charging dock platform in an urban park setting. The dock is a simple weatherproof platform with a small rain canopy. Bright green LED strips glowing as a charging indicator. The X-shaped scissor bag frame is lowered flat against the body in compact transport mode, giving the robot a low sleek profile. Evening golden hour lighting, 8K.
```

---

## Lifestyle / Context Shots (About Page, Social Media)

### 8. Robot and City Worker
**File:** `lifestyle-city-worker.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. A city parks maintenance worker in a high-visibility vest stands next to the robot, checking a tablet showing fleet status. The robot is about knee-height, with its X-shaped scissor bag frame raised and holding an open semi-transparent trash bag half-full of collected litter. They are in a well-maintained urban park. The worker looks pleased. Natural daylight, editorial photography style, 8K.
```

### 9. Night Operations
**File:** `lifestyle-night-ops.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. The robot is operating on a city street at night. The bright green LED strips illuminate the sidewalk around it. Two square green LED eyes on the front glow visibly. Street lamps in the background. The robot is mid-stride with its raised X-shaped scissor bag frame holding the open trash bag. The mechanical gripper arm is pressing a piece of litter down into the bag. Cinematic lighting, shallow depth of field, moody atmosphere, 8K.
```

### 10. Before/After — Park Clean-up
**File:** `lifestyle-before-after.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Split image composition: Left side shows a park area with scattered litter (plastic bottles, cans, food wrappers) on the ground. Right side shows the same park area pristine and clean, with the robot visible in the distance, its X-shaped scissor bag frame lowered flat, a sealed full bag dropped neatly at the curb behind it. Before/after comparison, bright daylight, professional editorial photography, 8K.
```

---

## Technical / Pitch Deck Shots

### 11. Exploded View
**File:** `tech-exploded-view.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Technical exploded view diagram on a white background. The robot's components are floating in space showing: dark olive-green weatherproof enclosure panels, aluminum frame chassis, 12 servo actuator modules for the four legs, stereo camera system, LiDAR sensor puck, compute module, 48V battery pack, single mechanical gripper arm assembly, X-shaped scissor bag frame with square rim, and bright green LED strip modules. Clean technical illustration style with thin leader lines and component labels. Professional, minimalist, 8K.
```

### 12. Dashboard Mockup with Robot
**File:** `tech-dashboard-mockup.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Split composition: Left side shows the robot in a park with its raised X-shaped scissor bag frame holding an open trash bag, its mechanical gripper arm actively picking up a piece of litter. Right side shows a fleet management dashboard on a large monitor displaying a map with robot positions, bags collected statistics, and battery levels. Modern office environment. Professional product marketing photo, 8K.
```

---

## Component Highlight Renders

### 13. Actuator Close-up (CubeMars Style)
**File:** `component-actuator.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Close-up detail shot focusing on one of the robot's compact brushless DC servo actuator modules, removed from a leg joint and placed on a reflective dark surface. Cylindrical black anodized aluminum housing, about 70mm diameter, with CAN bus connector cable. The full robot is visible in soft focus in the background, showing its olive-green body and X-shaped scissor bag frame. Subtle rim lighting, clean industrial premium feel, studio product photography, 8K.
```

### 14. Custom PCB
**File:** `component-pcb.png`
```
A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. Close-up detail shot focusing on the robot's custom green PCB (printed circuit board) with SMD components, CAN bus connectors, and power distribution rails. Clean solder joints, professional assembly. On a dark matte surface with dramatic side lighting. The full robot is visible in soft focus in the background showing its dark olive-green body and raised X-shaped scissor bag frame. 8K macro photography.
```

---

## Brand Guidelines for Renders

- **Body color:** Dark matte olive-green (#3B4A3F)
- **Accent color:** CleanWalker Green (#22c55e) — LED strips and eye indicators
- **Style:** Matte finishes, no chrome/glossy surfaces
- **Mood:** Professional, trustworthy, capable, modern
- **Key silhouette feature:** Raised X-shaped scissor bag frame with open semi-transparent trash bag
- **Arm:** Single articulated mechanical gripper (NOT soft/silicone)
- **Avoid:** Sci-fi/futuristic vibes, aggressive poses, pure black body, multiple arms, soft grippers
- **Aspire to:** Boston Dynamics Spot aesthetics + Dyson product design + municipal utility vehicle pragmatism
