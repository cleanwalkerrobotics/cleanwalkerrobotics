#!/usr/bin/env node
/**
 * Generate all 14 CleanWalker robot renders via Replicate API (seedream-4.5)
 * Usage: node scripts/generate-renders.mjs [--only hero-park,hero-sidewalk]
 */

import { writeFile } from 'fs/promises';
import { join } from 'path';

const REPLICATE_API_TOKEN = process.env.REPLICATE_API_TOKEN;
if (!REPLICATE_API_TOKEN) {
  console.error('ERROR: REPLICATE_API_TOKEN not set');
  process.exit(1);
}

const OUTPUT_DIR = join(import.meta.dirname, '..', 'apps', 'web', 'public', 'renders');

const CORE = `A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. At the rear edge of the body, a rectangular dark metal tube frame is hinged open at 135 degrees from the back surface, extending backward and upward — it is the same width as the body and about half the body length deep. A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.`;

const CORE_CHARGING = `A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels — not an organic dog head but a sleek sensor module with gentle contouring. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance, with rubber foot pads. A tall multi-joint robotic arm with mechanical gripper rises from the front top of the body, extending about 40cm above the body with visible shoulder, elbow, and wrist joints. On the center of the flat back, a black cylindrical bag roll dispenser mounted horizontally with its axis running left-to-right across the body width. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.`;

const CORE_EXPLODED = `A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.`;

const CORE_COMPONENT = `A custom-designed quadrupedal robot with a smooth matte dark olive-green body with softly rounded panel edges. The body is a flat horizontal rectangular enclosure, roughly 45cm long, 30cm wide, and 15cm tall. The head is a compact sensor housing module at the front with a flat face featuring two square bright green LED eye panels. Bright green LED light strips on the outward-facing sides of the upper leg segments only. Four articulated legs roughly the same length as the body, mammalian stance. A tall multi-joint robotic arm with mechanical gripper rises from the front top. At the rear, a rectangular dark metal tube frame is hinged open with a heavy-duty black trash bag hanging freely. Smooth clean modern surfaces throughout. No text, no logos, no branding, no watermarks.`;

const RENDERS = [
  {
    name: 'hero-park',
    prompt: `${CORE} The robot is in a clean urban park, its mechanical gripper arm reaching down and picking up a plastic bottle from the grass. The heavy-duty black trash bag hanging from the rear folding frame is partially filled with collected litter. Morning golden hour lighting, shallow depth of field, green grass and trees in background. Photorealistic product photography, 8K, ultra detailed.`
  },
  {
    name: 'hero-sidewalk',
    prompt: `${CORE} The robot is walking on a European city sidewalk with cobblestone street. Four legs in dynamic walking pose. Modern architecture in the background. Overcast sky, soft diffused lighting. Professional product photography, studio quality, 8K.`
  },
  {
    name: 'hero-fleet',
    prompt: `${CORE} Wide-angle photo showing three of these identical robots working together in a large urban park, picking up litter. One robot is picking up a crushed can with its mechanical gripper arm, another is walking with its bag nearly full of litter, and the third has its folding frame folded down flat against the body, a sealed full bag dropped neatly at the curb behind it. Drone perspective, morning light, professional marketing photo. Ultra detailed, 8K.`
  },
  {
    name: 'detail-gripper',
    prompt: `${CORE} Extreme close-up focused on the robot's mechanical gripper with 2-3 metal fingers firmly grasping a crushed aluminum can. The gripper is attached to the single articulated robotic arm. In the background, slightly out of focus, the rear folding frame holds the open trash bag with collected litter visible inside. Shallow depth of field, studio lighting, product photography style, 8K macro photography.`
  },
  {
    name: 'detail-sensors',
    prompt: `${CORE} Front view close-up of the robot's sensor housing head module showing two square bright green LED eye panels on the flat front face. The matte dark olive-green body panels have clean industrial design with softly rounded chamfered edges. Behind and above, the rear folding frame is partially visible holding the open trash bag. Studio product photography with soft rim lighting, 8K.`
  },
  {
    name: 'detail-side-profile',
    prompt: `${CORE} Clean side-profile product photo on a white/light gray studio background. Standing pose, four articulated legs visible with all joints and actuator housings, the single robotic arm with mechanical gripper visible at front top, and the rectangular folding frame hinged open at 135 degrees from the rear of the body with the heavy-duty black trash bag hanging freely — the robot's most distinctive feature. On the back, the horizontal bag roll dispenser is visible between the arm mount and the frame hinge. Professional product photography, even lighting, no harsh shadows, 8K.`
  },
  {
    name: 'detail-charging-dock',
    prompt: `${CORE_CHARGING} The robot is parked on a small charging dock platform in an urban park setting. The dock is a simple weatherproof platform with a small rain canopy. Bright green LED strips glowing on the upper legs as a charging indicator. The rectangular folding frame at the rear is folded down flat against the body in compact transport mode, giving the robot a low sleek profile. Evening golden hour lighting, 8K.`
  },
  {
    name: 'lifestyle-city-worker',
    prompt: `${CORE} A city parks maintenance worker in a high-visibility vest stands next to the robot, checking a tablet showing fleet status. The robot is about knee-height, with its folding frame open and the heavy-duty black trash bag half-full of collected litter hanging behind it. They are in a well-maintained urban park. The worker looks pleased. Natural daylight, editorial photography style, 8K.`
  },
  {
    name: 'lifestyle-night-ops',
    prompt: `${CORE} The robot is operating on a city street at night. The bright green LED strips on the upper legs illuminate the sidewalk around it. Two square green LED eye panels on the sensor housing head glow visibly. Street lamps in the background. The robot is mid-stride with its folding frame holding the open trash bag behind the body. The mechanical gripper arm is pressing a piece of litter down into the bag. Cinematic lighting, shallow depth of field, moody atmosphere, 8K.`
  },
  {
    name: 'lifestyle-before-after',
    prompt: `${CORE.replace('A heavy-duty black trash bag hangs freely from this frame, draping down behind the robot — no basket, no cage, no mesh. ', '')} Split image composition: Left side shows a park area with scattered litter (plastic bottles, cans, food wrappers) on the ground. Right side shows the same park area pristine and clean, with the robot visible in the distance, its folding frame folded down flat against the body, a sealed full bag dropped neatly at the curb behind it. Before/after comparison, bright daylight, professional editorial photography, 8K.`
  },
  {
    name: 'tech-exploded-view',
    prompt: `${CORE_EXPLODED} Technical exploded view diagram on a white background. The robot's components are floating in space showing: dark olive-green weatherproof enclosure panels with rounded edges, aluminum frame chassis, 12 servo actuator modules for the four legs, stereo camera system in the sensor housing head, LiDAR sensor puck, compute module, 48V battery pack, single mechanical gripper arm assembly, horizontal black cylindrical bag roll dispenser, rectangular dark metal folding bag frame with hinge mechanism, bright green LED strip modules for the upper legs, and rubber foot pads. Clean technical illustration style with thin leader lines. Professional, minimalist, 8K.`
  },
  {
    name: 'tech-dashboard-mockup',
    prompt: `${CORE} Split composition: Left side shows the robot in a park with its folding frame holding an open trash bag, its mechanical gripper arm actively picking up a piece of litter. Right side shows a fleet management dashboard on a large monitor displaying a map with robot positions, bags collected statistics, and battery levels. Modern office environment. Professional product marketing photo, 8K.`
  },
  {
    name: 'component-actuator',
    prompt: `${CORE_COMPONENT} Close-up detail shot focusing on one of the robot's compact brushless DC servo actuator modules, removed from a leg joint and placed on a reflective dark surface. Cylindrical black anodized aluminum housing, about 70mm diameter, with CAN bus connector cable. The full robot is visible in soft focus in the background, showing its olive-green body and the folding frame with trash bag behind. Subtle rim lighting, clean industrial premium feel, studio product photography, 8K.`
  },
  {
    name: 'component-pcb',
    prompt: `${CORE_COMPONENT} Close-up detail shot focusing on the robot's custom green PCB (printed circuit board) with SMD components, CAN bus connectors, and power distribution rails. Clean solder joints, professional assembly. On a dark matte surface with dramatic side lighting. The full robot is visible in soft focus in the background showing its dark olive-green body and the folding frame with trash bag. 8K macro photography.`
  }
];

async function createPrediction(prompt) {
  const res = await fetch('https://api.replicate.com/v1/models/bytedance/seedream-4.5/predictions', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${REPLICATE_API_TOKEN}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      input: {
        prompt,
        aspect_ratio: '16:9',
        num_outputs: 1
      }
    })
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(`Create prediction failed (${res.status}): ${text}`);
  }
  return res.json();
}

async function pollPrediction(id) {
  const url = `https://api.replicate.com/v1/predictions/${id}`;
  while (true) {
    const res = await fetch(url, {
      headers: { 'Authorization': `Bearer ${REPLICATE_API_TOKEN}` }
    });
    const data = await res.json();
    if (data.status === 'succeeded') return data;
    if (data.status === 'failed' || data.status === 'canceled') {
      throw new Error(`Prediction ${id} ${data.status}: ${data.error || 'unknown error'}`);
    }
    // Wait 3 seconds before polling again
    await new Promise(r => setTimeout(r, 3000));
  }
}

async function downloadImage(url, filePath) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Download failed: ${res.status}`);
  const buffer = Buffer.from(await res.arrayBuffer());
  await writeFile(filePath, buffer);
  return buffer.length;
}

async function generateOne(render) {
  const outPath = join(OUTPUT_DIR, `${render.name}.png`);
  console.log(`[${render.name}] Creating prediction...`);
  const prediction = await createPrediction(render.prompt);
  console.log(`[${render.name}] Prediction ${prediction.id} created, polling...`);
  const result = await pollPrediction(prediction.id);
  const imageUrl = Array.isArray(result.output) ? result.output[0] : result.output;
  console.log(`[${render.name}] Downloading image...`);
  const size = await downloadImage(imageUrl, outPath);
  console.log(`[${render.name}] Saved ${outPath} (${(size / 1024).toFixed(0)} KB)`);
  return { name: render.name, path: outPath, size };
}

// Parse --only flag
const onlyArg = process.argv.find(a => a.startsWith('--only='));
let filter = null;
if (onlyArg) {
  filter = onlyArg.split('=')[1].split(',');
  console.log(`Generating only: ${filter.join(', ')}`);
}

const toGenerate = filter ? RENDERS.filter(r => filter.includes(r.name)) : RENDERS;

// Generate in batches of 4 to avoid rate limiting
const BATCH_SIZE = 4;
const results = [];

console.log(`\nGenerating ${toGenerate.length} renders via Replicate API (seedream-4.5)...\n`);

for (let i = 0; i < toGenerate.length; i += BATCH_SIZE) {
  const batch = toGenerate.slice(i, i + BATCH_SIZE);
  console.log(`--- Batch ${Math.floor(i / BATCH_SIZE) + 1} (${batch.map(r => r.name).join(', ')}) ---`);
  const batchResults = await Promise.all(batch.map(r => generateOne(r)));
  results.push(...batchResults);
}

console.log(`\n=== DONE: ${results.length} renders generated ===`);
results.forEach(r => console.log(`  ${r.name}: ${(r.size / 1024).toFixed(0)} KB`));
