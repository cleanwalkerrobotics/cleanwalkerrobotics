#!/usr/bin/env node
/**
 * Regenerate contaminated renders via Replicate API (Seedream 4.5).
 * Submits all 11 renders in parallel, polls for completion, downloads results.
 */
import { writeFile, mkdir } from 'fs/promises';
import { resolve } from 'path';

const TOKEN = process.env.REPLICATE_API_TOKEN;
if (!TOKEN) { console.error('REPLICATE_API_TOKEN not set'); process.exit(1); }

const OUTPUT_DIR = resolve(import.meta.dirname, '../apps/web/public/renders');
const API = 'https://api.replicate.com/v1/models/bytedance/seedream-4.5/predictions';

const CORE = `A custom-designed quadrupedal robot with a matte dark olive-green body, bright green LED strips on each side, four articulated legs with rubber foot pads, a single articulated robotic arm with mechanical gripper mounted on the front top, and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. Industrial-modern aesthetic. No text, no logos, no branding, no watermarks anywhere on the robot body. Clean unmarked surfaces.`;

const RENDERS = [
  {
    file: 'hero-park.png',
    scene: `The robot is in a clean urban park, its mechanical gripper arm reaching down and picking up a plastic bottle from the grass. The semi-transparent trash bag on its back is partially filled with collected litter visible through the material. Morning golden hour lighting, shallow depth of field, green grass and trees in background. Photorealistic product photography, 8K, ultra detailed.`
  },
  {
    file: 'hero-sidewalk.png',
    scene: `The robot is walking on a European city sidewalk with cobblestone street. Four legs in dynamic walking pose. Modern architecture in the background. Overcast sky, soft diffused lighting. Professional product photography, studio quality, 8K.`
  },
  {
    file: 'hero-fleet.png',
    scene: `Wide-angle photo showing three of these identical robots working together in a large urban park, picking up litter. One robot is picking up a crushed can with its mechanical gripper arm, another is walking with its bag nearly full of litter, and the third has its scissor frame lowered flat, dropping a sealed full bag at the curb. Drone perspective, morning light, professional marketing photo. Ultra detailed, 8K.`
  },
  {
    file: 'detail-gripper.png',
    scene: `Extreme close-up focused on the robot's mechanical gripper with 2-3 metal fingers firmly grasping a crushed aluminum can. The gripper is attached to the single articulated robotic arm. In the background, slightly out of focus, the raised X-shaped scissor bag frame holds the open trash bag with collected litter visible inside. Shallow depth of field, studio lighting, product photography style, 8K macro photography.`
  },
  {
    file: 'detail-side-profile.png',
    scene: `Clean side-profile product photo on a white/light gray studio background. Standing pose, four articulated legs visible with all joints and servo motors, the single robotic arm with mechanical gripper visible at front top, and the X-shaped scissor bag frame raised on its back holding the open semi-transparent trash bag — the robot's most distinctive feature. Professional product photography, even lighting, no harsh shadows, 8K.`
  },
  {
    file: 'detail-charging-dock.png',
    scene: `The robot is parked on a small charging dock platform in an urban park setting. The dock is a simple weatherproof platform with a small rain canopy. Bright green LED strips glowing as a charging indicator. The X-shaped scissor bag frame is lowered flat against the body in compact transport mode, giving the robot a low sleek profile. Evening golden hour lighting, 8K.`
  },
  {
    file: 'lifestyle-city-worker.png',
    scene: `A city parks maintenance worker in a high-visibility vest stands next to the robot, checking a tablet showing fleet status. The robot is about knee-height, with its X-shaped scissor bag frame raised and holding an open semi-transparent trash bag half-full of collected litter. They are in a well-maintained urban park. The worker looks pleased. Natural daylight, editorial photography style, 8K.`
  },
  {
    file: 'lifestyle-night-ops.png',
    scene: `The robot is operating on a city street at night. The bright green LED strips illuminate the sidewalk around it. Two square green LED eyes on the front glow visibly. Street lamps in the background. The robot is mid-stride with its raised X-shaped scissor bag frame holding the open trash bag. The mechanical gripper arm is pressing a piece of litter down into the bag. Cinematic lighting, shallow depth of field, moody atmosphere, 8K.`
  },
  {
    file: 'component-actuator.png',
    scene: `Close-up detail shot focusing on one of the robot's compact brushless DC servo actuator modules, removed from a leg joint and placed on a reflective dark surface. Cylindrical black anodized aluminum housing, about 70mm diameter, with CAN bus connector cable. The full robot is visible in soft focus in the background, showing its olive-green body and X-shaped scissor bag frame. Subtle rim lighting, clean industrial premium feel, studio product photography, 8K.`
  },
  {
    file: 'component-pcb.png',
    scene: `Close-up detail shot focusing on the robot's custom green PCB (printed circuit board) with SMD components, CAN bus connectors, and power distribution rails. Clean solder joints, professional assembly. On a dark matte surface with dramatic side lighting. The full robot is visible in soft focus in the background showing its dark olive-green body and raised X-shaped scissor bag frame. 8K macro photography.`
  },
  {
    file: 'tech-dashboard-mockup.png',
    scene: `Split composition: Left side shows the robot in a park with its raised X-shaped scissor bag frame holding an open trash bag, its mechanical gripper arm actively picking up a piece of litter. Right side shows a fleet management dashboard on a large monitor displaying a map with robot positions, bags collected statistics, and battery levels. Modern office environment. Professional product marketing photo, 8K.`
  }
];

async function createPrediction(prompt) {
  const res = await fetch(API, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${TOKEN}`,
      'Content-Type': 'application/json',
      'Prefer': 'wait'  // Sync mode — waits up to 60s
    },
    body: JSON.stringify({
      input: {
        prompt,
        aspect_ratio: '16:9'
      }
    })
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(`API error ${res.status}: ${text}`);
  }
  return res.json();
}

async function pollPrediction(url) {
  while (true) {
    const res = await fetch(url, {
      headers: { 'Authorization': `Bearer ${TOKEN}` }
    });
    const data = await res.json();
    if (data.status === 'succeeded') return data;
    if (data.status === 'failed' || data.status === 'canceled') {
      throw new Error(`Prediction ${data.status}: ${data.error || 'unknown'}`);
    }
    // Still processing, wait 3s
    await new Promise(r => setTimeout(r, 3000));
  }
}

async function downloadImage(url, filepath) {
  const res = await fetch(url);
  const buf = Buffer.from(await res.arrayBuffer());
  await writeFile(filepath, buf);
  return buf.length;
}

async function generateRender(render) {
  const prompt = `${CORE} ${render.scene}`;
  const outPath = resolve(OUTPUT_DIR, render.file);

  console.log(`[START] ${render.file}`);

  try {
    let prediction = await createPrediction(prompt);

    // If not completed yet (sync mode timeout), poll
    if (prediction.status !== 'succeeded') {
      console.log(`[POLL]  ${render.file} — status: ${prediction.status}`);
      prediction = await pollPrediction(prediction.urls.get);
    }

    // Download the output image
    const outputUrl = Array.isArray(prediction.output) ? prediction.output[0] : prediction.output;
    const bytes = await downloadImage(outputUrl, outPath);
    console.log(`[DONE]  ${render.file} — ${(bytes / 1024).toFixed(0)} KB`);
    return { file: render.file, status: 'ok', bytes };
  } catch (err) {
    console.error(`[FAIL]  ${render.file} — ${err.message}`);
    return { file: render.file, status: 'failed', error: err.message };
  }
}

// Run all 11 in parallel (Replicate handles queuing)
console.log(`\nGenerating ${RENDERS.length} renders via Seedream 4.5...\n`);
const results = await Promise.all(RENDERS.map(r => generateRender(r)));

console.log('\n=== RESULTS ===');
const ok = results.filter(r => r.status === 'ok');
const failed = results.filter(r => r.status === 'failed');
console.log(`Succeeded: ${ok.length}/${RENDERS.length}`);
if (failed.length > 0) {
  console.log(`Failed:`);
  failed.forEach(f => console.log(`  - ${f.file}: ${f.error}`));
}
console.log('');

process.exit(failed.length > 0 ? 1 : 0);
