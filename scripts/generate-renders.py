#!/usr/bin/env python3
"""
CleanWalker Robotics — AI Render Generator
Uses the Replicate API to generate photorealistic robot renders.
Model: Seedream 4.5 (bytedance/seedream-4.5) at 2560x1440 (16:9)
"""

import os
import sys
import time
import json
import requests
from pathlib import Path

API_TOKEN = os.environ.get("REPLICATE_API_TOKEN")
if not API_TOKEN:
    print("ERROR: REPLICATE_API_TOKEN not set")
    sys.exit(1)

HEADERS = {
    "Authorization": f"Bearer {API_TOKEN}",
    "Content-Type": "application/json",
}

BASE_DIR = Path(__file__).resolve().parent.parent / "apps" / "web" / "public" / "renders"
TEST_DIR = BASE_DIR / "test"

MODEL_ID = "bytedance/seedream-4.5"

# All 14 render prompts — updated for Bag Cassette System design
RENDERS = [
    {
        "name": "hero-park",
        "prompt": (
            "Photorealistic product photo of an autonomous quadrupedal litter-collecting "
            "robot in a clean urban park. The robot is matte charcoal gray with green LED "
            "accent lines, has four articulated legs with rubber foot pads, a front-mounted "
            "stereo camera system, and a small robotic arm with soft silicone gripper picking "
            "up a plastic bottle. On top of the robot, an X-shaped metal frame is raised "
            "upward holding a standard trash bag open with a square rim — the bag is partially "
            "filled with collected litter. The raised bag frame gives the robot a distinctive "
            "tall silhouette. Morning golden hour lighting, shallow depth of field, green "
            "grass and trees in background. Professional product photography style. 8K, "
            "ultra detailed."
        ),
    },
    {
        "name": "hero-sidewalk",
        "prompt": (
            "Photorealistic render of a sleek autonomous quadrupedal robot walking on a "
            "clean European city sidewalk. Matte dark gray body with green accent lighting, "
            "four articulated legs in walking pose, stereo camera eyes on the front, small "
            "LiDAR sensor on top. On the robot's back, a raised X-shaped metal frame holds "
            "a trash bag open with a square rim, giving the robot a distinctive tall "
            "silhouette. The robot is about the size of a medium dog. Modern architecture "
            "and cobblestone street in the background. Overcast sky, soft diffused lighting. "
            "Professional product photography, studio quality. 8K."
        ),
    },
    {
        "name": "hero-fleet",
        "prompt": (
            "Photorealistic wide-angle photo of three autonomous quadrupedal robots working "
            "in a large urban park, picking up litter. The robots are matte charcoal with "
            "green LED strips, each about dog-sized with four legs. Each robot has a raised "
            "X-shaped metal frame on its back holding open a trash bag with a square rim. "
            "One robot is picking up a can with its small robotic arm, another is walking "
            "with its bag nearly full, and the third has its frame lowered flat — dropping "
            "a sealed full bag at the curb before reloading a fresh one. Drone perspective, "
            "morning light, professional marketing photo. Ultra detailed, 8K."
        ),
    },
    {
        "name": "detail-gripper",
        "prompt": (
            "Extreme close-up photorealistic render of a robotic soft gripper made of "
            "translucent silicone, with three fingers gently grasping a crushed aluminum "
            "can. The gripper is attached to a small 2-DOF robotic arm on a matte charcoal "
            "quadrupedal robot. In the background, slightly out of focus, the robot's raised "
            "X-shaped bag frame holds open a trash bag where collected litter is visible. "
            "Shallow depth of field, studio lighting, product photography style. Shows the "
            "precision and gentleness of the grip. 8K macro photography."
        ),
    },
    {
        "name": "detail-sensors",
        "prompt": (
            "Front view close-up of an autonomous robot's sensor head. Two circular camera "
            "lenses (stereo depth camera) with a small IR dot projector between them, "
            "mounted on a matte charcoal body panel. A small green LED status indicator "
            "glows. Behind and above, the raised X-shaped bag cassette frame is partially "
            "visible, holding an open trash bag. Clean industrial design, slightly rounded "
            "edges. Studio product photography with soft rim lighting. 8K."
        ),
    },
    {
        "name": "detail-side-profile",
        "prompt": (
            "Clean side-profile product photo of an autonomous quadrupedal litter robot on "
            "a white/light gray studio background. Matte charcoal body with green accent "
            "line, four articulated legs in standing pose, visible joints with aluminum "
            "hardware, front stereo camera, top-mounted LiDAR puck, small robotic arm "
            "folded underneath. On top, the X-shaped bag cassette frame is raised, holding "
            "a standard trash bag open with a square metal rim — the robot's most distinctive "
            "feature. The internal bag roll cassette is faintly visible through a panel. "
            "Professional product photography, even lighting, no shadows. 8K."
        ),
    },
    {
        "name": "detail-charging-dock",
        "prompt": (
            "Photorealistic render of an autonomous quadrupedal robot parked on a small "
            "charging dock in an urban park setting. The dock is a simple weatherproof "
            "platform with a small rain canopy. The robot has green LED strip glowing "
            "(charging indicator). The X-shaped bag cassette frame is lowered flat against "
            "the body in compact transport mode, giving the robot a low, sleek profile. "
            "Pogo pin contacts visible at the base. Clean, modern design. Evening golden "
            "hour lighting. 8K."
        ),
    },
    {
        "name": "lifestyle-city-worker",
        "prompt": (
            "Photorealistic photo of a city parks maintenance worker in a high-vis vest "
            "standing next to an autonomous quadrupedal litter robot, checking a "
            "tablet/dashboard. The robot is matte charcoal with green accents, about "
            "knee-height, with its X-shaped bag frame raised and holding an open trash bag "
            "half-full of collected litter. They are in a well-maintained urban park. The "
            "worker looks pleased. Natural daylight, editorial photography style. 8K."
        ),
    },
    {
        "name": "lifestyle-night-ops",
        "prompt": (
            "Moody photorealistic render of an autonomous quadrupedal robot operating on a "
            "city street at night. The robot's green LED status strip illuminates the "
            "sidewalk. Its camera sensors glow faintly. Street lamps in the background. The "
            "robot is mid-stride with its raised X-shaped bag frame holding an open trash "
            "bag — the robotic arm is pressing a piece of litter down into the bag, "
            "compacting it. Cinematic lighting, shallow depth of field. 8K."
        ),
    },
    {
        "name": "lifestyle-before-after",
        "prompt": (
            "Split image: Left side shows a park with scattered litter (bottles, cans, "
            "wrappers). Right side shows the same park pristine and clean, with a small "
            "quadrupedal robot in the distance, its X-shaped bag frame lowered flat, a "
            "sealed full bag dropped neatly at the curb behind it. Before/after comparison, "
            "bright daylight, professional editorial photography. 8K."
        ),
    },
    {
        "name": "tech-exploded-view",
        "prompt": (
            "Technical exploded view diagram of an autonomous quadrupedal robot on a white "
            "background. Components floating in space showing: aluminum frame chassis, 12 "
            "servo actuator modules, stereo camera system, LiDAR sensor, Jetson compute "
            "module, 48V battery pack, soft gripper arm assembly, X-shaped bag cassette "
            "frame with square rim, replaceable bag roll cartridge, snap-close bag seal "
            "mechanism, 4G antenna, and weatherproof enclosure panels. Clean technical "
            "illustration style with thin leader lines and labels. Professional, minimalist. 8K."
        ),
    },
    {
        "name": "tech-dashboard-mockup",
        "prompt": (
            "Split composition: left side shows a real quadrupedal litter robot in a park "
            "with its raised X-shaped bag frame holding an open trash bag, right side shows "
            "a fleet management dashboard on a large monitor displaying a map with robot "
            "positions, bags collected statistics, and battery levels. Modern office "
            "environment. Professional product marketing photo. 8K."
        ),
    },
    {
        "name": "component-actuator",
        "prompt": (
            "Product photography of a compact brushless DC servo actuator module. "
            "Cylindrical black anodized aluminum housing, about 70mm diameter, with CAN bus "
            "connector cable. The actuator sits on a reflective dark surface with subtle rim "
            "lighting. Clean, industrial, premium feel. Studio product photography. 8K."
        ),
    },
    {
        "name": "component-pcb",
        "prompt": (
            'Close-up product photography of a custom green PCB (printed circuit board) '
            'with SMD components, CAN bus connectors, and power distribution rails. Clean '
            'solder joints, professional assembly. The board has "CleanWalker" silkscreen '
            'text. On a dark matte surface with dramatic side lighting. 8K macro photography.'
        ),
    },
]


def create_prediction(prompt: str, aspect_ratio: str = "16:9") -> dict:
    """Submit a prediction to the Replicate API."""
    url = f"https://api.replicate.com/v1/models/{MODEL_ID}/predictions"
    payload = {
        "input": {
            "prompt": prompt,
            "aspect_ratio": aspect_ratio,
        }
    }
    resp = requests.post(url, headers=HEADERS, json=payload, timeout=30)
    if resp.status_code not in (200, 201):
        print(f"  ERROR creating prediction: {resp.status_code} {resp.text}")
        return None
    return resp.json()


def poll_prediction(prediction_url: str, max_wait: int = 300) -> dict:
    """Poll a prediction until it succeeds, fails, or times out."""
    start = time.time()
    while time.time() - start < max_wait:
        resp = requests.get(prediction_url, headers=HEADERS, timeout=30)
        data = resp.json()
        status = data.get("status")
        if status == "succeeded":
            return data
        if status in ("failed", "canceled"):
            print(f"  Prediction {status}: {data.get('error', 'unknown error')}")
            return None
        time.sleep(3)
    print("  Prediction timed out")
    return None


def download_image(image_url: str, output_path: Path) -> bool:
    """Download an image from a URL."""
    try:
        resp = requests.get(image_url, timeout=120)
        if resp.status_code == 200:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(resp.content)
            size_kb = len(resp.content) / 1024
            print(f"  Saved: {output_path.name} ({size_kb:.0f} KB)")
            return True
        else:
            print(f"  Download failed: {resp.status_code}")
            return False
    except Exception as e:
        print(f"  Download error: {e}")
        return False


def get_output_url(result: dict) -> str:
    """Extract the image URL from a prediction result."""
    output = result.get("output")
    if isinstance(output, list):
        return output[0] if output else None
    if isinstance(output, str):
        return output
    return None


def generate_single(prompt: str, output_path: Path) -> bool:
    """Generate a single image: create prediction, poll, download."""
    print(f"  Submitting to {MODEL_ID}...")
    prediction = create_prediction(prompt)
    if not prediction:
        return False

    pred_url = prediction.get("urls", {}).get("get")
    if not pred_url:
        print("  ERROR: No polling URL returned")
        return False

    print(f"  Polling for result (id: {prediction['id']})...")
    result = poll_prediction(pred_url)
    if not result:
        return False

    image_url = get_output_url(result)
    if not image_url:
        print("  ERROR: No output URL in result")
        return False

    return download_image(image_url, output_path)


def generate_all():
    """Generate all 14 renders using Seedream 4.5."""
    print("=" * 60)
    print(f"Generating all 14 renders with {MODEL_ID}")
    print("=" * 60)

    success_count = 0
    fail_count = 0

    for i, render in enumerate(RENDERS, 1):
        output_path = BASE_DIR / f"{render['name']}.png"
        print(f"\n[{i}/14] {render['name']}")
        ok = generate_single(render["prompt"], output_path)
        if ok:
            success_count += 1
        else:
            fail_count += 1

    print("\n" + "=" * 60)
    print(f"DONE: {success_count} succeeded, {fail_count} failed")
    print("=" * 60)
    return success_count, fail_count


if __name__ == "__main__":
    generate_all()
