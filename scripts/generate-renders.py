#!/usr/bin/env python3
"""
CleanWalker Robotics — AI Render Generator v2.0
Uses the Replicate API to generate photorealistic robot renders.
Model: Seedream 4.5 (bytedance/seedream-4.5) at 16:9
All prompts start with the CANONICAL robot description from docs/design/robot-design-spec.md
"""

import json
import os
import sys
import time
import urllib.request
import urllib.error

API_TOKEN = os.environ.get("REPLICATE_API_TOKEN")
if not API_TOKEN:
    print("ERROR: REPLICATE_API_TOKEN environment variable not set")
    sys.exit(1)
MODEL = "bytedance/seedream-4.5"
API_BASE = "https://api.replicate.com/v1"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "apps", "web", "public", "renders")

# Canonical robot description — MUST start every prompt
CORE = (
    "A quadrupedal robot dog (similar to Unitree Go2) with a matte dark olive-green body, "
    "bright green LED strips on each side, four articulated legs with rubber foot pads, "
    "a single articulated robotic arm with mechanical gripper mounted on the front top, "
    "and an X-shaped scissor frame on its back holding open a semi-transparent black trash bag "
    "with a square rim at the top. The robot is about medium-dog sized, ~60cm at the body. "
    "Industrial-modern aesthetic."
)

# All 14 render scenes — each gets CORE prepended
RENDERS = [
    {
        "filename": "hero-park.png",
        "scene": (
            " The robot is in a clean urban park, its mechanical gripper arm reaching down and "
            "picking up a plastic bottle from the grass. The semi-transparent trash bag on its "
            "back is partially filled with collected litter visible through the material. "
            "Morning golden hour lighting, shallow depth of field, green grass and trees in "
            "background. Photorealistic product photography, 8K, ultra detailed."
        ),
    },
    {
        "filename": "hero-sidewalk.png",
        "scene": (
            " The robot is walking on a European city sidewalk with cobblestone street. Four "
            "legs in dynamic walking pose. Modern architecture in the background. Overcast sky, "
            "soft diffused lighting. Professional product photography, studio quality, 8K."
        ),
    },
    {
        "filename": "hero-fleet.png",
        "scene": (
            " Wide-angle photo showing three of these identical robots working together in a "
            "large urban park, picking up litter. One robot is picking up a crushed can with "
            "its mechanical gripper arm, another is walking with its bag nearly full of litter, "
            "and the third has its scissor frame lowered flat, dropping a sealed full bag at "
            "the curb. Drone perspective, morning light, professional marketing photo. "
            "Ultra detailed, 8K."
        ),
    },
    {
        "filename": "detail-gripper.png",
        "scene": (
            " Extreme close-up focused on the robot's mechanical gripper with 2-3 metal "
            "fingers firmly grasping a crushed aluminum can. The gripper is attached to the "
            "single articulated robotic arm. In the background, slightly out of focus, the "
            "raised X-shaped scissor bag frame holds the open trash bag with collected litter "
            "visible inside. Shallow depth of field, studio lighting, product photography "
            "style, 8K macro photography."
        ),
    },
    {
        "filename": "detail-sensors.png",
        "scene": (
            " Front view close-up of the robot's face showing two square bright green LED "
            "eyes (camera/sensor windows) on the front panel. The matte dark olive-green body "
            "panels have clean industrial design with slightly rounded chamfered edges. Behind "
            "and above, the raised X-shaped scissor bag frame is partially visible holding "
            "the open trash bag. Studio product photography with soft rim lighting, 8K."
        ),
    },
    {
        "filename": "detail-side-profile.png",
        "scene": (
            " Clean side-profile product photo on a white/light gray studio background. "
            "Standing pose, four articulated legs visible with all joints and servo motors, "
            "the single robotic arm with mechanical gripper visible at front top, and the "
            "X-shaped scissor bag frame raised on its back holding the open semi-transparent "
            "trash bag — the robot's most distinctive feature. Professional product "
            "photography, even lighting, no harsh shadows, 8K."
        ),
    },
    {
        "filename": "detail-charging-dock.png",
        "scene": (
            " The robot is parked on a small charging dock platform in an urban park setting. "
            "The dock is a simple weatherproof platform with a small rain canopy. Bright green "
            "LED strips glowing as a charging indicator. The X-shaped scissor bag frame is "
            "lowered flat against the body in compact transport mode, giving the robot a low "
            "sleek profile. Evening golden hour lighting, 8K."
        ),
    },
    {
        "filename": "lifestyle-city-worker.png",
        "scene": (
            " A city parks maintenance worker in a high-visibility vest stands next to the "
            "robot, checking a tablet showing fleet status. The robot is about knee-height, "
            "with its X-shaped scissor bag frame raised and holding an open semi-transparent "
            "trash bag half-full of collected litter. They are in a well-maintained urban park. "
            "The worker looks pleased. Natural daylight, editorial photography style, 8K."
        ),
    },
    {
        "filename": "lifestyle-night-ops.png",
        "scene": (
            " The robot is operating on a city street at night. The bright green LED strips "
            "illuminate the sidewalk around it. Two square green LED eyes on the front glow "
            "visibly. Street lamps in the background. The robot is mid-stride with its raised "
            "X-shaped scissor bag frame holding the open trash bag. The mechanical gripper arm "
            "is pressing a piece of litter down into the bag. Cinematic lighting, shallow "
            "depth of field, moody atmosphere, 8K."
        ),
    },
    {
        "filename": "lifestyle-before-after.png",
        "scene": (
            " Split image composition: Left side shows a park area with scattered litter "
            "(plastic bottles, cans, food wrappers) on the ground. Right side shows the same "
            "park area pristine and clean, with the robot visible in the distance, its "
            "X-shaped scissor bag frame lowered flat, a sealed full bag dropped neatly at the "
            "curb behind it. Before/after comparison, bright daylight, professional editorial "
            "photography, 8K."
        ),
    },
    {
        "filename": "tech-exploded-view.png",
        "scene": (
            " Technical exploded view diagram on a white background. The robot's components "
            "are floating in space showing: dark olive-green weatherproof enclosure panels, "
            "aluminum frame chassis, 12 servo actuator modules for the four legs, stereo "
            "camera system, LiDAR sensor puck, compute module, 48V battery pack, single "
            "mechanical gripper arm assembly, X-shaped scissor bag frame with square rim, and "
            "bright green LED strip modules. Clean technical illustration style with thin "
            "leader lines and component labels. Professional, minimalist, 8K."
        ),
    },
    {
        "filename": "tech-dashboard-mockup.png",
        "scene": (
            " Split composition: Left side shows the robot in a park with its raised X-shaped "
            "scissor bag frame holding an open trash bag, its mechanical gripper arm actively "
            "picking up a piece of litter. Right side shows a fleet management dashboard on a "
            "large monitor displaying a map with robot positions, bags collected statistics, "
            "and battery levels. Modern office environment. Professional product marketing "
            "photo, 8K."
        ),
    },
    {
        "filename": "component-actuator.png",
        "scene": (
            " Close-up detail shot focusing on one of the robot's compact brushless DC servo "
            "actuator modules, removed from a leg joint and placed on a reflective dark "
            "surface. Cylindrical black anodized aluminum housing, about 70mm diameter, with "
            "CAN bus connector cable. The full robot is visible in soft focus in the "
            "background, showing its olive-green body and X-shaped scissor bag frame. Subtle "
            "rim lighting, clean industrial premium feel, studio product photography, 8K."
        ),
    },
    {
        "filename": "component-pcb.png",
        "scene": (
            " Close-up detail shot focusing on the robot's custom green PCB (printed circuit "
            "board) with SMD components, CAN bus connectors, and power distribution rails. "
            "Clean solder joints, professional assembly. On a dark matte surface with dramatic "
            "side lighting. The full robot is visible in soft focus in the background showing "
            "its dark olive-green body and raised X-shaped scissor bag frame. 8K macro "
            "photography."
        ),
    },
]


def api_request(method, url, data=None):
    """Make an authenticated API request to Replicate."""
    headers = {
        "Authorization": f"Bearer {API_TOKEN}",
        "Content-Type": "application/json",
    }
    body = json.dumps(data).encode() if data else None
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        error_body = e.read().decode() if e.fp else "no body"
        print(f"  HTTP {e.code}: {error_body[:200]}")
        return None
    except Exception as e:
        print(f"  Request error: {e}")
        return None


def create_prediction(prompt):
    """Create a prediction on Replicate."""
    url = f"{API_BASE}/models/{MODEL}/predictions"
    data = {
        "input": {
            "prompt": prompt,
            "aspect_ratio": "16:9",
        }
    }
    return api_request("POST", url, data)


def poll_prediction(prediction_id, max_wait=300):
    """Poll a prediction until it completes or fails."""
    url = f"{API_BASE}/predictions/{prediction_id}"
    start = time.time()
    while time.time() - start < max_wait:
        result = api_request("GET", url)
        if result is None:
            time.sleep(5)
            continue
        status = result.get("status")
        if status == "succeeded":
            return result
        elif status in ("failed", "canceled"):
            print(f"  Prediction {status}: {result.get('error', 'unknown')}")
            return None
        time.sleep(4)
    print(f"  Prediction timed out after {max_wait}s")
    return None


def download_file(url, filepath):
    """Download a file from URL to filepath. Returns file size in bytes."""
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=120) as resp:
        data = resp.read()
    with open(filepath, "wb") as f:
        f.write(data)
    return len(data)


def generate_render(render, attempt=1):
    """Generate a single render. Returns (success, filesize)."""
    filename = render["filename"]
    prompt = CORE + render["scene"]
    filepath = os.path.join(OUTPUT_DIR, filename)

    # For retries, add emphasis on key design elements
    if attempt > 1:
        prompt += (
            " IMPORTANT: The robot body color is dark olive-green (not black, not gray). "
            "Bright green LED strips must be visible on the sides. It has exactly one "
            "mechanical metal gripper arm (not silicone, not soft). The X-shaped scissor "
            "frame with semi-transparent black trash bag on the back must be clearly visible."
        )

    print(f"\n{'='*60}")
    print(f"[{filename}] Attempt {attempt}")
    sys.stdout.flush()

    # Create prediction
    result = create_prediction(prompt)
    if result is None:
        return False, 0

    pred_id = result.get("id")
    if not pred_id:
        print(f"  No prediction ID returned")
        return False, 0
    print(f"  Prediction: {pred_id}")
    sys.stdout.flush()

    # Poll for completion
    result = poll_prediction(pred_id)
    if result is None:
        return False, 0

    # Get output URL
    output = result.get("output")
    if isinstance(output, list):
        output_url = output[0] if output else None
    elif isinstance(output, str):
        output_url = output
    else:
        print(f"  Unexpected output format: {type(output)}")
        return False, 0

    if not output_url:
        print(f"  No output URL")
        return False, 0

    # Download
    try:
        filesize = download_file(output_url, filepath)
        size_kb = filesize / 1024
        print(f"  Downloaded: {size_kb:.0f} KB -> {filepath}")
        sys.stdout.flush()
    except Exception as e:
        print(f"  Download error: {e}")
        return False, 0

    # Validate
    if filesize < 200_000:
        print(f"  WARNING: Suspiciously small ({size_kb:.0f} KB < 200 KB)")
        return False, filesize

    print(f"  OK ({size_kb:.0f} KB)")
    return True, filesize


def main():
    print("=" * 60)
    print("CleanWalker Render Generation v2.0")
    print(f"Model: Seedream 4.5 via Replicate")
    print(f"Renders: {len(RENDERS)}")
    print(f"Output: {OUTPUT_DIR}")
    print("=" * 60)
    sys.stdout.flush()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    stats = {"first_try": 0, "retry": 0, "failed": 0}
    details = []

    for i, render in enumerate(RENDERS):
        print(f"\n>>> [{i+1}/{len(RENDERS)}] {render['filename']}")
        sys.stdout.flush()

        success, filesize = generate_render(render, attempt=1)

        if not success:
            print(f"  RETRYING {render['filename']}...")
            sys.stdout.flush()
            success, filesize = generate_render(render, attempt=2)
            if success:
                stats["retry"] += 1
            else:
                stats["failed"] += 1
        else:
            stats["first_try"] += 1

        details.append({
            "filename": render["filename"],
            "success": success,
            "filesize": filesize,
        })

    # Summary
    print("\n" + "=" * 60)
    print("GENERATION SUMMARY")
    print("=" * 60)
    print(f"First-try success:  {stats['first_try']}")
    print(f"Retry success:      {stats['retry']}")
    print(f"Failed:             {stats['failed']}")
    print(f"Total:              {len(RENDERS)}")
    print()
    for d in details:
        status = "OK" if d["success"] else "FAIL"
        kb = d["filesize"] / 1024 if d["filesize"] else 0
        print(f"  [{status:4s}] {d['filename']:35s} {kb:6.0f} KB")
    print("=" * 60)
    sys.stdout.flush()

    return stats


if __name__ == "__main__":
    stats = main()
    sys.exit(0 if stats["failed"] == 0 else 1)
