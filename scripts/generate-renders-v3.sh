#!/usr/bin/env bash
set -uo pipefail

# Load API token
export REPLICATE_API_TOKEN=$(grep REPLICATE_API_TOKEN ~/.openclaw/.env | cut -d= -f2)

# Base64 encode reference image
REF_B64=$(base64 -w0 /home/deploy/.openclaw/media/inbound/file_9---48a73478-a478-4607-b636-2a20f4e1ba33.jpg)
REF_URI="data:image/jpeg;base64,${REF_B64}"

OUTPUT_DIR="/home/deploy/cleanwalkerrobotics/apps/web/public/renders/v3"
mkdir -p "$OUTPUT_DIR"

PREFIX="Using this robot design exactly as shown in the reference image,"
SUFFIX="Maintain exact robot design from reference. No text, no logos, no branding, no watermarks. Photorealistic, 8K quality."

SUCCEEDED=0
FAILED=0
FAILED_NAMES=""

generate_render() {
  local FILENAME="$1"
  local PROMPT="$2"
  local OUTPATH="${OUTPUT_DIR}/${FILENAME}"

  # Skip if already exists and is >500KB
  if [[ -f "$OUTPATH" ]] && [[ $(stat -c%s "$OUTPATH" 2>/dev/null || echo 0) -gt 512000 ]]; then
    echo "SKIP: $FILENAME already exists and is >500KB"
    return 0
  fi

  echo ""
  echo "=========================================="
  echo "GENERATING: $FILENAME"
  echo "=========================================="

  # Write JSON payload to temp file to avoid arg length issues
  local TMPJSON=$(mktemp)
  python3 -c "
import json, sys
payload = {
    'input': {
        'prompt': sys.argv[1],
        'image_input': [sys.argv[2]],
        'output_format': 'png'
    }
}
with open(sys.argv[3], 'w') as f:
    json.dump(payload, f)
" "$PROMPT" "$REF_URI" "$TMPJSON"

  # Create prediction
  local RESPONSE
  RESPONSE=$(curl -s -w "\n__HTTP__%{http_code}" -X POST "https://api.replicate.com/v1/models/google/nano-banana/predictions" \
    -H "Authorization: Bearer ${REPLICATE_API_TOKEN}" \
    -H "Content-Type: application/json" \
    -d @"$TMPJSON")
  rm -f "$TMPJSON"

  local HTTP_CODE
  HTTP_CODE=$(echo "$RESPONSE" | grep "__HTTP__" | sed 's/__HTTP__//')
  RESPONSE=$(echo "$RESPONSE" | grep -v "__HTTP__")

  echo "HTTP: $HTTP_CODE"

  if [[ "$HTTP_CODE" != "201" && "$HTTP_CODE" != "200" ]]; then
    echo "ERROR: HTTP $HTTP_CODE for $FILENAME"
    echo "$RESPONSE" | head -5
    return 1
  fi

  local PRED_ID
  PRED_ID=$(echo "$RESPONSE" | jq -r '.id // empty')
  local PRED_URL="https://api.replicate.com/v1/predictions/${PRED_ID}"

  echo "Prediction ID: $PRED_ID"
  echo "Polling..."

  # Poll until done (max 5 minutes = 60 * 5s)
  for i in $(seq 1 60); do
    sleep 5
    local POLL
    POLL=$(curl -s -H "Authorization: Bearer ${REPLICATE_API_TOKEN}" "$PRED_URL")
    local PSTATUS
    PSTATUS=$(echo "$POLL" | jq -r '.status // empty')

    if [[ "$PSTATUS" == "succeeded" ]]; then
      local OUTPUT_URL
      OUTPUT_URL=$(echo "$POLL" | jq -r 'if (.output | type) == "array" then .output[0] else .output end // empty')
      if [[ -n "$OUTPUT_URL" && "$OUTPUT_URL" != "null" ]]; then
        curl -s -o "$OUTPATH" "$OUTPUT_URL"
        local FSIZE
        FSIZE=$(stat -c%s "$OUTPATH" 2>/dev/null || echo 0)
        echo "SUCCESS: $FILENAME (${FSIZE} bytes) after $((i * 5))s"
        return 0
      else
        echo "ERROR: no output URL for $FILENAME"
        return 1
      fi
    elif [[ "$PSTATUS" == "failed" || "$PSTATUS" == "canceled" ]]; then
      local ERR
      ERR=$(echo "$POLL" | jq -r '.error // "unknown"')
      echo "FAILED: $FILENAME ($PSTATUS): $ERR"
      return 1
    fi

    echo "  [$((i * 5))s] status=$PSTATUS"
  done

  echo "TIMEOUT: $FILENAME after 300s"
  return 1
}

# Define all 14 renders
declare -a FILENAMES=(
  "hero-park.png"
  "hero-sidewalk.png"
  "hero-fleet.png"
  "detail-gripper.png"
  "detail-sensors.png"
  "detail-side-profile.png"
  "detail-charging-dock.png"
  "lifestyle-city-worker.png"
  "lifestyle-night-ops.png"
  "lifestyle-before-after.png"
  "tech-exploded-view.png"
  "tech-dashboard-mockup.png"
  "component-actuator.png"
  "component-pcb.png"
)

declare -a PROMPTS=(
  "${PREFIX} the robot stands in a lush urban park, its mechanical gripper precisely picking up a crumpled plastic bottle from the grass. Morning golden hour sunlight streams through the trees, casting warm highlights on the olive-green body panels. Green LED eyes glow softly. The robotic arm extends toward the bottle while the bag roll system on the back is visible. Shallow depth of field blurs the park background beautifully. ${SUFFIX}"
  "${PREFIX} the robot walks confidently along a European cobblestone sidewalk in a historic city center. Dynamic mid-stride walking pose showing articulated legs in motion. The olive-green body catches the diffused light of an overcast sky. Green LED eyes illuminate forward. The metal frame at the rear holds a hanging trash bag. Stone buildings and cafe terraces visible in the background. ${SUFFIX}"
  "${PREFIX} three identical olive-green robots work together in a large urban park. The foreground robot picks up a crushed aluminum can with its gripper arm. A second robot walks in the mid-ground carrying a full trash bag on its rear frame. A third robot in the background deposits a sealed bag at the curb. All three share the same design with green LED eyes, bag roll systems, and metal frames. Bright daylight, wide shot. ${SUFFIX}"
  "${PREFIX} extreme close-up macro photography of the robotic mechanical gripper firmly grasping a crushed aluminum soda can. The gripper fingers show precise articulation and mechanical detail. In the soft-focus background, the robot body is visible with a partially filled trash bag hanging from the rear metal frame containing collected litter. Studio-quality macro lighting with sharp focus on the gripper mechanism. ${SUFFIX}"
  "${PREFIX} front view close-up portrait of the robot sensor head showing the distinctive green LED eye panels glowing with inner light. The olive-green body panels frame the sensor array. Studio lighting setup with soft rim light creating a halo effect around the head. Clean dark background. The green LEDs cast a subtle glow on the surrounding panels. ${SUFFIX}"
  "${PREFIX} clean side profile view of the robot standing on a white and light gray studio background. Full body visible in neutral standing pose. All key features clearly shown: olive-green body panels, green LED eyes, robotic arm with gripper at the side, bag roll mounted on the back, metal frame at the rear with hanging trash bag. Professional product photography with even studio lighting. ${SUFFIX}"
  "${PREFIX} the robot is parked on a small charging dock station in a park setting. The rear metal frame is folded down flat against the body in a compact resting configuration. A charging cable connects to the dock. Evening golden hour light creates warm tones across the olive-green body. Green LED eyes glow dimly in standby mode. Trees and a park path visible in the background. ${SUFFIX}"
  "${PREFIX} a city maintenance worker wearing a high-visibility yellow vest stands next to the robot in a park, checking a tablet device. The robot reaches only to the worker knee height, emphasizing its compact size. The trash bag on the rear frame is half-full with collected litter. The worker looks pleased while reviewing data on the tablet. Green park setting with trees and benches. Natural daylight. ${SUFFIX}"
  "${PREFIX} the robot patrols a city street at night in a cinematic scene. The green LED eyes glow brightly, casting green light onto the wet sidewalk pavement. Street lamps create pools of warm light in the background. The olive-green body panels reflect the ambient city lights. Moody cinematic lighting with blue and orange color grading. The robotic arm is extended, scanning for litter. Urban nighttime atmosphere. ${SUFFIX}"
  "${PREFIX} split image composition. Left half shows an urban park area with scattered litter, plastic bottles, cans, and paper waste on the grass and pathway, looking messy and neglected. Right half shows the exact same park area now pristine and clean, with the robot visible in the distance walking away, and a neatly sealed trash bag deposited at the curb. Clear visual contrast between dirty and clean. Daylight. ${SUFFIX}"
  "${PREFIX} technical exploded view diagram on a clean white background. The robot components float in space showing the internal assembly: olive-green outer panels separated from the frame, leg actuators and servos, the sensor head with green LED arrays, battery pack, robotic arm assembly with gripper, the bag roll system, and the rear metal frame. Components arranged vertically with clear spacing between each layer. Technical illustration style. ${SUFFIX}"
  "${PREFIX} split composition. Left side shows the robot actively collecting litter in a green park, gripper reaching for a plastic bottle. Right side shows a fleet management dashboard displayed on a modern monitor screen, featuring a city map with robot location pins, battery status bars, collection statistics, and route paths. Clean modern UI design on the dashboard. Office and park environments. ${SUFFIX}"
  "${PREFIX} close-up product shot of a brushless servo actuator motor unit placed on a reflective dark surface. The actuator shows precision engineering with visible mounting points and cable connections. In the soft focus background, the complete robot stands showing how the actuator integrates into the leg joints. Studio lighting with dramatic reflections on the surface. ${SUFFIX}"
  "${PREFIX} close-up macro photography of a green printed circuit board with surface-mount SMD electronic components, microprocessors, capacitors, and trace routing clearly visible. The PCB sits on an anti-static surface. In the soft focus background, the complete robot is visible showing the electronics housing location. Clean macro photography lighting revealing every component detail. ${SUFFIX}"
)

for idx in "${!FILENAMES[@]}"; do
  if generate_render "${FILENAMES[$idx]}" "${PROMPTS[$idx]}"; then
    SUCCEEDED=$((SUCCEEDED + 1))
  else
    FAILED=$((FAILED + 1))
    FAILED_NAMES="${FAILED_NAMES} ${FILENAMES[$idx]}"
  fi
  # Small delay between renders to avoid rate limits
  sleep 2
done

echo ""
echo "=========================================="
echo "RENDER GENERATION COMPLETE"
echo "Succeeded: $SUCCEEDED / $((SUCCEEDED + FAILED))"
echo "Failed: $FAILED"
if [[ -n "$FAILED_NAMES" ]]; then
  echo "Failed renders:${FAILED_NAMES}"
fi
echo "=========================================="
echo ""
echo "Output files:"
ls -lh "${OUTPUT_DIR}/" 2>/dev/null || echo "No files found"

# Save summary
echo "${SUCCEEDED}|${FAILED}|${FAILED_NAMES}" > /tmp/render-summary.txt
