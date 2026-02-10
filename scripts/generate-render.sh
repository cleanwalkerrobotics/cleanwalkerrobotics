#!/bin/bash
# Generate a render via Replicate API
# Usage: ./generate-render.sh <model> <prompt> <output_path>
# Models: "seedream" or "flux"

set -euo pipefail

MODEL="$1"
PROMPT="$2"
OUTPUT="$3"

if [ "$MODEL" = "seedream" ]; then
  MODEL_ID="bytedance/seedream-4.5"
  BODY=$(jq -n --arg prompt "$PROMPT" '{
    input: {
      prompt: $prompt,
      aspect_ratio: "16:9",
      num_outputs: 1
    }
  }')
elif [ "$MODEL" = "flux" ]; then
  MODEL_ID="black-forest-labs/flux-1.1-pro"
  BODY=$(jq -n --arg prompt "$PROMPT" '{
    input: {
      prompt: $prompt,
      aspect_ratio: "16:9"
    }
  }')
else
  echo "Unknown model: $MODEL"
  exit 1
fi

# Create prediction
RESPONSE=$(curl -s -X POST "https://api.replicate.com/v1/models/${MODEL_ID}/predictions" \
  -H "Authorization: Bearer ${REPLICATE_API_TOKEN}" \
  -H "Content-Type: application/json" \
  -H "Prefer: wait" \
  -d "$BODY")

# Check for errors
ERROR=$(echo "$RESPONSE" | jq -r '.error // empty')
if [ -n "$ERROR" ]; then
  echo "API ERROR: $ERROR"
  exit 1
fi

STATUS=$(echo "$RESPONSE" | jq -r '.status')

if [ "$STATUS" = "succeeded" ]; then
  # Get output URL
  if [ "$MODEL" = "flux" ]; then
    IMG_URL=$(echo "$RESPONSE" | jq -r '.output')
  else
    IMG_URL=$(echo "$RESPONSE" | jq -r '.output[0] // .output')
  fi

  if [ -z "$IMG_URL" ] || [ "$IMG_URL" = "null" ]; then
    echo "No output URL in response"
    echo "$RESPONSE" | jq .
    exit 1
  fi

  # Download
  curl -s -o "$OUTPUT" "$IMG_URL"
  echo "SUCCESS: Saved to $OUTPUT ($(du -h "$OUTPUT" | cut -f1))"
  exit 0
else
  # Need to poll
  PREDICTION_URL=$(echo "$RESPONSE" | jq -r '.urls.get')
  if [ -z "$PREDICTION_URL" ] || [ "$PREDICTION_URL" = "null" ]; then
    echo "No prediction URL, status: $STATUS"
    echo "$RESPONSE" | jq .
    exit 1
  fi

  for i in $(seq 1 60); do
    sleep 5
    POLL=$(curl -s "$PREDICTION_URL" \
      -H "Authorization: Bearer ${REPLICATE_API_TOKEN}")
    STATUS=$(echo "$POLL" | jq -r '.status')

    if [ "$STATUS" = "succeeded" ]; then
      if [ "$MODEL" = "flux" ]; then
        IMG_URL=$(echo "$POLL" | jq -r '.output')
      else
        IMG_URL=$(echo "$POLL" | jq -r '.output[0] // .output')
      fi
      curl -s -o "$OUTPUT" "$IMG_URL"
      echo "SUCCESS: Saved to $OUTPUT ($(du -h "$OUTPUT" | cut -f1))"
      exit 0
    elif [ "$STATUS" = "failed" ] || [ "$STATUS" = "canceled" ]; then
      echo "FAILED: $STATUS"
      echo "$POLL" | jq -r '.error // "unknown error"'
      exit 1
    fi
    echo "Polling... ($STATUS, attempt $i/60)"
  done
  echo "TIMEOUT: prediction did not complete in 5 minutes"
  exit 1
fi
