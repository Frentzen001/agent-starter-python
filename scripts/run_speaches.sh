#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${SPEACHES_CONTAINER_NAME:-speaches}"
IMAGE="${SPEACHES_IMAGE:-ghcr.io/speaches-ai/speaches:latest-cuda}"
PORT="${SPEACHES_PORT:-8000}"
VOLUME_NAME="${SPEACHES_VOLUME:-hf-hub-cache}"
USE_GPU="${SPEACHES_USE_GPU:-1}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required to start Speaches." >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "Speaches is already running in container '$CONTAINER_NAME'."
  exit 0
fi

docker_args=(
  run
  --rm
  --detach
  --publish "${PORT}:8000"
  --name "$CONTAINER_NAME"
  --volume "${VOLUME_NAME}:/home/ubuntu/.cache/huggingface/hub"
)

if [[ "$USE_GPU" == "1" ]]; then
  docker_args+=(--gpus all)
fi

docker_args+=("$IMAGE")

docker "${docker_args[@]}"
echo "Speaches is starting on http://127.0.0.1:${PORT}/v1"
