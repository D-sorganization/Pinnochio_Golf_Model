#!/usr/bin/env bash
set -euo pipefail

echo "=== Gepetto Viewer Setup (conda 'viz' environment) ==="

# ------------------------------------------------------------------------------
# 1. Ensure conda (Miniforge) is installed
# ------------------------------------------------------------------------------

CONDA_DIR="${HOME}/miniforge3"

if [ ! -d "${CONDA_DIR}" ]; then
  echo ">>> Miniforge not found, installing to ${CONDA_DIR} ..."
  cd "${HOME}"
  wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O Miniforge3-Linux-x86_64.sh
  bash Miniforge3-Linux-x86_64.sh -b
  rm Miniforge3-Linux-x86_64.sh
else
  echo ">>> Miniforge already present at ${CONDA_DIR} (reusing)."
fi

# shellcheck disable=SC1090
source "${CONDA_DIR}/etc/profile.d/conda.sh"

# ------------------------------------------------------------------------------
# 2. Create (or reuse) the 'viz' conda environment
# ------------------------------------------------------------------------------

ENV_NAME="viz"

if conda env list | grep -qE "^${ENV_NAME} "; then
  echo ">>> Conda env '${ENV_NAME}' already exists (reusing)."
else
  echo ">>> Creating conda env '${ENV_NAME}' with gepetto-viewer and pinocchio ..."
  conda create -y -n "${ENV_NAME}" -c conda-forge python=3.10 gepetto-viewer gepetto-viewer-corba pinocchio
fi

echo ">>> Activating '${ENV_NAME}' environment..."
conda activate "${ENV_NAME}"

# ------------------------------------------------------------------------------
# 3. Sanity check: launch Gepetto backend and basic Pinocchio call
# ------------------------------------------------------------------------------

echo ">>> Running Pinocchio sanity check in 'viz' env..."

python - << 'EOF'
import pinocchio as pin

print("Pinocchio (viz env):", pin.__version__)
print("Model frames example:", pin.buildSampleModelManipulator().nframes)
print("Gepetto Viewer is installed; run 'gepetto-gui' in this env to start the GUI.")
EOF

echo "=== Gepetto setup complete. Usage: ==="
echo "  1) source ${CONDA_DIR}/etc/profile.d/conda.sh"
echo "  2) conda activate viz"
echo "  3) gepetto-gui"
