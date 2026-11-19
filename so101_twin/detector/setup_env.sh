#!/usr/bin/env bash
# One-time setup of the detector environment (separate from the LeRobot env).
#   bash setup_env.sh            # conda env "locany" if conda exists, else ./.venv
# Re-running is safe.
set -euo pipefail
cd "$(dirname "$0")"
ENV=locany

if command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)"
  if ! conda env list | awk '{print $1}' | grep -qx "$ENV"; then
    echo "== creating conda env $ENV (python 3.10)"
    conda create -y -n "$ENV" python=3.10
  fi
  conda activate "$ENV"
  ACTIVATE="conda activate $ENV"
else
  echo "== conda not found, using ./.venv (needs python3.10 or 3.11)"
  PY=$(command -v python3.10 || command -v python3.11 || true)
  [ -n "$PY" ] || { echo "Install python3.10 (sudo apt install python3.10-venv) or conda first."; exit 1; }
  [ -d .venv ] || "$PY" -m venv .venv
  source .venv/bin/activate
  ACTIVATE="source $(pwd)/.venv/bin/activate"
fi

python -m pip install -U pip
echo "== PyTorch for CUDA 12.8 (needed for RTX 50-series / Blackwell)"
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
echo "== model requirements"
pip install -r requirements.txt

echo "== GPU check"
python - <<'EOF'
import torch
ok = torch.cuda.is_available()
print("CUDA available:", ok)
if ok:
    p = torch.cuda.get_device_properties(0)
    print(f"GPU: {p.name}, {p.total_memory / 2**30:.1f} GB, compute capability {p.major}.{p.minor}")
    x = torch.randn(256, 256, device="cuda", dtype=torch.bfloat16)
    print("bf16 matmul ok:", float((x @ x).float().abs().mean()) > 0)
EOF

echo "== downloading nvidia/LocateAnything-3B (about 8 GB, once)"
if command -v hf >/dev/null 2>&1; then
  hf download nvidia/LocateAnything-3B || echo "Download failed. If it says 401/403: accept the licence on the model page, then run: hf auth login"
else
  huggingface-cli download nvidia/LocateAnything-3B || echo "Download failed. If it says 401/403: accept the licence on the model page, then run: huggingface-cli login"
fi

echo
echo "Done. Next time just run:  $ACTIVATE"
