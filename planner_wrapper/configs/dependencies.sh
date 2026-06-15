#!/bin/bash

set -e

# Python (assumindo que você já tem Python 3.10)
python3 -m pip install --upgrade pip setuptools wheel

# Base científica
pip install numpy==1.26.4 scipy==1.15.3 pandas==2.3.3 matplotlib==3.8.4 scikit-learn==1.7.2

# Deep Learning (CUDA 12.x)
pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu124

# CUDA libs (normalmente vêm com torch, mas mantendo compatibilidade)
pip install \
nvidia-cublas-cu12==12.4.5.8 \
nvidia-cuda-cupti-cu12==12.4.127 \
nvidia-cuda-nvrtc-cu12==12.4.127 \
nvidia-cuda-runtime-cu12==12.4.127 \
nvidia-cudnn-cu12==9.1.0.70 \
nvidia-cufft-cu12==11.2.1.3 \
nvidia-curand-cu12==10.3.5.147 \
nvidia-cusolver-cu12==11.6.1.9 \
nvidia-cusparse-cu12==12.3.1.170 \
nvidia-nccl-cu12==2.21.5 \
nvidia-nvjitlink-cu12==12.4.127 \
nvidia-nvtx-cu12==12.4.127

# Visão computacional
pip install opencv-python==4.9.0.80 open3d==0.19.0 imageio==2.37.0 imageio-ffmpeg==0.6.0

# HuggingFace / Diffusion
pip install diffusers==0.33.1 huggingface-hub==1.7.2 safetensors==0.7.0 transformers accelerate

# Web / APIs
pip install fastapi==0.135.1 uvicorn==0.42.0 starlette==0.52.1 flask==3.1.1

# Interface / visualização
pip install gradio==5.31.0 gradio-client==1.10.1 plotly==6.6.0 dash==4.0.0

# Utilitários principais
pip install \
addict==2.4.0 \
einops==0.8.2 \
hydra-core==1.3.2 \
omegaconf==2.3.0 \
pyyaml==6.0.3 \
tqdm==4.67.3 \
requests==2.32.3 \
pydantic==2.11.10

# Restante (geral)
pip install \
aiofiles anyio attrs click decorator filelock fsspec h11 httpx \
ipython ipywidgets jinja2 joblib jsonschema jupyter-core \
kiwisolver markdown-it-py mpmath networkx pillow platformdirs \
psutil pygments pyparsing python-dateutil pytz regex rich \
sympy threadpoolctl typer typing-extensions urllib3 werkzeug \
casadi open3d imageio

echo "Instalação concluída."
