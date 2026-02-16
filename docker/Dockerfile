FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-venv \
    build-essential \
    ca-certificates \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN python3 -m venv /opt/venv

# Activate venv by default
ENV PATH="/opt/venv/bin:$PATH"

# Upgrade pip inside venv
RUN pip install --upgrade pip setuptools wheel

# Install torch CPU first
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Install ultralytics
RUN pip install ultralytics numpy

WORKDIR /workspaces/repairman_ws

CMD ["bash"]