#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "install_build_deps_ubuntu.sh supports Ubuntu Linux only" >&2
  exit 2
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root: sudo ./scripts/install_build_deps_ubuntu.sh" >&2
  exit 2
fi

# shellcheck disable=SC1091
export DEBIAN_FRONTEND=noninteractive

# Optional arguments are additional apt packages needed by a particular build
# lane (for example the exact runner kernel's virtual-CAN module package).
extra_packages=("$@")

apt-get update
apt-get install --no-install-recommends --yes \
  build-essential \
  ca-certificates \
  cmake \
  ccache \
  git \
  libboost-filesystem-dev \
  libboost-program-options-dev \
  libboost-serialization-dev \
  libboost-system-dev \
  libeigen3-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libzmq3-dev \
  ninja-build \
  pkg-config \
  "${extra_packages[@]}"

echo "Installed arm_controls source-build dependencies for Ubuntu."
