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
apt_options=()
if [[ -n "${ARM_CONTROLS_APT_CACHE_DIR:-}" ]]; then
  apt_archives_dir="${ARM_CONTROLS_APT_CACHE_DIR}/archives"
  install -d -m 0777 "${apt_archives_dir}" "${apt_archives_dir}/partial"
  apt_options=(
    -o "Dir::Cache::archives=${apt_archives_dir}"
    -o "APT::Keep-Downloaded-Packages=true"
  )
  if [[ "${ARM_CONTROLS_APT_CACHE_LISTS:-0}" == "1" ]]; then
    apt_lists_dir="${ARM_CONTROLS_APT_CACHE_DIR}/lists"
    apt_update_stamp="${ARM_CONTROLS_APT_CACHE_DIR}/update-complete"
    install -d -m 0777 "${apt_lists_dir}" "${apt_lists_dir}/partial"
    apt_options+=(-o "Dir::State::lists=${apt_lists_dir}")
  fi
fi

if [[ -z "${apt_update_stamp:-}" || ! -f "${apt_update_stamp}" ]]; then
  apt-get "${apt_options[@]}" update
  if [[ -n "${apt_update_stamp:-}" ]]; then
    touch "${apt_update_stamp}"
  fi
fi
apt-get "${apt_options[@]}" install --no-install-recommends --yes \
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

if [[ -n "${ARM_CONTROLS_APT_CACHE_DIR:-}" ]]; then
  # apt creates root-owned lock/partial entries even inside the workspace.
  # The cache action runs as the runner user and must be able to archive them.
  chmod -R a+rwX "${ARM_CONTROLS_APT_CACHE_DIR}"
fi

echo "Installed arm_controls source-build dependencies for Ubuntu."
