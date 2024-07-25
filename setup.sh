#! /bin/bash

required_cmake_version="3.20.5"
required_python_version="3.8.2"
required_dtc_version="1.4.7-3ubuntu2"
gn_dir="gn"

# Function to compare version numbers
version_compare() {
    local v1=$1
    local v2=$2
    if [[ "$(printf '%s\n' "$v1" "$v2" | sort -V | head -n1)" == "$v1" ]]; then
        return 1  # v1 is less than or equal to v2
    else
        return 0  # v1 is greater than v2
    fi
}

if [[ "$(expr substr $(uname -s) 1 5)" == "Linux" ]]; then
  sudo apt update > /dev/null
  sudo apt upgrade -y > /dev/null
  sudo apt install -y wget > /dev/null
  # Get current Ubuntu version
  ubuntu_version=$(lsb_release -rs)

  # Target Ubuntu version to compare against (22.04)
  target_version="22.04"

  # Compare versions
  if version_compare "$ubuntu_version" "$target_version"; then
    mkdir -p kitware
    cd kitware
    wget https://apt.kitware.com/kitware-archive.sh
    sudo bash kitware-archive.sh
    cd ..
  fi

  sudo apt install --no-install-recommends -y git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler wget \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel python3-venv xz-utils file \
    make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1 > /dev/null

  cmake_version=$(cmake --version | grep -oP "(?<=version )[0-9]+\.[0-9]+\.[0-9]+")

  if ! version_compare "$cmake_version" "$required_cmake_version"; then
    echo "CMake version $cmake_version is not sufficient (< $required_cmake_version)"
    exit 1
  fi

  python_version=$(python3 --version | cut -d' ' -f2)

  if ! version_compare "$python_version" "$required_python_version"; then
    echo "Python version $python_version is not sufficient (< $required_python_version)"
    exit 1
  fi

  dtc_version=$(dtc --version | grep -oP "(?<=Version: DTC )[0-9]+\.[0-9]+\.[0-9]+")

  if ! version_compare "$dtc_version" "$required_dtc_version"; then
      echo "DTC version $dtc_version is not sufficient (< $required_dtc_version)"
      exit 1
  fi

  if [ ! -d "$gn_dir" ]; then
    mkdir gn && cd gn
    wget -O gn.zip https://chrome-infra-packages.appspot.com/dl/gn/gn/linux-amd64/+/latest
    unzip gn.zip
    rm gn.zip
    cd ..
  fi

  # TODO: export gn path in makefile

  # Create environment
  python3 -m venv .venv
  source .venv/bin/activate
  pip install west

  mkdir -p ncs && cd ncs

  west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.6.1
  west update
  west zephyr-export

  pip install -r zephyr/scripts/requirements.txt
  pip install -r nrf/scripts/requirements.txt
  pip install -r bootloader/mcuboot/scripts/requirements.txt
  cd ..

  wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/zephyr-sdk-0.16.1_linux-x86_64.tar.xz
  wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/sha256.sum | shasum --check --ignore-missing
  tar xvf zephyr-sdk-0.16.1_linux-x86_64.tar.xz
  rm zephyr-sdk-0.16.1_linux-x86_64.tar.xz

  cd zephyr-sdk-0.16.1
  ./setup.sh
  cd ..

  sudo cp zephyr-sdk-0.16.1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
  sudo udevadm control --reload

  mkdir nrfcmdtools
  cd nrfcmdtools
  wget https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/nrf-command-line-tools_10.24.2_amd64.deb
  sudo dpkg -i nrf-command-line-tools_10.24.2_amd64.deb
  sudo apt-get install -f
  cd ..

  cd JLink
  sudo dpkg -i JLink_Linux_V798a_x86_64.deb
  sudo apt-get install -f
  cd ..
fi
