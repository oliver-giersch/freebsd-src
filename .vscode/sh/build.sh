#!/bin/sh

set -euo pipefail

exit_error() {
	echo $1 >&2
	exit 1
}

readonly kernconf=${1:-GENERIC}
readonly root=$(pwd)
readonly build=".build"

readonly id=$(grep ^ID= /etc/os-release | sed "s/ID=//g")
case "$id" in
	"arch") xbindir=/usr/bin ;;
	"fedora") xbindir=/usr/lib64/llvm18/bin ;;
	*) exit_error "only implemented for [arch|fedora]";;
esac

mkdir -p "$build"
export MAKEOBJDIRPREFIX=$(realpath "$build")

readonly make="./tools/build/make.py"
readonly makeargs="--debug --cross-bindir=$xbindir \
    TARGET=amd64 TARGET_ARCH=amd64 -s -j$(nproc)"

$make $makeargs kernel-toolchain -DWITH_DISK_IMAGE_TOOLS_BOOTSTRAP
bear --append -- $make $makeargs KERNCONF="$kernconf" NO_MODULES=yes buildkernel

