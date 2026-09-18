#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
version="1.10.0"
archive="zenoh-plugin-ros2dds-${version}-x86_64-unknown-linux-gnu-standalone.zip"
sha256="77ecb5fca3f9254ee5f82c19200c2eef5cdfa73cbb8b628a84441d45fec78165"
install_dir="$repo_root/.pixi/tools/zenoh-bridge-ros2dds/$version"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "The pinned bridge package requires Linux x86-64" >&2
  exit 1
fi
for dependency in curl unzip sha256sum; do
  command -v "$dependency" >/dev/null || { echo "$dependency is required" >&2; exit 1; }
done
tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT
curl --fail --location --proto '=https' --tlsv1.2 \
  --output "$tmp_dir/$archive" \
  "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/$version/$archive"
printf '%s  %s\n' "$sha256" "$tmp_dir/$archive" | sha256sum --check --status
unzip -q "$tmp_dir/$archive" zenoh-bridge-ros2dds -d "$tmp_dir"
mkdir -p "$install_dir"
install -m 755 "$tmp_dir/zenoh-bridge-ros2dds" "$install_dir/zenoh-bridge-ros2dds"
echo "Installed checksum-verified bridge: $install_dir/zenoh-bridge-ros2dds"
