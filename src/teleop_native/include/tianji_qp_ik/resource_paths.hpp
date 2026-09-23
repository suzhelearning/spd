#pragma once

#include <filesystem>
#include <stdexcept>

namespace tianji_qp_ik {

// Resolve from the executable, never the cwd, a ROS overlay or another checkout.
inline std::filesystem::path runtimeResource(const std::filesystem::path& relative) {
  namespace fs = std::filesystem;
  const auto executable = fs::canonical("/proc/self/exe");
  const auto installed = executable.parent_path().parent_path() / "share/teleop_native";
  if (fs::is_directory(installed)) {
    const auto target = installed / relative;
    if (fs::exists(target)) return fs::canonical(target);
    throw std::runtime_error("installed teleop resource missing: " + target.string());
  }
  // Also support running directly from this checkout's .teleop/build tree.
  for (auto root = executable.parent_path(); !root.empty(); root = root.parent_path()) {
    const auto source = root / "src/teleop_native";
    if (fs::is_regular_file(root / "pixi.toml") && fs::is_directory(source)) {
      const auto target = source / relative;
      if (fs::exists(target)) return fs::canonical(target);
      throw std::runtime_error("teleop resource missing: " + target.string());
    }
    if (root == root.root_path()) break;
  }
  throw std::runtime_error("cannot locate standalone teleop resources beside executable");
}

// Explicit paths stay explicit; relative paths belong to their own profile.
inline std::filesystem::path controllerResource(
    const std::filesystem::path& profile, const std::filesystem::path& value) {
  namespace fs = std::filesystem;
  if (value.is_absolute()) return value;
  const auto local = fs::absolute(profile).parent_path() / value;
  return fs::exists(local) ? fs::canonical(local) : local.lexically_normal();
}

}  // namespace tianji_qp_ik
