// Configure Livox MID-360 hardware FOV through Livox-SDK2.
//
// The horizontal window is centred on the sensor front (yaw 0 degrees).
// Two FOV windows are required because the selected span crosses yaw 0.
// The vertical window is fixed to 0..+52 degrees. This removes only the
// downward-looking MID-360 band (-7..0 degrees).

// Example: keep 225 degrees horizontally and remove the lower -7..0 band:
//   ./set_mid360_fov MID360_config.json 225
//
// Restore the full hardware FOV:
//   ./set_mid360_fov MID360_config.json off


#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace {

constexpr int32_t kPitchStart = -7;
constexpr int32_t kPitchStop = 52;

std::atomic<bool> g_done{false};
double g_span_deg = 225.0;
bool g_disable = false;

void OnFovCfg0(livox_status status, uint32_t,
               LivoxLidarAsyncControlResponse* response, void*) {
  std::printf("[fov_cfg0] status=%d ret_code=%u\n", status,
              response ? response->ret_code : 0u);
}

void OnFovCfg1(livox_status status, uint32_t,
               LivoxLidarAsyncControlResponse* response, void*) {
  std::printf("[fov_cfg1] status=%d ret_code=%u\n", status,
              response ? response->ret_code : 0u);
}

void OnFovEnable(livox_status status, uint32_t,
                 LivoxLidarAsyncControlResponse* response, void*) {
  std::printf("[fov_enable] status=%d ret_code=%u\n", status,
              response ? response->ret_code : 0u);
  g_done = true;
}

void OnWorkMode(livox_status status, uint32_t,
                LivoxLidarAsyncControlResponse*, void*) {
  std::printf("[work_mode] status=%d\n", status);
}

void LidarInfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info,
                             void*) {
  if (info == nullptr) {
    std::printf("LiDAR info callback received nullptr\n");
    return;
  }

  std::printf("LiDAR connected: handle=%u sn=%s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, OnWorkMode, nullptr);

  if (g_disable) {
    std::printf("Disabling hardware FOV windows (full sensor FOV)\n");
    EnableLivoxLidarFov(handle, 0, OnFovEnable, nullptr);
    return;
  }

  // Truncation preserves the original 225-degree windows: 0..112 and
  // 248..359 (225 integer-degree directions in total).
  const int32_t half = static_cast<int32_t>(g_span_deg / 2.0);

  FovCfg cfg0{};
  cfg0.yaw_start = 0;
  cfg0.yaw_stop = half;
  cfg0.pitch_start = kPitchStart;
  cfg0.pitch_stop = kPitchStop;

  FovCfg cfg1{};
  cfg1.yaw_start = 360 - half;
  cfg1.yaw_stop = 359;
  cfg1.pitch_start = kPitchStart;
  cfg1.pitch_stop = kPitchStop;

  std::printf("window 0: yaw %d..%d pitch %d..%d\n", cfg0.yaw_start,
              cfg0.yaw_stop, cfg0.pitch_start, cfg0.pitch_stop);
  std::printf("window 1: yaw %d..%d pitch %d..%d\n", cfg1.yaw_start,
              cfg1.yaw_stop, cfg1.pitch_start, cfg1.pitch_stop);

  SetLivoxLidarFovCfg0(handle, &cfg0, OnFovCfg0, nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  SetLivoxLidarFovCfg1(handle, &cfg1, OnFovCfg1, nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // bit 0 enables window 0 and bit 1 enables window 1.
  EnableLivoxLidarFov(handle, 0x03, OnFovEnable, nullptr);
}

void PrintUsage(const char* program) {
  std::printf(
      "usage:\n"
      "  %s <config.json> <horizontal_span>\n"
      "  %s <config.json> off\n\n"
      "examples:\n"
      "  %s MID360_config.json 225\n"
      "  %s MID360_config.json off\n",
      program, program, program, program);
}

bool ParseArguments(int argc, char** argv) {
  if (argc < 3) {
    return false;
  }

  const std::string mode = argv[2];
  if (mode == "off") {
    g_disable = true;
    return argc == 3;
  }

  g_span_deg = std::atof(argv[2]);
  if (argc != 3 || g_span_deg <= 0.0 || g_span_deg >= 360.0) {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (!ParseArguments(argc, argv)) {
    PrintUsage(argv[0]);
    return 1;
  }

  if (!LivoxLidarSdkInit(argv[1])) {
    std::printf("LivoxLidarSdkInit failed; check config path and host IP\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  std::printf("Waiting for MID-360 to connect...\n");

  for (int i = 0; i < 150 && !g_done; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  const bool success = g_done.load();
  if (!success) {
    std::printf("Timed out: no completed response from the LiDAR\n");
  } else {
    std::printf("FOV configuration request completed\n");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  LivoxLidarSdkUninit();
  return success ? 0 : 1;
}
