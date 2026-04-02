/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 #include "camera_listener.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <cstdlib>
#include <optional>

#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>

#include "gtsam_utils.h"

using std::vector;
using namespace gtsam;

namespace {
struct HotvisionCameraInfo {
  Cal3_S2 intrinsics;
  Pose3 robotTcamera;
};

std::optional<wpi::json> LoadHotvisionConfig() {
  const char* envPath = std::getenv("HOTVISION_CONFIG");
  std::string path =
      envPath ? std::string(envPath) : std::string{"./.config/camera_config.json"};

  auto fileBuffer = wpi::MemoryBuffer::GetFile(path);
  if (!fileBuffer) {
    fmt::println("HOTVision: failed to read camera config at '{}'", path);
    return std::nullopt;
  }

  try {
    wpi::json json = wpi::json::parse(fileBuffer.value()->GetCharBuffer());
    return json;
  } catch (const std::exception& e) {
    fmt::println("HOTVision: failed to parse camera config JSON: {}", e.what());
    return std::nullopt;
  }
}

std::optional<HotvisionCameraInfo> GetHotvisionCameraInfo(
    std::string_view cameraName) {
  static std::optional<wpi::json> cachedJson = LoadHotvisionConfig();
  if (!cachedJson) {
    return std::nullopt;
  }

  auto it = cachedJson->find(std::string(cameraName));
  if (it == cachedJson->end()) {
    fmt::println("HOTVision: camera '{}' not found in config JSON", cameraName);
    return std::nullopt;
  }

  const auto& cam = *it;
  if (!cam.contains("K")) {
    fmt::println("HOTVision: camera '{}' missing K", cameraName);
    return std::nullopt;
  }

  auto K = cam.at("K");
  if (!K.is_array() || K.size() != 3 || !K[0].is_array() || !K[1].is_array()) {
    fmt::println("HOTVision: camera '{}' has malformed K matrix", cameraName);
    return std::nullopt;
  }

  double fx = K[0][0].get<double>();
  double cx = K[0][2].get<double>();
  double fy = K[1][1].get<double>();
  double cy = K[1][2].get<double>();

  // Transform can legitimately be null in HOTVision config; fall back to
  // identity (camera at robot origin) in that case so gtsam-node can still run.
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  if (cam.contains("transform") && !cam.at("transform").is_null()) {
    auto t = cam.at("transform");
    x = t.value("x", 0.0);
    y = t.value("y", 0.0);
    z = t.value("z", 0.0);
    roll = t.value("roll", 0.0);
    pitch = t.value("pitch", 0.0);
    yaw = t.value("yaw", 0.0);
  } else {
    fmt::println("HOTVision: camera '{}' has null/absent transform, using identity", cameraName);
  }

  // WPILib / HOTVision use NWU; GTSAM uses its own convention, but we treat
  // this Pose3 as robot->camera in the same coordinates as the field layout.
  Cal3_S2 intrinsics{fx, fy, 0.0, cx, cy};
  Pose3 robotTcam{Rot3::RzRyRx(roll, pitch, yaw), Point3{x, y, z}};

  HotvisionCameraInfo info{intrinsics, robotTcam};
  return info;
}
}  // namespace

CameraListener::CameraListener(std::string rootTable, CameraConfig config)
    : config(config),
      useHotvision(rootTable == "HOTVision"),
      tagSub(nt::NetworkTableInstance::GetDefault()
                 .GetStructArrayTopic<TagDetection>(
                     rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                     config.subtableName + "/input/tags")
                 .Subscribe({},
                            {
                                .pollStorage = 100,
                                .sendAll = true,
                                .keepDuplicates = true,
                            })),
      robotTcamSub(nt::NetworkTableInstance::GetDefault()
                       .GetStructTopic<frc::Transform3d>(
                           rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                           config.subtableName + "/input/robotTcam")
                       .Subscribe({},
                                  {
                                      .pollStorage = 1,
                                      .sendAll = false,
                                      .keepDuplicates = false,
                                  })),
      pinholeIntrinsicsSub(
          nt::NetworkTableInstance::GetDefault()
              .GetDoubleArrayTopic(
                  rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                  config.subtableName + "/input/cam_intrinsics")
              .Subscribe({},
                         {
                             .pollStorage = 1,
                             .sendAll = false,
                             .keepDuplicates = false,
                         })),
      hotvisionTagSub(
          nt::NetworkTableInstance::GetDefault()
              .GetDoubleArrayTopic(rootTable + "/" + config.subtableName +
                                   "/apriltag/detections")
              .Subscribe({},
                         {
                             .pollStorage = 100,
                             .sendAll = true,
                             .keepDuplicates = true,
                         })),
      measurementNoise(noiseModel::Isotropic::Sigma(2, config.pixelNoise)) {
  if (useHotvision) {
    // For HOTVision we load intrinsics/extrinsics from the JSON config once.
    auto info = GetHotvisionCameraInfo(config.subtableName);
    if (info) {
      cameraK = info->intrinsics;
      robotTcamera = info->robotTcamera;
      fmt::println("HOTVision: loaded camera '{}' intrinsics/extrinsics",
                   config.subtableName);
    } else {
      fmt::println("HOTVision: failed to load camera '{}'", config.subtableName);
    }
  }
}

bool CameraListener::ReadyToOptimize() {
  if (useHotvision) {
    if (!cameraK || !robotTcamera) {
      fmt::println(
          "HOTVision camera {}: intrinsics/extrinsics not available from "
          "config",
          config.subtableName);
      return false;
    }
    return true;
  }

  // PhotonVision / generic NT path (original behavior)

  // grab the latest camera cal
  const auto last_K = pinholeIntrinsicsSub.GetAtomic();
  // if not published, time will be zero
  if (last_K.time > 0) {
    // Update calibration!
    std::vector<double> K_ = last_K.value;
    if (K_.size() != 4) {
      fmt::println("Camera {}: K of odd size {}?", config.subtableName,
                   K_.size());
      return false;
    }
    // assume order is [fx fy cx cy] from NT
    auto newK = Cal3_S2{K_[0], K_[1],
                        0, // no skew
                        K_[2], K_[3]};
    if (!cameraK || !cameraK->equals(newK, 1e-6)) {
      cameraK = newK;
      cameraK->print("New camera calibration");
    }
  }
  if (!cameraK) {
    fmt::println("Camera {}: no intrinsics set?", config.subtableName);
    return false;
  }

  // grab the latest robot-cam transform
  const auto last_rTc = robotTcamSub.GetAtomic();
  // if not published, time will be zero
  if (last_rTc.time == 0) {
    fmt::println("Camera {}: no robot-cam set?", config.subtableName);
    return false;
  }

  robotTcamera =
      Transform3dToGtsamPose3(last_rTc.value)
      // add transform to change from the wpilib/photon default (camera optical
      // axis along +x) to the standard from opencv (z along optical axis)
      /*
      We want:
      x in [0,-1,0]
      y in [0,0,-1]
      z in [1,0,0]
      */
      * Pose3{Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), Point3{0.0, 0, 0.0}};

  return cameraK && robotTcamera;
}

std::vector<CameraVisionObservation> CameraListener::Update() {
  std::vector<CameraVisionObservation> ret;

  if (useHotvision) {
    const auto dets = hotvisionTagSub.ReadQueue();
    ret.reserve(dets.size());

    for (const auto& arr : dets) {
      const auto& vals = arr.value;
      std::size_t i = 0;
      while (i + 1 + 8 <= vals.size()) {
        int id = static_cast<int>(std::lround(vals[i++]));
        vector<Point2> cornersForGtsam;
        cornersForGtsam.reserve(4);
        for (int k = 0; k < 4; ++k) {
          double u = vals[i++];
          double v = vals[i++];
          cornersForGtsam.emplace_back(u, v);
        }

        ret.emplace_back(arr.time, id, cornersForGtsam, *cameraK, *robotTcamera,
                         measurementNoise);
      }
    }

    return ret;
  }

  const auto tags = tagSub.ReadQueue();

  ret.reserve(tags.size());

  // For each tag-array in the queue
  for (const auto &tarr : tags) {
    // For each tag in this tag array
    for (const auto &t : tarr.value) {
      vector<Point2> cornersForGtsam;
      cornersForGtsam.reserve(4);
      for (const auto &c : t.corners) {
        cornersForGtsam.emplace_back(c.first, c.second);
      }

      ret.emplace_back(tarr.time, t.id, cornersForGtsam, *cameraK,
                       *robotTcamera, measurementNoise);
    }
  }

  return ret;
}
