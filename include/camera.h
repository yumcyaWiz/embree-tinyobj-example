#ifndef _CAMERA_H
#define _CAMERA_H

#include <spdlog/spdlog.h>

#include <cmath>

#include "core.h"

// pinhole camera
class Camera {
 private:
  Vec3 position;
  Vec3 forward;
  Vec3 right;
  Vec3 up;

  float FOV;
  float focal_length;

 public:
  Camera(const Vec3& position, const Vec3& forward, float FOV = 0.5f * PI)
      : position(position), forward(forward) {
    right = normalize(cross(forward, Vec3(0, 1, 0)));
    up = normalize(cross(right, forward));

    spdlog::info("[Camera] position: ({}, {}, {})", position[0], position[1],
                 position[2]);
    spdlog::info("[Camera] forward: ({}, {}, {})", forward[0], forward[1],
                 forward[2]);
    spdlog::info("[Camera] right: ({}, {}, {})", right[0], right[1], right[2]);
    spdlog::info("[Camera] up: ({}, {}, {})", up[0], up[1], up[2]);

    // compute focal length from FOV
    focal_length = 1.0f / std::tan(0.5f * FOV);

    spdlog::info("[Camera] focal_length: {}", focal_length);
  }

  bool sampleRay(const Vec2& uv, Ray& ray) const {
    const Vec3 pinholePos = position + focal_length * forward;
    const Vec3 sensorPos = position + uv[0] * right + uv[1] * up;
    ray = Ray(sensorPos, normalize(pinholePos - sensorPos));
    return true;
  }
};

#endif