#include <embree3/rtcore.h>

#include <iostream>
#include <limits>

#include "camera.h"
#include "core.h"
#include "image.h"
#include "scene.h"

int main() {
  const uint32_t width = 512;
  const uint32_t height = 512;
  const Vec3 camPos = Vec3(0, 0, 5);
  const Vec3 camForward = Vec3(0, 0, -1);
  const float FOV = PI_DIV_4;

  Image image(width, height);

  Camera camera(camPos, camForward, FOV);

  Scene scene;
  scene.loadModel("test.obj");
  scene.build();

  for (uint32_t i = 0; i < height; ++i) {
    for (uint32_t j = 0; j < width; ++j) {
      Vec2 uv;
      uv[0] = (2.0f * j - width) / height;
      uv[1] = (2.0f * i - height) / height;

      Ray ray;
      if (camera.sampleRay(uv, ray)) {
        IntersectInfo info;
        if (scene.intersect(ray, info)) {
          image.setPixel(i, j, Vec3(info.t));
        } else {
          image.setPixel(i, j, Vec3(0));
        }
      }
    }
  }

  image.writePPM("output.ppm");

  return 0;
}