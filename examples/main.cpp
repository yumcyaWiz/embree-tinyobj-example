#include <embree3/rtcore.h>

#include <iostream>
#include <limits>

#include "camera.h"
#include "core.h"
#include "image.h"
#include "scene.h"

enum class AOV {
  DEPTH,
  POSITION,
  NORMAL,
  TEXCOORDS,
  BARYCENTRIC,
  DIFFUSE,
  SPECULAR,
  EMISSION
};

int main() {
  const uint32_t width = 512;
  const uint32_t height = 512;

  const AOV aov = AOV::NORMAL;

  const Vec3 camPos = Vec3(0, 1, 6.5);
  const Vec3 camForward = Vec3(0, 0, -1);
  const float FOV = 0.25f * PI;

  Image image(width, height);

  Camera camera(camPos, camForward, FOV);

  Scene scene;
  scene.loadModel("CornellBox-Water.obj");
  scene.build();

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (uint32_t i = 0; i < height; ++i) {
    for (uint32_t j = 0; j < width; ++j) {
      Vec2 uv;
      uv[0] = (2.0f * j - width) / height;
      uv[1] = (2.0f * i - height) / height;

      Ray ray;
      if (camera.sampleRay(uv, ray)) {
        IntersectInfo info;
        if (scene.intersect(ray, info)) {
          switch (aov) {
            case AOV::DEPTH:
              image.setPixel(i, j, Vec3(info.t));
              break;
            case AOV::POSITION:
              image.setPixel(i, j, info.surfaceInfo.position);
              break;
            case AOV::NORMAL:
              image.setPixel(i, j, Vec3(0.5 * (info.surfaceInfo.normal + 1)));
              break;
            case AOV::TEXCOORDS:
              image.setPixel(i, j,
                             Vec3(info.surfaceInfo.texcoords[0],
                                  info.surfaceInfo.texcoords[1], 0));
              break;
            case AOV::BARYCENTRIC:
              image.setPixel(i, j,
                             Vec3(info.surfaceInfo.barycentric[0],
                                  info.surfaceInfo.barycentric[1], 0));
              break;
            case AOV::DIFFUSE:
              image.setPixel(i, j, info.material->kd);
              break;
            case AOV::SPECULAR:
              image.setPixel(i, j, info.material->ks);
              break;
            case AOV::EMISSION:
              image.setPixel(i, j, info.material->ke);
              break;
          }
        } else {
          image.setPixel(i, j, Vec3(0));
        }
      }
    }
  }

  image.writePPM("output.ppm");

  return 0;
}