#include <embree3/rtcore.h>

#include <iostream>
#include <limits>

#include "core.h"
#include "scene.h"

int main() {
  Scene scene;
  scene.addModel("asdf.obj");
  return 0;
}