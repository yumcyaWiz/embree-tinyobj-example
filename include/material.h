#ifndef _MATERIAL_H
#define _MATERIAL_H
#include "core.h"

class Material {
 public:
  const Vec3 kd;
  const Vec3 ks;
  const Vec3 ke;

  Material(const Vec3& kd, const Vec3& ks, const Vec3& ke)
      : kd(kd), ks(ks), ke(ke) {}
};

#endif