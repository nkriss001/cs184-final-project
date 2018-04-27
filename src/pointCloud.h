#ifndef CGL_POINTCLOUD_H
#define CGL_POINTCLOUD_H

#include "scene.h"
#include "material.h"

namespace CGL {

  struct PointCloud : Instance {

    std::vector<Vector3D> points;
    std::vector<Vector3D> point_normals;

  }; // struct PointCloud

} // namespace CGL

#endif // CGL_POINTCLOUD_H
