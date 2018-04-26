#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"

using namespace std;

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
    pair<list<Vertex>, Vector3D> seed_triangle(list<Vertex> vertices);
    list<Vector3D> get_center(Vector3D p1, Vector3D p2, Vector3D p3);
    list<Edge> join(list<Edge> front, Vertex vk, Edge e, list<Vertex> vertices);
    void glue(list<Edge> front, Vertex vk, Edge e, list<Vertex> vertices);
    void output_triangle(list<Vertex> triangle);
  };
}

#endif // STUDENT_CODE_H
