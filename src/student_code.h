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

    // Structs for the ball pivot algorithm
    struct vertex_struct {
      VertexIter v;
      bool used = false;
    };

    struct edge_struct {
      EdgeIter e;
      VertexIter v1;
      VertexIter v2;
      VertexIter v3;
    };

    friend bool operator==(const vertex_struct& v1, const vertex_struct& v2) {
      return v1.v == v2.v;
    };

    friend bool operator==(const edge_struct& e1, const edge_struct& e2) {
      return e1.e == e2.e;
    };

    void upsample(HalfedgeMesh& mesh);
    void ballPivot(HalfedgeMesh& mesh, double BPAr);
    int hash_position(Vector3D pos, double r, Vector3D dimensions, Vector3D minDimensions);
    Vector3D get_center(Vector3D p1, Vector3D p2, Vector3D p3, double r, Vector3D n);
    void join(HalfedgeMesh& mesh, list<edge_struct *>& front, VertexIter vi, VertexIter vj, 
      VertexIter vk, EdgeIter e_ij);
    void glue(HalfedgeMesh& mesh, list<edge_struct *>& front, VertexIter vi, VertexIter vj, 
      VertexIter vk, EdgeIter e_ij);
    bool find_seed_triangle(HalfedgeMesh& mesh, vector<vector <vertex_struct *> >& voxels, 
      vector<vertex_struct *> vertices, list<edge_struct *>& front, Vector3D dimensions, Vector3D minDimensions, double r);
  };
}

#endif // STUDENT_CODE_H
