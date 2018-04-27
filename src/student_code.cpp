#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    int prevLevelSize = evaluatedLevels.back().size();
    if (prevLevelSize == 1) {
      return;
    } else {
      std::vector<Vector2D> newLevel;
      for (int i = 0; i < prevLevelSize-1; i++) {
        Vector2D newPoint = (1-t) * evaluatedLevels.back()[i] + t * evaluatedLevels.back()[i+1];
        newLevel.push_back(newPoint);
      }
      evaluatedLevels.push_back(newLevel);
    }
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> column;
    for (std::vector<Vector3D> row : controlPoints) {
      Vector3D point = evaluate1D(row, u);
      column.push_back(point);
    }
    Vector3D finalPoint = evaluate1D(column, v);

    return finalPoint;
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    std::vector< std::vector<Vector3D> > prevLevels;
    prevLevels.push_back(points);
    while (prevLevels.back().size() > 1) {
      std::vector<Vector3D> newLevel;
      for (int i = 0; i < prevLevels.back().size()-1; i++) {
        Vector3D newPoint = (1-t) * prevLevels.back()[i] + t * prevLevels.back()[i+1];
        newLevel.push_back(newPoint);
      }
      prevLevels.push_back(newLevel);
    }

    return prevLevels.back()[0];
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n(0, 0, 0);

    HalfedgeCIter h = halfedge();
    HalfedgeCIter h_orig = halfedge();
    do {
      Vector3D p0 = h->vertex()->position;
      Vector3D p1 = h->next()->vertex()->position;
      Vector3D p2 = h->next()->next()->vertex()->position;

      Vector3D e1 = p1 - p0;
      Vector3D e2 = p2 - p0;

      n += cross( e1, e2 );

      h = h->twin()->next();
    } while (h != h_orig);

    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()) {
      return e0;
    }

    // DEFINE MESH ELEMENTS
    // halfedges to flip
    HalfedgeIter h1 = e0->halfedge();
    HalfedgeIter h2 = h1->twin();
    // remaining halfedges
    HalfedgeIter h3 = h1->next();
    HalfedgeIter h4 = h2->next();
    HalfedgeIter h5 = h1->next()->next();
    HalfedgeIter h6 = h2->next()->next();

    // vertices
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h1->next()->next()->vertex();
    VertexIter v4 = h2->next()->next()->vertex();

    // faces
    FaceIter f1 = h1->face();
    FaceIter f2 = h2->face();


    // BEGIN CHANGING ASSIGNMENTS
    // set halfedges
    h1->next() = h6;
    h6->next() = h3;
    h3->next() = h1;
    h2->next() = h5;
    h5->next() = h4;
    h4->next() = h2;

    // set vertices
    h1->vertex() = v3;
    h2->vertex() = v4;
    v3->halfedge() = h1;
    v4->halfedge() = h2;
    v1->halfedge() = h4;
    v2->halfedge() = h3;

    // set faces
    h5->face() = f2;
    h6->face() = f1;
    f2->halfedge() = h5;
    f1->halfedge() = h6;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
      return e0->halfedge()->vertex();
    }

    // DEFINE MESH ELEMENTS
    // halfedges
    HalfedgeIter h1 = e0->halfedge();
    HalfedgeIter h2 = h1->twin();
    HalfedgeIter h3 = h1->next();
    HalfedgeIter h4 = h2->next();
    HalfedgeIter h5 = h1->next()->next();
    HalfedgeIter h6 = h2->next()->next();
    // new halfedges
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();

    // edges
    EdgeIter e1 = h1->edge();
    // new edges
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();
    EdgeIter e4 = newEdge();

    // vertices
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h1->next()->next()->vertex();
    VertexIter v4 = h2->next()->next()->vertex();
    // new, center vertex
    VertexIter v5 = newVertex();

    // faces
    FaceIter f1 = h1->face();
    FaceIter f2 = h2->face();
    // new faces
    FaceIter f3 = newFace();
    FaceIter f4 = newFace();


    // BEGIN CHANGING ASSIGNMENTS
    // define vertex
    v5->position = ( v1->position+v2->position ) / 2;

    // set edges
    h1->edge() = e1;
    h8->edge() = e1;
    e1->halfedge() = h1;
    h2->edge() = e2;
    h7->edge() = e2;
    e2->halfedge() = h2;
    h9->edge() = e3;
    h11->edge() = e3;
    e3->halfedge() = h11;
    h10->edge() = e4;
    h12->edge() = e4;
    e4->halfedge() = h12;

    // set twins
    h1->twin() = h8;
    h8->twin() = h1;
    h2->twin() = h7;
    h7->twin() = h2;
    h9->twin() = h11;
    h11->twin() = h9;
    h10->twin() = h12;
    h12->twin() = h10;

    // set next
    h1->next() = h9;
    h9->next() = h5;
    h2->next() = h10;
    h10->next() = h6;
    h3->next() = h11;
    h11->next() = h7;
    h7->next() = h3;
    h4->next() = h12;
    h12->next() = h8;
    h8->next() = h4;

    // set vertices
    h7->vertex() = v5;
    h8->vertex() = v5;
    h9->vertex() = v5;
    h10->vertex() = v5;
    v5->halfedge() = h7;
    h11->vertex() = v3;
    h12->vertex() = v4;

    // set faces
    h9->face() = f1;
    f1->halfedge() = h1;
    h10->face() = f2;
    f2->halfedge() = h2;
    h7->face() = f3;
    h3->face() = f3;
    h11->face() = f3;
    f3->halfedge() = h7;
    h8->face() = f4;
    h4->face() = f4;
    h12->face() = f4;
    f4->halfedge() = h8;

    return v5;
  }

  int MeshResampler::hash_position(Vector3D pos, double r, int grid_width, int grid_height) {
    // Hash a 3D position into a unique float identifier that represents
    // membership in some uniquely identified 3D box volume.
    Vector3D box((pos.x - fmod(pos.x, 2*r)) / 2*r, (pos.y - fmod(pos.y, 2*r)) / 2*r, (pos.z - fmod(pos.z, 2*r)) / 2*r);
    return floor(box.x + box.y * grid_width + box.z * grid_width * grid_height);
  }

  void MeshResampler::ballPivot(HalfedgeMesh& mesh) {
    // The radius of the ball we will pivot
    double r = 1.0;

    // A vertex of all vertices in the point cloud
    vector<vertex_struct> vertices;

    // Get the vertex normals and the dimensions of the voxel grid
    Vector3D maxDimensions(-INFINITY, -INFINITY, -INFINITY), minDimensions(INFINITY, INFINITY, INFINITY);
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->norm = v->normal();
      vertex_struct v_struct;
      v_struct.v = v;
      vertices.push_back(v_struct);
      for (int i = 0; i < 3; i++) {
        maxDimensions[i] = max(v->position[i], maxDimensions[i]);
        minDimensions[i] = min(v->position[i], minDimensions[i]);
      }
    }

    Vector3D dimensions = maxDimensions - minDimensions;
    for (int i = 0; i < 3; i++) {
      dimensions[i] = ceil(dimensions[i]/(2*r));
    }

    // Create the voxel grid
    vector<vector <vertex_struct> > voxels;
    for (int x = 0; x < dimensions[0]; x++) {
      for (int y = 0; y < dimensions[1]; y++) {
        for (int z = 0; z < dimensions[2]; z++) {
          vector<vertex_struct> voxel;
          voxels.push_back(voxel);
        }
      }
    }

    // Add vertices to voxels
    for (vertex_struct v_struct : vertices) {
      float cell = hash_position(v_struct.v->position, r, dimensions[0], dimensions[1]);
      voxels[cell].push_back(v_struct);
    }
    
    // Delete all objects in the mesh
    mesh.halfedges.clear();
    mesh.edges.clear();
    mesh.boundaries.clear();
    mesh.faces.clear();
    mesh.vertices.clear();

    // Initialize the front
    list<edge_struct> front;
    Vector3D center;

    // The main ball-pivot algorithm
    while (true) {
      while (!front.empty()) {
        // Get the vertices of the first edge on the front
        edge_struct e_ij_struct = front.front();
        EdgeIter e_ij = e_ij_struct.e;
        VertexIter vi = e_ij->halfedge()->vertex();
        VertexIter vj = e_ij->halfedge()->twin()->vertex();
        Vector3D mid = 0.5*(vi->position + vj->position);

        // Get all points vk such that a ball of radius r touches vi, vj, and vk
        // and the centers of those balls
        vector<Vector3D> possibleCenters;
        vector<vertex_struct> possibleVectors;
        int voxelNumber = hash_position(mid, r, dimensions[0], dimensions[1]);
        for (int x = voxelNumber - 1; x < voxelNumber + 2; x++) {
          for (int y = voxelNumber - 1; y < voxelNumber + 2; y++) {
            for (int z = voxelNumber - 1; z < voxelNumber + 2; z++) {
              for (vertex_struct vk_struct : voxels[x + y*dimensions[0] + z*dimensions[0]*dimensions[1]]) {
                VertexIter vk = vk_struct.v;
                Vector3D e_ik = vi->position - vk->position;
                Vector3D e_jk = vj->position - vk->position;
                float cosTheta = dot(e_ik, e_jk)/(e_ik.norm() * e_jk.norm());
                if ((vk->position - mid).norm() <= 2*r && abs(cosTheta) != 1) {
                  possibleVectors.push_back(vk_struct);
                  for (Vector3D center: get_centers(vi->position, vj->position, vk->position, r)) {
                    possibleCenters.push_back(center);
                  }
                }
              }
            }
          }
        }

        // Get the first center along the trajectory of the rotation
        float minTheta = 2*M_PI;
        vertex_struct vk_struct;
        Vector3D e_cm = center - mid;
        for (int i = 0; i < possibleVectors.size(); i++) {
          for (int j = 0; j < 2; j++) {
            VertexIter vk = possibleVectors[i].v;
            Vector3D vk_pos = vk->position;
            Vector3D e_ck = center - vk_pos;
            Vector3D n = cross(vi->position - vk_pos, vj->position - vk_pos);
            float theta;
            if (dot(n, e_cm) < 0) {
              theta = acos(dot(e_cm, e_ck)/(e_cm.norm() * e_ck.norm()));
            } else {
              theta = M_PI + acos(dot(e_cm, e_ck)/(e_cm.norm() * e_ck.norm()));
            }
            if (theta < minTheta) {
              minTheta = theta;
              vk_struct = possibleVectors[i];
            }
          }
        }

        VertexIter vk = vk_struct.v;
        // Add elements to the mesh
        if (!vk_struct.used) {
          front.remove(e_ij_struct);
          vk_struct.used = true;
          join(mesh, front, vi, vj, vk_struct.v, e_ij);
        } else {
          bool in_front = false;
          for (edge_struct e_struct: front) {
            if (e_struct.v1 == vk || e_struct.v2 == vk) {
              in_front = true;
              glue(mesh, front, vi, vj, vk_struct.v, e_ij);
            }
          }
          if (!in_front) {
            front.remove(e_ij_struct);
          }
        }
      }

      center = find_seed_triangle(mesh, voxels, vertices, front, r);
      break;
    }
  }

  vector<Vector3D> MeshResampler::get_centers(Vector3D a, Vector3D b, Vector3D c, double r) {
    // https://en.wikipedia.org/wiki/Circumscribed_circle
    // Returns the possible centers of a sphere of radius r that touches the three given points
    Vector3D e_ac = a - c;
    Vector3D e_bc = b - c;
    Vector3D n = cross(e_ac, e_bc);
    Vector3D p0 = (cross(dot(e_ac, e_ac) * e_bc - dot(e_bc, e_bc) * e_ac, n) / 2 * dot(n, n)) + c;
    float r0 = (e_ac.norm() * e_bc.norm() * (e_ac - e_bc).norm())/(2*n.norm());
    double h = sqrt(r*r - r0*r0);
    Vector3D c1 = p0 + n * h;
    Vector3D c2 = p0 - n * h;
    vector<Vector3D> centers;
    centers.push_back(c1);
    centers.push_back(c2);
    return centers;
  }

  void MeshResampler::join(HalfedgeMesh& mesh, list<edge_struct> front, VertexIter vi, VertexIter vj, 
    VertexIter vk, EdgeIter e_ij) {
    // Edge e has vertices vi and vj, and we want to add triangle (vi, vk, vj) to the mesh
    // and add edges (vi, vk) and (vk, vj) and remove edge (vi, vj) from the front. We return the
    // newly added edges for use in other functions.
    HalfedgeIter h_ij = mesh.newHalfedge();
    HalfedgeIter h_ik = mesh.newHalfedge();
    HalfedgeIter h_jk = mesh.newHalfedge();
    EdgeIter e_ik = mesh.newEdge();
    EdgeIter e_jk = mesh.newEdge();
    FaceIter f_ijk = mesh.newFace();

    vk->halfedge() = h_ik;

    h_ij->edge() = e_ij;
    h_ij->twin() = e_ij->halfedge();
    h_ik->edge() = e_ik;
    h_jk->edge() = e_jk;
    h_ij->face() = f_ijk;
    h_ik->face() = f_ijk;
    h_jk->face() = f_ijk;
    if (e_ij->halfedge()->vertex() == vi) {
      h_ij->next() = h_ik;
      h_ij->vertex() = vj;

      h_ik->next() = h_jk;
      h_ik->vertex() = vi;

      h_jk->next() = h_ij;
      h_jk->vertex() = vk;
    } else {
      h_ij->next() = h_jk;
      h_ij->vertex() = vi;

      h_ik->next() = h_ij;
      h_ik->vertex() = vk;

      h_jk->next() = h_ik;
      h_jk->vertex() = vj;
    }
    e_ik->halfedge() = h_ik;
    e_jk->halfedge() = h_jk;
    f_ijk->halfedge() = h_ij;

    edge_struct e_ik_struct, e_jk_struct;
    e_ik_struct.e = e_ik;
    e_jk_struct.e = e_jk;
    e_ik_struct.v1 = vi;
    e_ik_struct.v2 = vk;
    e_jk_struct.v1 = vj;
    e_jk_struct.v2 = vk;

    front.push_back(e_jk_struct);
    front.push_back(e_ik_struct);
  }

  void MeshResampler::glue(HalfedgeMesh& mesh, list<edge_struct> front, VertexIter vi, VertexIter vj, 
    VertexIter vk, EdgeIter e_ij) {
    bool e_ik_bool = false;
    bool e_jk_bool = false;
    EdgeIter e_ik;
    EdgeIter e_jk;
    for (edge_struct e_struct: front) {
      if ((e_struct.v1 == vi && e_struct.v2 == vk) || (e_struct.v1 == vk && e_struct.v2 == vi)) {
        e_ik_bool = true;
        front.remove(e_struct);
      } else if ((e_struct.v1 == vj && e_struct.v2 == vk) || (e_struct.v1 == vk && e_struct.v2 == vj)) {
        e_jk_bool = true;
        front.remove(e_struct);
      }
    }

    HalfedgeIter h_ij = mesh.newHalfedge();
    HalfedgeIter h_ik = mesh.newHalfedge();
    HalfedgeIter h_jk = mesh.newHalfedge();
    FaceIter f_ijk = mesh.newFace();
    h_ij->edge() = e_ij;
    h_ij->twin() = e_ij->halfedge();
    h_ij->face() = f_ijk;
    h_ik->face() = f_ijk;
    h_jk->face() = f_ijk;

    if (!e_ik_bool) {
      e_ik = mesh.newEdge();
      e_ik->halfedge() = h_ik;
      edge_struct e_ik_struct;
      e_ik_struct.e = e_ik;
      e_ik_struct.v1 = vi;
      e_ik_struct.v2 = vk;
      front.push_back(e_ik_struct);
    }
    if (!e_jk_bool) {
      e_jk = mesh.newEdge();
      e_jk->halfedge() = h_jk;
      edge_struct e_jk_struct;
      e_jk_struct.e = e_jk;
      e_jk_struct.v1 = vj;
      e_jk_struct.v2 = vk;
      front.push_back(e_jk_struct);
    }
    if (e_ij->halfedge()->vertex() == vi) {
      h_ij->next() = h_ik;
      h_ij->vertex() = vj;

      h_ik->next() = h_jk;
      h_ik->vertex() = vi;

      h_jk->next() = h_ij;
      h_jk->vertex() = vk;
    } else {
      h_ij->next() = h_jk;
      h_ij->vertex() = vi;

      h_ik->next() = h_ij;
      h_ik->vertex() = vk;

      h_jk->next() = h_ik;
      h_jk->vertex() = vj;
    }
  }

  Vector3D MeshResampler::find_seed_triangle(HalfedgeMesh& mesh, vector<vector <vertex_struct> > voxels, vector<vertex_struct> vertices, list<edge_struct> front, double r) {
    // Given a list of unused vertices, finds two more vertices such that a sphere
    // of radius r touches all three vertices and contains no other vertices
    // TODO: Speed up with voxels
    vector<VertexIter> seed;
    for (int k = 0; k < vertices.size(); k++) {
      vertex_struct vk_struct = vertices[k];
      VertexIter vk = vk_struct.v;
      if (!vk_struct.used) {
        for (int i = k; i < vertices.size(); i++) {
          vertex_struct vi_struct = vertices[i];
          VertexIter vi = vi_struct.v;
          if ((vi->position - vk->position).norm() <= 2*r) {
            for (int j = i; j < vertices.size(); j++) {
              vertex_struct vj_struct = vertices[j];
              VertexIter vj = vj_struct.v;
              if ((vj->position - vk->position).norm() <= 2*r) {
                Vector3D pi = vi->position;
                Vector3D pj = vj->position;
                Vector3D pk = vk->position;
                Vector3D e_ik = pi - pk;
                Vector3D e_jk = pj - pk;
                Vector3D n = cross(e_ik, e_jk);
                // Check that the normals of the three vertices are consistent with the normal of the 
                // triangle they form
                if ((dot(n, vi->norm) <= 0 && dot(n, vj->norm) <= 0 && dot(n, vk->norm) <= 0) || (dot(n, vi->norm) >= 0 && dot(n, vj->norm) >= 0 && dot(n, vk->norm) >= 0)) {
                  vector<Vector3D> centers = get_centers(pi, pj, pk, r);

                  // Check that the spheres touching the three points are valid, i.e. contain no other data point
                  bool good1 = true;
                  bool good2 = true;
                  for (vertex_struct u_struct : vertices) {
                    VertexIter u = u_struct.v;
                    if (u != vi && u != vj && u != vk && (u->position - centers[0]).norm() <= r) {
                      good1 = false;
                      continue;
                    }
                    if (u != vi && u != vj && u != vk && (u->position - centers[1]).norm() <= r) {
                      good2 = false;
                      continue;
                    }
                  }
                  if (good1 || good2) {
                    VertexIter new_vk = mesh.newVertex();
                    new_vk = vk;
                    if (!vi_struct.used) {
                      VertexIter new_vi = mesh.newVertex();
                      new_vi = vi;
                    }
                    if (!vj_struct.used) {
                      VertexIter new_vj = mesh.newVertex();
                      new_vj = vj;
                    }
                    HalfedgeIter h_ij = mesh.newHalfedge();
                    HalfedgeIter h_ik = mesh.newHalfedge();
                    HalfedgeIter h_jk = mesh.newHalfedge();
                    EdgeIter e_ij = mesh.newEdge();
                    EdgeIter e_ik = mesh.newEdge();
                    EdgeIter e_jk = mesh.newEdge();
                    FaceIter f_ijk = mesh.newFace();

                    vk->halfedge() = h_ik;

                    h_ij->edge() = e_ij;
                    h_ij->twin() = e_ij->halfedge();
                    h_ik->edge() = e_ik;
                    h_jk->edge() = e_jk;
                    h_ij->face() = f_ijk;
                    h_ik->face() = f_ijk;
                    h_jk->face() = f_ijk;
                    if (e_ij->halfedge()->vertex() == vi) {
                      h_ij->next() = h_ik;
                      h_ij->vertex() = vj;

                      h_ik->next() = h_jk;
                      h_ik->vertex() = vi;

                      h_jk->next() = h_ij;
                      h_jk->vertex() = vk;
                    } else {
                      h_ij->next() = h_jk;
                      h_ij->vertex() = vi;

                      h_ik->next() = h_ij;
                      h_ik->vertex() = vk;

                      h_jk->next() = h_ik;
                      h_jk->vertex() = vj;
                    }
                    e_ij->halfedge() = h_ij;
                    e_ik->halfedge() = h_ik;
                    e_jk->halfedge() = h_jk;
                    f_ijk->halfedge() = h_ij;

                    edge_struct e_ij_struct;
                    e_ij_struct.e = e_ij;
                    e_ij_struct.v1 = vi;
                    e_ij_struct.v2 = vj;

                    edge_struct e_ik_struct;
                    e_ik_struct.e = e_ik;
                    e_ik_struct.v1 = vi;
                    e_ik_struct.v2 = vk;

                    edge_struct e_jk_struct;
                    e_jk_struct.e = e_jk;
                    e_jk_struct.v1 = vj;
                    e_jk_struct.v2 = vk;

                    front.push_back(e_ij_struct);
                    front.push_back(e_jk_struct);
                    front.push_back(e_ik_struct);
                  }
                  if (good1) {
                    return centers[0];
                  } else if (good2) {
                    return centers[1];
                  }
                }
              }
            }
          }
        }
      }
    }
    return NULL;
  }

  /*void MeshResampler::glue(HalfedgeMesh& mesh, list<EdgeIter> front, VertexIter vk,
    EdgeIter e_ij, VertexIter vi, VertexIter vj, list<VertexIter> unused) {
    // Edge e has vertices vi and vj.
    for (Edge edge : front) {
      // If vk is part of some edge in the front, we join. If either of the joined edges
      // is coincident, i.e. connects the same two vertices but with opposite direction,
      // to an existing edge in the front, we remove both edges from the front.
      if (vk in edge) {
        list<Edge> joined = join(front, vk, e, vertices);
        for (Edge j : joined) {
          if (coincident of j in front) {
            front.remove(j);
            front.remove(coincident);
          }
        }
      } else {
        // If vk is part of the mesh but not in the front, we cannot create triangle (vi, vk, vj) 
        // since it would be non-manifold, so we know e is a boundary edge.
        mark e as boundary edge;
      }
    }
  }*/

    /*// This routine now runs the Ball Pivot Algorithm, starting with a complete mesh
    // TODO: Implement voxel grid to make vertex traversal reasonable
    // Strip all features of the mesh except for vertices

    // Initialize the front, which is a linked list of edges that have been added
    // to the mesh and might still be used as a pivot edge
    double r = 1.0;
    list<Edge> front;
    list<Vertex> vertices = mesh.vertices;
    list<Vertex> used;
    list<Vertex> seed;
    Vector3D center;

    // We loop until we are no longer able to pivot or find a seed triangle
    while (true) {
      Vertex v0 = seed[0];
      Vertex v1 = seed[1];
      Vertex v2 = seed[2];

      // We find an edge from the front to pivot around
      while (!front.empty()) {
        Vector3D m = 0.5*(v0.position + v1.position);
        double R = (center - m).norm();
        for (Vertex v : vertices) {
          // We find the center of a sphere that touches the vertices of the pivot edge
          // and another vertex
          // TODO: Take the FIRST center along the circular rotation trajectory
          if ((v.position - m).norm() <= 2*r) {
            (c1, c2) = get_center(v0.position, v1.position, v.position);
            // If v is not part of any triangle, we call join
            // If v is part of the mesh, we call glue
            if (!(v in used)) {
              join(front, v, edge(v0, v1), vertices);
            } else {
              glue(front, v, edge(v0, v1), vertices);
            }
          }
        }
      }
      
      // If there is no active edge on the front, we find a new seed triangle and
      // add that triangle to our mesh
      pair<list<Vertex>, Vector3D> seed_pair = seed_triangle(vertices);
      seed = seed_pair.first;
      center = seed_pair.second;
      HalfedgeIter h0 = newHalfedge();
      HalfedgeIter h1 = newHalfedge();
      HalfedgeIter h2 = newHalfedge();
      EdgeIter e0 = newEdge();
      EdgeIter e1 = newEdge();
      EdgeIter e2 = newEdge();
      FaceIter f0 = newFace();
      //TODO: set halfedge mesh features
      front.push_back(e0);
      front.push_back(e1);
      front.push_back(e2);
      vertices.remove(v0);
      vertices.remove(v1);
      vertices.remove(v2);
    }
  }

  pair<list<Vertex>, Vector3D> MeshResampler::seed_triangle(list<Vertex> vertices) {
    // Give a list of unused vertices, finds two more vertices such that a sphere
    // of radius r touches all three vertices and contains no other vertices
    double r = 1.0;
    list<Vertex> seed;
    for (Vertex v : vertices) {
      for (Vertex v1 : vertices) {
        if ((v1.position - v.position).norm() <= 2*r && !(&v1 == &v)) {
          for (Vertex v2 : vertices) {
            if ((v2.position - v.position).norm() <= 2*r && !(&v2 == &v) && !(&v2 == &v1)) {
              Vector3D p1 = v.position;
              Vector3D p2 = v1.position;
              Vector3D p3 = v2.position;
              Vector3D p21 = p2-p1;
              Vector3D p31 = p3-p1;
              Vector3D n = cross(p21, p31);
              // Check that the normals of the three vertices are consistent with the normal of the 
              // triangle they form
              if (!(dot(n, v.normal()) <= 0 && dot(n, v1.normal()) <= 0 && dot(n, v.normal()) <= 0)
                || !(dot(n, v.normal()) >= 0 && dot(n, v1.normal()) >= 0 && dot(n, v.normal()) >= 0)) {
                (c1, c2) = get_center(p1, p2, p3);

                // Check that the spheres touching the three points are valid, i.e. contain no other data point
                bool good1 = true;
                bool good2 = true;
                for (Vertex u : vertices) {
                  if (!(&u == &v) && !(&u == &v1) && !(&u == &v2) && (u.position - c1).norm() <= r) {
                    good1 = false;
                    continue;
                  }
                  if (!(&u == &v) && !(&u == &v1) && !(&u == &v2) && (u.position - c2).norm() <= r) {
                    good2 = false;
                    continue;
                  }
                }
                if (good1) {
                  seed.push_back(v);
                  seed.push_back(v1);
                  seed.push_back(v2);
                  return make_pair(seed, c1);
                } else if (good2) {
                  seed.push_back(v);
                  seed.push_back(v1);
                  seed.push_back(v2);
                  return make_pair(seed, c2);
                }
              }
            }
          }
        }
      }
    }
    return make_pair(NULL, NULL);
  }
    
  void MeshResampler::glue(list<Edge> front, Vertex vk, Edge e, list<Vertex> vertices) {
    // Edge e has vertices vi and vj.
    for (Edge edge : front) {
      // If vk is part of some edge in the front, we join. If either of the joined edges
      // is coincident, i.e. connects the same two vertices but with opposite direction,
      // to an existing edge in the front, we remove both edges from the front.
      if (vk in edge) {
        list<Edge> joined = join(front, vk, e, vertices);
        for (Edge j : joined) {
          if (coincident of j in front) {
            front.remove(j);
            front.remove(coincident);
          }
        }
      } else {
        // If vk is part of the mesh but not in the front, we cannot create triangle (vi, vk, vj) 
        // since it would be non-manifold, so we know e is a boundary edge.
        mark e as boundary edge;
      }
    }
  }*/

  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;

      int d = 0; // degree
      Vector3D sum_neighbor_pos = Vector3D(0, 0, 0);
      HalfedgeIter h = v->halfedge();
      do {
        d++;
        sum_neighbor_pos += h->next()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      double u = 3.0 / (8.0*d);
      v->newPosition = (1-d*u) * v->position + u * sum_neighbor_pos;
    }


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;

      HalfedgeIter h1 = e->halfedge();
      HalfedgeIter h2 = h1->twin();

      Vector3D A = h1->vertex()->position;
      Vector3D B = h2->vertex()->position;
      Vector3D C = h1->next()->next()->vertex()->position;
      Vector3D D = h2->next()->next()->vertex()->position;

      e->newPosition = (3.0/8.0) * (A+B) + (1.0/8.0) * (C+D);
    }


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      HalfedgeIter h1 = e->halfedge();
      HalfedgeIter h2 = h1->twin();
      bool v1_new = h1->vertex()->isNew;
      bool v2_new = h2->vertex()->isNew;

      if (!v1_new && !v2_new) {
        VertexIter v = mesh.splitEdge(e);
        v->isNew = true;

        v->newPosition = e->newPosition;

        HalfedgeIter h = v->halfedge();
        h->edge()->isNew = false;
        h = h->twin()->next();
        h->edge()->isNew = true;
        h = h->twin()->next();
        h->edge()->isNew = false;
        h = h->twin()->next();
        h->edge()->isNew = true;
      }
    }


    // TODO Now flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew) {
        HalfedgeIter h1 = e->halfedge();
        HalfedgeIter h2 = h1->twin();

        bool v1_new = h1->vertex()->isNew;
        bool v2_new = h2->vertex()->isNew;

        if ((v1_new && !v2_new) || (!v1_new && v2_new)) {
          mesh.flipEdge(e);
        }
      }
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
    }
  }
}
