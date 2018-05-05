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
    v5->norm = (v1->norm + v2->norm + v3->norm + v4->norm)/4.0;

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

  int MeshResampler::hash_position(Vector3D pos, double r, Vector3D dimensions, Vector3D minDimensions) {
    // Hash a 3D position into a unique int identifier that represents
    // membership in some uniquely identified 3D box volume.
    pos = pos - minDimensions;
    Vector3D box = Vector3D((pos.x - fmod(pos.x, 2*r)) / (2.0 * r), (pos.y - fmod(pos.y, 2*r)) / (2.0 * r), (pos.z - fmod(pos.z, 2.0 * r)) / (2.0 * r));
    return floor(box.x + box.y * dimensions[0] + box.z * dimensions[0] * dimensions[1]);
  }

  void MeshResampler::ballPivot(HalfedgeMesh& mesh, double BPAr) {
    double r = BPAr; // The radius of the ball we will pivot
    double delta = pow(10, -6); // An error for calculating the angle
    vector<vertex_struct *> vertices; // A vector of vertices in the point cloud
    Vector3D maxDimensions(-INFINITY, -INFINITY, -INFINITY), minDimensions(INFINITY, INFINITY, INFINITY);
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      if (v->norm[0] == INFINITY) {
        v->norm = v->normal(); // Set vertex normals so we don't need faces to calculate them
      }
      vertex_struct *v_struct = new vertex_struct(); // Instantiate a vertex struct for each vertex
      v_struct->v = v;
      vertices.push_back(v_struct);
      for (int i = 0; i < 3; i++) { // Find the dimensions of the smallest box containing our entire point cloud
        maxDimensions[i] = max(v->position[i], maxDimensions[i]);
        minDimensions[i] = min(v->position[i], minDimensions[i]);
      }
    }

    Vector3D dimensions = maxDimensions - minDimensions; // Get length of each dimension  
    for (int i = 0; i < 3; i++) {
      dimensions[i] = ceil(dimensions[i]/(2*r)) + 1; // Set dimensions to be the number of cells in each dimension
    }
    vector<vector <vertex_struct *> > voxels(dimensions[0] * dimensions[1] * dimensions[2]); // Initialize the voxel grid
    for (vertex_struct *v_struct : vertices) {
      int cell = hash_position(v_struct->v->position, r, dimensions, minDimensions - Vector3D(2*r, 2*r, 2*r));
      voxels[cell].push_back(v_struct); // Add vertices to correct voxels
    }

    // Delete all objects in the mesh
    mesh.halfedges.clear();
    mesh.edges.clear();
    mesh.boundaries.clear();
    mesh.faces.clear();
    //mesh.vertices.clear();

    // Initialize the front
    list<edge_struct *> front;
    // The main ball-pivot algorithm
    while (true) {
      while (!front.empty()) {
        // Get the vertices of the first edge on the front and its opposite vertex
        // Since the edge is on the front, it must be connected to exactly one triangle
        edge_struct* e_ij_struct = front.front();
        EdgeIter e_ij = e_ij_struct->e;
        VertexIter vi = e_ij_struct->v1;
        VertexIter vj = e_ij_struct->v2;
        VertexIter last_vk = e_ij_struct->v3;
        // The positions of the vertices associated with this edge
        Vector3D vi_pos = vi->position;
        Vector3D vj_pos = vj->position;
        Vector3D last_vk_pos = last_vk->position;
        // The midpoint of the edge we will rotate around
        Vector3D mid = 0.5 * (vi_pos + vj_pos);
        // The normal of the triangle calculated from the cross product of vk -> vi and vk -> vj)
        Vector3D n = cross(vi_pos - last_vk_pos, vj_pos - last_vk_pos);
        // The "actual" face normal, calculated from the normals at the vertices
        Vector3D old_face_normal = (vi->norm + vj->norm + last_vk->norm)/3.0;
        // The center of the sphere touching our three points wit hthe given normal
        Vector3D center = get_center(vi_pos, vj_pos, last_vk_pos, r, old_face_normal);

        // Vertices that we might reach by rotating on the current edge
        vector<vertex_struct *> possibleVertices;
        // The center of each sphere touching vi, vj, and a possible vertex
        vector<Vector3D> possibleCenters;
        // The voxel containing the center of the trajectory, mid
        int voxelNumber = hash_position(mid, r, dimensions, minDimensions - Vector3D(2*r, 2*r, 2*r));
        for (int x = -1; x < 2; x++) {
          for (int y = -1; y < 2; y++) {
            for (int z = -1; z < 2; z++) {
              int newVoxelNumber = voxelNumber + x + y * dimensions[0] + z * dimensions[0] * dimensions[1];
              if (newVoxelNumber >= 0 && newVoxelNumber < voxels.size()) {
                for (vertex_struct *vk_struct : voxels[newVoxelNumber]) {
                  // A potential vertex vk
                  VertexIter vk = vk_struct->v;
                  Vector3D vk_pos = vk->position;
                  if (vk != vi && vk != vj && vk != last_vk) {
                    Vector3D e_ik = vi_pos - vk_pos;
                    Vector3D e_jk = vj_pos - vk_pos;
                    // The normal we would get if we rotated and resized the seed triangle
                    Vector3D rotate_face_normal = cross(vi_pos - vk_pos, vj_pos - vk_pos);
                    // The "actual" new face normal, calculated from the normals at the vertices
                    Vector3D new_face_normal = (vi->norm + vj->norm + vk->norm)/3.0;
                    // We need to make sure our new vertices will not create a non-orientable manifold.
                    // If old_face_normal and n were in the same direction, we need rotate_face_normal and
                    // new_face_normal to be in opposite directions, and vice versa
                    if ((dot(old_face_normal, n) <= 0 && dot(rotate_face_normal, new_face_normal) >= 0)
                      || (dot(old_face_normal, n) >= 0 && dot(rotate_face_normal, new_face_normal) <= 0)) {
                      Vector3D center = get_center(vi_pos, vj_pos, vk_pos, r, new_face_normal);
                      if (center[0] != INFINITY) {
                        possibleVertices.push_back(vk_struct);
                        possibleCenters.push_back(center);
                      }
                    }
                  }
                }
              }
            }
          }
        }

        // The edge will either form a second and final triangle or be unable to form any triangles in the mesh,
        // and so we can remove it from the front.
        front.remove(e_ij_struct);
        // If vi and vj can only form a triangle with last_vk, we break the loop early and continue in the front.
        if (possibleVertices.size() == 0) {
          continue;
        } 

        // Get the first center along the trajectory of the rotation
        // All rotations will be in the range [0, 2*M_PI)
        double minTheta = 2.0 * M_PI;
        // The indices of the first center on the trajectory and its corresponding vk
        int index = -1;
        // A vector pointing from mid, the center of the trajectory, to center, which is on the trajectory
        Vector3D e_cm = center - mid;
        // Calculations to find the circumcenter of the circle touching vi, vj, and last_vk
        Vector3D old_e_ac = vi_pos - last_vk_pos;
        Vector3D old_e_bc = vj_pos - last_vk_pos;
        Vector3D old_n = cross(old_e_ac, old_e_bc);
        Vector3D old_p0 = (cross(dot(old_e_ac, old_e_ac) * old_e_bc - dot(old_e_bc, old_e_bc) * old_e_ac, old_n) / (2 * dot(old_n, old_n))) + last_vk->position;
        for (int i = 0; i < possibleVertices.size(); i++) {
          double theta;
          // The center we are testing, and its corresponding vertex
          Vector3D possible_center = possibleCenters[i];
          VertexIter possible_vk = possibleVertices[i]->v;

          // A vector pointing from mid, the center of the trajectory, to possible_center, which is on the trajectory
          Vector3D e_nm = possible_center - mid;
          // The angle between e_cm and e_nm
          theta = acos(dot(e_cm, e_nm)/(e_cm.norm() * e_nm.norm()));
          // We use delta to account for math errors.
          if (theta > delta && ((dot(last_vk_pos - mid, old_p0 - mid) >= 0 && dot(cross(e_cm, e_nm), cross(e_cm, old_p0 - mid)) >= 0)
            || (dot(last_vk_pos - mid, old_p0 - mid) <= 0 && dot(cross(e_cm, e_nm), cross(e_cm, old_p0 - mid)) <= 0))) {
            theta = 2 * M_PI - theta;
          }
          if (theta <= minTheta) {
            minTheta = theta;
            index = i;
          }
        }

        vertex_struct *vk_struct = possibleVertices[index];
        VertexIter vk = vk_struct->v;
        Vector3D possibleCenter = possibleCenters[index];
        if (!vk_struct->used) { // If vk is not already par of the mesh
          vk_struct->used = true;
          join(mesh, front, vi, vj, vk, e_ij);
        } else {
          for (edge_struct *e_struct: front) {
            if (e_struct->v1 == vk || e_struct->v2 == vk) { // If vk is connected to some edge on the front
              glue(mesh, front, vi, vj, vk, e_ij);
              break;
            }            
          }
        }
      }

      bool found_seed = find_seed_triangle(mesh, voxels, vertices, front, dimensions, minDimensions, r);
      if (!found_seed) {
        break;
      }
    }
    for (vertex_struct *v_struct : vertices) {
      if (!v_struct->used) {
        mesh.deleteVertex(v_struct->v);
      }
    }
    return;
  }

  bool MeshResampler::find_seed_triangle(HalfedgeMesh& mesh, vector<vector <vertex_struct *> >& voxels, vector<vertex_struct *> vertices, 
    list<edge_struct *>& front, Vector3D dimensions, Vector3D minDimensions, double r) {
    // Given a list of unused vertices, finds two more vertices such that a sphere
    // of radius r touches all three vertices and contains no other vertices
    for (int v = 0; v < voxels.size(); v++) { // Iterate through every voxel
      vector<vertex_struct *> vox = voxels[v];
      bool vox_used = false;
      for (vertex_struct* v: vox) { // See if any point in the voxel has been used
        if (v->used) {
          vox_used = true;
          break;
        }
      }
      if (!vox_used) { // If no vertex in this voxel has been used, we look for a seed triange
        for (int k = 0; k < vox.size(); k++) { // We look at every vertex in the voxel
          vertex_struct* vk_struct = vox[k];
          VertexIter vk = vk_struct->v; // A point that is not yet part of the mesh
          //vk_struct->used = true;
          vector<vertex_struct *> possibleVertices; // Vertices within 2r of our seed vertex vk
          for (int x = -1; x < 2; x++) { // We look at the surrounding voxels for potential seed points
            for (int y = -1; y < 2; y++) {
              for (int z = -1; z < 2; z++) {
                int newVoxelNumber = v + x + y * dimensions[0] + z * dimensions[0] * dimensions[1];
                if (newVoxelNumber >= 0 && newVoxelNumber < voxels.size()) {
                  for (vertex_struct *v_struct : voxels[newVoxelNumber]) {
                    if (v_struct->v != vk && (v_struct->v->position - vk->position).norm() <= 2.0 * r) {
                      possibleVertices.push_back(v_struct);
                    }
                  }
                }
              }
            }
          }
          for (int i = 0; i < possibleVertices.size(); i++) { // Potential vi's
            vertex_struct* vi_struct = possibleVertices[i];
            VertexIter vi = vi_struct->v;
            for (int j = i + 1; j < possibleVertices.size(); j++) { // Potential vj's
              vertex_struct* vj_struct = possibleVertices[j];
              VertexIter vj = vj_struct->v;
              // The positions of our potential seed vertices
              Vector3D pi = vi->position;
              Vector3D pj = vj->position;
              Vector3D pk = vk->position;
              if ((pi - pj).norm() <= 2.0 * r) { // If all vertices are within 2r of each other
                Vector3D e_ik = pi - pk;
                Vector3D e_jk = pj - pk;
                Vector3D n = (vi->norm + vj->norm + vk->norm)/3.0;
                // Check that the normals of the three vertices are consistent
                if ((dot(n, vi->norm) <= 0 && dot(n, vj->norm) <= 0 && dot(n, vk->norm) <= 0)
                  || (dot(n, vi->norm) >= 0 && dot(n, vj->norm) >= 0 && dot(n, vk->norm) >= 0)) {
                  // Ge the sphere on top of the triangle, assuming it is valid
                  Vector3D center = get_center(pi, pj, pk, r, n);
                  if (center[0] == INFINITY) {
                    continue;
                  }
                  // Check that the sphere does not contain any other data point
                  bool c_bool = true;
                  for (vertex_struct *u_struct : possibleVertices) {
                    VertexIter u = u_struct->v;
                    if (u != vi && u != vj && (u->position - center).norm() < r) {
                      c_bool = false;
                      break;
                    }
                  }
                  // If the sphere touches vi, vj and vk and contains no other data point, (vi, vj, vk) is a valid seed triangle
                  if (c_bool) {
                    // Set all vertices to used
                    vk_struct->used = true;
                    vi_struct->used = true;
                    vj_struct->used = true;

                    // Add elements to the mesh
                    HalfedgeIter h_ij = mesh.newHalfedge();
                    HalfedgeIter h_ik = mesh.newHalfedge();
                    HalfedgeIter h_jk = mesh.newHalfedge();
                    EdgeIter e_ij = mesh.newEdge();
                    EdgeIter e_ik = mesh.newEdge();
                    EdgeIter e_jk = mesh.newEdge();
                    FaceIter f_ijk = mesh.newFace();

                    vi->halfedge() = h_ij;
                    vj->halfedge() = h_jk;
                    vk->halfedge() = h_ik;

                    h_ij->edge() = e_ij;
                    h_ik->edge() = e_ik;
                    h_jk->edge() = e_jk;

                    h_ij->twin() = h_ij;
                    h_ik->twin() = h_ik;
                    h_jk->twin() = h_jk;

                    h_ij->face() = f_ijk;
                    h_ik->face() = f_ijk;
                    h_jk->face() = f_ijk;
                          
                    h_ij->next() = h_ik;
                    h_ij->vertex() = vj;

                    h_ik->next() = h_jk;
                    h_ik->vertex() = vi;

                    h_jk->next() = h_ij;
                    h_jk->vertex() = vk;

                    e_ij->halfedge() = h_ij;
                    e_ik->halfedge() = h_ik;
                    e_jk->halfedge() = h_jk;
                    f_ijk->halfedge() = h_ij;

                    edge_struct *e_ij_struct = new edge_struct();
                    e_ij_struct->e = e_ij;
                    e_ij_struct->v1 = vi;
                    e_ij_struct->v2 = vj;
                    e_ij_struct->v3 = vk;

                    edge_struct *e_ik_struct = new edge_struct();
                    e_ik_struct->e = e_ik;
                    e_ik_struct->v1 = vi;
                    e_ik_struct->v2 = vk;
                    e_ik_struct->v3 = vj;

                    edge_struct *e_jk_struct = new edge_struct();
                    e_jk_struct->e = e_jk;
                    e_jk_struct->v1 = vj;
                    e_jk_struct->v2 = vk;
                    e_jk_struct->v3 = vi;

                    front.push_front(e_ij_struct);
                    front.push_front(e_jk_struct);
                    front.push_front(e_ik_struct);
                    // Return that we could find a seed triangle
                    return true;
                  }
                }
              }
            }
          }
        }
      }
    }
    // Return that we could not find a seed triangle
    return false;
  }

  Vector3D MeshResampler::get_center(Vector3D a, Vector3D b, Vector3D c, double r, Vector3D norm) {
    // https://en.wikipedia.org/wiki/Circumscribed_circle
    // Returns the center of a sphere of radius r that touches the three given points and is in the direction of n
    Vector3D e_ac = a - c;
    Vector3D e_bc = b - c;
    // A unit vector perpendicular to the plane of the triangle
    Vector3D c_norm = cross(e_ac, e_bc);
    // The circumcenter of the circle touching all three points
    Vector3D p0 = (cross(dot(e_ac, e_ac) * e_bc - dot(e_bc, e_bc) * e_ac, c_norm) / (2.0 * dot(c_norm, c_norm))) + c;
    // The radius of the circumscribed circle
    double r0 = (e_ac.norm() * e_bc.norm() * (e_ac - e_bc).norm())/(2.0 * c_norm.norm());
    // Check that we can actually form a sphere of radius r
    if (c_norm.norm() == 0 || r < r0) {
      return Vector3D(INFINITY, INFINITY, INFINITY);
    }
    double h = sqrt(r*r - r0*r0);
    Vector3D c1 = p0 + c_norm * h/c_norm.norm();
    Vector3D c2 = p0 - c_norm * h/c_norm.norm();
    if (dot(c1 - p0, norm) >= 0) {
      return c1;
    } else {
      return c2;
    }
  }

  void MeshResampler::join(HalfedgeMesh& mesh, list<edge_struct *>& front, VertexIter vi, VertexIter vj, 
    VertexIter vk, EdgeIter e_ij) {
    // Edge e_ij has vertices vi and vj, and we want to add triangle (vi, vk, vj) and
    // edges (vi, vk) and (vk, vj) to the mesh.
    HalfedgeIter h_ij = mesh.newHalfedge();
    HalfedgeIter h_ik = mesh.newHalfedge();
    HalfedgeIter h_jk = mesh.newHalfedge();
    EdgeIter e_ik = mesh.newEdge();
    EdgeIter e_jk = mesh.newEdge();
    FaceIter f_ijk = mesh.newFace();

    vk->halfedge() = h_ik;

    h_ij->twin() = e_ij->halfedge();
    e_ij->halfedge()->twin() = h_ij;
    h_ik->twin() = h_ik;
    h_jk->twin() = h_jk;

    vk->halfedge() = h_ik;
    vi->halfedge() = h_ij;
    vj->halfedge() = h_jk;

    h_ij->edge() = e_ij;
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

    edge_struct *e_ik_struct = new edge_struct;
    e_ik_struct->e = e_ik;
    e_ik_struct->v1 = vi;
    e_ik_struct->v2 = vk;
    e_ik_struct->v3 = vj;

    edge_struct *e_jk_struct = new edge_struct;
    e_jk_struct->e = e_jk;
    e_jk_struct->v1 = vj;
    e_jk_struct->v2 = vk;
    e_jk_struct->v3 = vi;

    front.push_front(e_jk_struct);
    front.push_front(e_ik_struct);
  }

  void MeshResampler::glue(HalfedgeMesh& mesh, list<edge_struct *>& front, VertexIter vi, VertexIter vj, 
    VertexIter vk, EdgeIter e_ij) {
    // Edge e_ij has vertices vi and vj, and we want to add triangle (vi, vk, vj) and
    // potentially edges (vi, vk) and (vk, vj) to the mesh.
    bool e_ik_bool = false;
    bool e_jk_bool = false;

    edge_struct *e_ik_struct = new edge_struct();
    e_ik_struct->v1 = vi;
    e_ik_struct->v2 = vk;
    e_ik_struct->v3 = vj;

    edge_struct *e_jk_struct = new edge_struct();
    e_jk_struct->v1 = vj;
    e_jk_struct->v2 = vk;
    e_jk_struct->v3 = vi;

    // A list of edges to be removed from the front
    list<edge_struct *> removed;

    for (edge_struct *e_struct: front) {
      if ((e_struct->v1 == vi && e_struct->v2 == vk) || (e_struct->v1 == vk && e_struct->v2 == vi)) {
        e_ik_struct = e_struct;
        e_ik_bool = true;
        removed.push_back(e_ik_struct);
      } 
      if ((e_struct->v1 == vj && e_struct->v2 == vk) || (e_struct->v1 == vk && e_struct->v2 == vj)) {
        e_jk_struct = e_struct;
        e_jk_bool = true;
        removed.push_back(e_jk_struct);
      }
      if (e_ik_bool && e_jk_bool) {
        break;
      }
    }

    HalfedgeIter h_ij = mesh.newHalfedge();
    HalfedgeIter h_ik = mesh.newHalfedge();
    HalfedgeIter h_jk = mesh.newHalfedge();
    FaceIter f_ijk = mesh.newFace();

    h_ij->edge() = e_ij;
    h_ij->twin() = e_ij->halfedge();
    e_ij->halfedge()->twin() = h_ij;

    h_ij->face() = f_ijk;
    h_ik->face() = f_ijk;
    h_jk->face() = f_ijk;
    
    f_ijk->halfedge() = h_ij;

    if (!e_ik_bool) {
      EdgeIter e_ik = mesh.newEdge();
      e_ik->halfedge() = h_ik;
      e_ik_struct->e = e_ik;
      front.push_front(e_ik_struct);
      h_ik->twin() = h_ik;
      h_ik->edge() = e_ik;
    } else {
      EdgeIter e_ik = e_ik_struct->e;
      h_ik->twin() = e_ik->halfedge();
      e_ik->halfedge()->twin() = h_ik;
      h_ik->edge() = e_ik;
    }
    if (!e_jk_bool) {
      EdgeIter e_jk = mesh.newEdge();
      e_jk->halfedge() = h_jk;
      e_jk_struct->e = e_jk;
      front.push_front(e_jk_struct);
      h_jk->twin() = h_jk;
      h_jk->edge() = e_jk;
    } else {
      EdgeIter e_jk = e_jk_struct->e;
      h_jk->twin() = e_jk->halfedge();
      e_jk->halfedge()->twin() = h_jk;
      h_jk->edge() = e_jk;
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
    for (edge_struct *e_struct : removed) {
      front.remove(e_struct);
    }
  }

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
