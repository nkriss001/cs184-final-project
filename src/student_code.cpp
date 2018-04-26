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


    // This routine now runs the Ball Pivot Algorithm, starting with a complete mesh
    // TODO: Implement voxel grid to make vertex traversal reasonable
    // Strip all features of the mesh except for vertices
    mesh.halfedges.clear();
    mesh.edges.clear();
    mesh.boundaries.clear();
    mesh.faces.clear();

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

  list<Vector3D> MeshResampler::get_center(Vector3D p1, Vector3D p2, Vector3D p3) {
    // https://stackoverflow.com/questions/11719168/how-do-i-find-the-sphere-center-from-3-points-and-radius
    // Returns the possible centers of a sphere of radius r that touches the three given points
    double r = 1.0;
    Vector3D p21 = p2-p1;
    Vector3D p31 = p3-p1;
    Vector3D n = cross(p21, p31);
    Vector3D p0 = cross(dot(p21, p21) * p31 - dot(p31, p31) * p21, n) / 2 / dot(n, n) + p1;
    double t1 = sqrt((r*r - dot(p0 - p1, p0 - p1))/dot(n, n));
    double t2 = -sqrt((r*r - dot(p0 - p1, p0 - p1))/dot(n, n));
    Vector3D c1 = p0 + n * t1;
    Vector3D c2 = p0 + n * t2;
    return make_pair(c1, c2);
  }

  list<Edge> MeshResampler::join(list<Edge> front, Vertex vk, Edge e, list<Vertex> vertices) {
    // Edge e has vertices vi and vj, and we want to add triangle (vi, vk, vj) to the mesh
    // and add edges (vi, vk) and (vk, vj) and remove edge (vi, vj) from the front. We return the
    // newly added edges for use in other functions.
    HalfedgeIter h0 = newHalfedge();
    HalfedgeIter h1 = newHalfedge();
    HalfedgeIter h2 = newHalfedge();
    EdgeIter eik = newEdge();
    EdgeIter ekj = newEdge();
    EdgeIter eji = newEdge();
    FaceIter f0 = newFace();
    //TODO: set halfedge mesh features
    front.push_back(eik);
    front.push_back(ekj);
    front.push_back(eji);
    vertices.remove(vi);
    vertices.remove(vj);
    vertices.remove(vk);
    front.remove(e);
    front.push_back(ei);
    front.push_back(ek);
    return list<Edge>(ei, ek);
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
  }

    /*for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
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

  void MeshResampler::ballpivot(list<Vector3D*> vertices, list<Vector3D*> normals, double radius) {
    // TODO
      // - Set up voxel grid (project 4 hashMap, but as a three-dimensional array)
      // - If front is empty, use seed_triangle to get new seed triangle and triangle center
      // - 
      // - 

  }

  list<Vector3D*> MeshResampler::seed_triangle(list<Vector3D*> vertices, ) {
    // TODO:
      // - Pick a vertex that has not been used in triangulation (seed vertex)
      // - Build a priority queue of pairs in its neighborhood in order of distance from seed
      // - Test 1: For element in queue, determine if all normals point "outward" (at least in the same direction)
      // - Test 2: Is there a point c on the outward side for which the distance to each proposed point is exactly r
      // - If both tests pass, return seed triangle and seed point
  }

  // DATATYPES:
    // - Front: collection of linked lists of edges
    // - Edge: Includes endpoint 1, endpoint 2, opposite vertex, center of ball touching those three points, 
    //   links to next & previous edges, active/boundary/frozen*/
}
