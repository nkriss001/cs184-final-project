#include "CGL/CGL.h"

#include "collada.h"
#include "meshEdit.h"
#include "bezierPatch.h"
#include "bezierCurve.h"
#include "mergeVertices.h"
#include "shaderUtils.h"

#include <iostream>

using namespace std;
using namespace CGL;

#define msg(s) cerr << "[Collada Viewer] " << s << endl;


void loadPlyFile(const char * path, std::vector<Vector3D> &points);


int loadFile(MeshEdit* collada_viewer, const char* path) {

  Scene* scene = new Scene();

  std::string path_str = path;
  if (path_str.substr(path_str.length()-4, 4) == ".dae")
  {
    if (ColladaParser::load(path, scene) < 0) {
      delete scene;
      return -1;
    }
  }
  else if (path_str.substr(path_str.length()-4, 4) == ".bez")
  {
    Camera* cam = new Camera();
    cam->type = CAMERA;
    Node node;
    node.instance = cam;
    scene->nodes.push_back(node);
    Polymesh* mesh = new Polymesh();

    FILE* file = fopen(path, "r");
    int n = 0;
    fscanf(file, "%d", &n);
    for (int i = 0; i < n; i++)
    {
      BezierPatch patch;
      patch.loadControlPoints(file);
      patch.add2mesh(mesh);
      mergeVertices(mesh);
    }
    fclose(file);

    mesh->type = POLYMESH;
    node.instance = mesh;
    scene->nodes.push_back(node);
  }
  else if (path_str.substr(path_str.length()-4, 4) == ".ply")
  {
    Camera* cam = new Camera();
    cam->type = CAMERA;
    Node node;
    node.instance = cam;
    scene->nodes.push_back(node);

    std::vector<Vector3D> points;
    loadPlyFile(path, points);

    PointCloud* pc = new PointCloud();
    pc->type = POINTCLOUD;
    pc->points = points;

    node.instance = pc;
    scene->nodes.push_back(node);

    collada_viewer->pointCloudMode = true;
  }
  else
  {
    return -1;
  }

  collada_viewer->load( scene );

  GLuint tex = makeTex("envmap/envmap.png");
  if(!tex) tex = makeTex("../envmap/envmap.png");
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, tex);
  glActiveTexture(GL_TEXTURE2);

  return 0;
}

void loadPlyFile(const char * path, std::vector<Vector3D> &points) {
  std::cout << "Loading PLY file" << endl;
  FILE* file = fopen(path, "r");

  char word[64];
  int num_vertices;
  while (true) {

    fscanf(file, "%s", word);

    if (strcmp(word, "element") == 0) {
      char element[64];
      fscanf(file, "%s", element);

      if (strcmp(element, "vertex") == 0) {
        fscanf(file, "%d", &num_vertices);
      }
    }

    if (strcmp(word, "end_header") == 0) {
      break;
    }

  }

  for (int _ = num_vertices; _--; _ > 0) {

    float x, y, z;
    fscanf(file, "%f", &x);
    fscanf(file, "%f", &y);
    fscanf(file, "%f", &z);
    Vector3D point = Vector3D(x, y, z);
    points.push_back(point);
    
  }

  fclose(file);
}

int main( int argc, char** argv ) {

  const char* path = argv[1];
  std::string path_str = path;

  //////////////////////////////
  // Bezier curve viewer code //
  //////////////////////////////

  if (path_str.substr(path_str.length()-4, 4) == ".bzc")
  {
    // Each file contains a single Bezier curve's control points
    FILE* file = fopen(path, "r");

    int numControlPoints;
    fscanf(file, "%d", &numControlPoints);

    BezierCurve curve(numControlPoints);
    curve.loadControlPoints(file);
    fclose(file);

    // Create viewer
    Viewer viewer = Viewer();
    viewer.set_renderer(&curve);
    viewer.init();
    viewer.start();

    exit(EXIT_SUCCESS);

    return 0;
  }

  // create viewer
  Viewer viewer = Viewer();

  // create collada_viewer
  MeshEdit* collada_viewer = new MeshEdit();

  // set collada_viewer as renderer
  viewer.set_renderer(collada_viewer);

  // init viewer
  viewer.init();

  // load tests
  if( argc == 2 ) {
    if (loadFile(collada_viewer, argv[1]) < 0) exit(0);
  } else {
    msg("Usage: ./meshedit <path to scene file>"); exit(0);
  }

  // start viewer
  viewer.start();

  return 0;
}
