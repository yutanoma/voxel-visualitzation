#include "polyscope/polyscope.h"

#include <igl/PI.h>
#include <igl/avg_edge_length.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/exact_geodesic.h>
#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/lscm.h>
#include <igl/massmatrix.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"

#include <iostream>
#include <unordered_set>
#include <utility>

// @hirata ボクセルの一辺の長さ。実際のボクセルサイズより少し小さめに設定すると良い
const double voxelSize = 0.05;

// The mesh, Eigen representation
Eigen::MatrixXd meshV;
Eigen::MatrixXi meshF;

int main(int argc, char **argv) {
  // Options
  polyscope::options::autocenterStructures = true;
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;

  // Initialize polyscope
  polyscope::init();

  std::string filename = "../mesh.obj";
  std::cout << "loading: " << filename << std::endl;

  // Read the mesh
  igl::readOBJ(filename, meshV, meshF);

  Eigen::MatrixXd hexVertices(meshV.rows() * 8, 3);

  for (int i = 0; i < meshV.rows(); i++) {
    double len = voxelSize / 2;

    hexVertices.row(i * 8 + 0) = meshV.row(i) + Eigen::RowVector3d(len, -len, -len);
    hexVertices.row(i * 8 + 1) = meshV.row(i) + Eigen::RowVector3d(len, len, -len);
    hexVertices.row(i * 8 + 2) = meshV.row(i) + Eigen::RowVector3d(-len, len, -len);
    hexVertices.row(i * 8 + 3) = meshV.row(i) + Eigen::RowVector3d(-len, -len, -len);
    hexVertices.row(i * 8 + 4) = meshV.row(i) + Eigen::RowVector3d(len, -len, len);
    hexVertices.row(i * 8 + 5) = meshV.row(i) + Eigen::RowVector3d(len, len, len);
    hexVertices.row(i * 8 + 6) = meshV.row(i) + Eigen::RowVector3d(-len, len, len);
    hexVertices.row(i * 8 + 7) = meshV.row(i) + Eigen::RowVector3d(-len, -len, len);
  }

  Eigen::MatrixXi hexIndices(meshV.rows(), 8);

  for (int i = 0; i < meshV.rows(); i++) {
    hexIndices.row(i) << i * 8 + 0, i * 8 + 1, i * 8 + 2, i * 8 + 3, i * 8 + 4, i * 8 + 5, i * 8 + 6, i * 8 + 7;
  }

  std::vector<double> hexValues(meshV.rows());

   std::ifstream file("../data.txt");
   if (!file) {
      std::cerr << "Could not open the file!" << std::endl;
      return 1;
    }

  for (int i = 0; i < meshV.rows(); i++) {
    double num;
    file >> num;
    hexValues[i] = num;
  }

  // Register the mesh with Polyscope
  polyscope::registerHexMesh("input mesh", hexVertices, hexIndices);

  polyscope::getVolumeMesh("input mesh")->addCellScalarQuantity("values", hexValues);

  // Show the gui
  polyscope::show();

  return 0;
}
