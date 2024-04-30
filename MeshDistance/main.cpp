#include <assimp/mesh.h>
#include <assimp/vector3.h>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include <stack>
#include <span>
#include <ranges>
#include <algorithm>
#include <optional>
#include <array>
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include <cassert>
#include <exception>
#include <memory>   
#include <time.h> 
#include <omp.h>
#include <fstream>
#include <filesystem>
#include "TriangledMesh.h"
#include "OctTree.h"
namespace fs = std::filesystem;

int main()
{ 

   std::string resPath = fs::current_path().parent_path().string() + "\\res\\";

  
   aiVector3D direction = { 0.0, 0.0, 1.0 };

   TriangledMesh first{ resPath + "cone1.stl" };
   TriangledMesh second{ resPath + "Sphere1.stl" };

   auto mesh_ptr = std::make_shared<TriangledMesh>(second);

   std::cout << "Start build octree..\n";

   OctTree tree(mesh_ptr, 10, std::numeric_limits<size_t>::max(), 0.1f);

   std::cout << "End build octree\n";

   std::cout << "Build operations: " << tree.buildOperations << "\n";

   std::cout << "Naive method distance: " << first.getDistance(second, direction) << " operations count: " << first.operations << "\n";
 
   std::cout << "Vertex-Octree method distance: " << tree.getDistanceFromMesh(first, direction) << "\n";

}
