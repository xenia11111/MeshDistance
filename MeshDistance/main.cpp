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
#include "OctTree.h"   
#include <time.h> 



int main()
{ 
   std::string resPath = "C:/Users/Frog/source/repos/MeshDistance/res/";

   TriangledMesh first{ resPath + "02_SIDE_PLATE_LEFT.stl" };
   TriangledMesh second{ resPath + "model2.stl" };

   aiVector3D direction(1, 0, 0);

   // std::cout << first.getDistance(second, direction) << "\n";
   time_t start, end;

   std::cout << "Octree building start..." << "\n";
   time(&start);

   OctTree tree{ first.getAABB(), first.getTriangles(), 1, 10 };

   time(&end);
   std::cout << "Octree was built in..." << difftime(end, start) << "sec" << "\n";

   std::cout << "Start searching distance..." << "\n";
   time(&start);

   std::cout << tree.getDistanceFromMesh(direction, second) << "\n";

   time(&end);
   std::cout << "Got the distance in..." << difftime(end, start) << "sec" << "\n";


   return EXIT_SUCCESS;
}



