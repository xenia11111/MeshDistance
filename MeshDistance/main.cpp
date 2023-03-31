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

#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include <cassert>
#include <exception>
#include <memory>
#include "BVH.h"   

int main()
{
    std::string resPath = "C:/Users/Frog/source/repos/MeshDistance/res/";

    TriangledMesh first{ resPath + "02_SIDE_PLATE_LEFT.stl" };
    TriangledMesh second{ resPath + "model2.stl" };
    aiVector3D direction(1, 0, 0);

   // std::cout << first.getDistance(second, direction) << "\n";

   
    OctTree* root = new OctTree(first.getAABB(), first.getTriangles());
    root.Build();
    return EXIT_SUCCESS;
}



