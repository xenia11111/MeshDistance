#pragma once
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
#include <unordered_map>



import Geometry;
using namespace geometry;

class TriangledMesh
{
    public:
        explicit TriangledMesh(std::string const& filename);
      
        void processNode(aiNode* node, const aiScene* scene);
        void processMesh(aiMesh* mesh, const aiScene* scene);
        float getDistance(TriangledMesh const& other, aiVector3D const& direction);
        std::vector<Triangle> getTriangles() const;
        aiAABB getAABB() const { return aabb_; }
        std::vector<aiVector3D> getVertices() const { return vertices_; }
        
        size_t operations = 0;
    private:
        std::vector<aiVector3D> vertices_;
        std::vector<aiFace> faces_;
        aiAABB aabb_;
};


