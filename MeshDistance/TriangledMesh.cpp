#include "TriangledMesh.hpp"


    TriangledMesh::TriangledMesh(std::string const& filename)
    {
        Assimp::Importer importer;
        aiScene const* scene = importer.ReadFile(filename, aiProcess_Triangulate
            | aiProcess_JoinIdenticalVertices
            | aiProcess_OptimizeMeshes);
        std::cerr << scene->mRootNode->mNumChildren << "\n";
        processNode(scene->mRootNode, scene);
        if (!scene)
            throw std::runtime_error("No such file or directory");
        if (scene->mNumMeshes != 1)
            throw std::runtime_error("One mesh is required");
        aiMesh_ = scene->mMeshes[0];
        if (aiMesh_->mPrimitiveTypes != aiPrimitiveType_TRIANGLE)
            throw std::runtime_error("Triangled mesh required");

    }

    void TriangledMesh::processNode(aiNode* node, const aiScene* scene)
    {
        // process all the node's meshes (if any)
        for (unsigned int i = 0; i < node->mNumMeshes; i++)
        {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            processMesh(mesh, scene);
        }
        // then do the same for each of its children
        for (unsigned int i = 0; i < node->mNumChildren; i++)
        {
            processNode(node->mChildren[i], scene);
        }
    }


    void TriangledMesh::processMesh(aiMesh* mesh, const aiScene* scene)
    {
        for (uint32_t i = 0; i < mesh->mNumVertices; i++)
        {
            vertices_.push_back(mesh->mVertices[i]);
        }

        for (uint32_t i = 0; i < mesh->mNumFaces; ++i)
        {
            faces_.push_back(mesh->mFaces[i]);
        }
    }

   
    float TriangledMesh::getDistance(TriangledMesh const& other, aiVector3D const& direction)
    {

        float distance = std::numeric_limits<float>::max();
        for (auto const& face : faces_)
        {

            Triangle triangle
            {
                vertices_[face.mIndices[0]],
                vertices_[face.mIndices[1]],
                vertices_[face.mIndices[2]],
            };

            triangles_.push_back(triangle);

            for (auto const& vertex : other.vertices_)
            {
                distance = std::min(distance, geometry::getDistance(vertex, triangle, direction).value_or(distance));
            }
        }
        return distance;
    }


