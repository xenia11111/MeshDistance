#include<iostream>
#include <array>
#include <ranges>
#include <algorithm>
#include "TriangledMesh.hpp"



class OctTree {

public:
	OctTree(aiAABB aabb, std::vector<Triangle> triangles, size_t maxTrianglesPerNode, size_t maxDepth): minNumberOfTriangles(maxTrianglesPerNode),
																									    maxDepth(maxDepth)
	{
		root = new OctNode;
		Build(root, triangles, aabb, 0);
	}
	
	struct OctNode {

		aiAABB aabb_;
	
		std::vector<Triangle> triangles_;
		std::vector<OctNode*> children_;
	};

	float getDistanceFromMesh(aiVector3D const& direction, TriangledMesh const& otherMesh);

	
    /*size_t getTrianglesCount() const
	{
		return getCount(root);
	}

	static size_t getCount(OctNode* node)
	{
		size_t sum = 0;
		sum += node->triangles_.size();
		for (auto const& child : node->children_)
		{
			sum += getCount(child);
		}
		return sum;
	}*/


private:
	float getDistanceFromPoint(aiVector3D const& direction, aiVector3D const& origin);

	float getDistanceFromPoint(OctNode* octNode, aiVector3D const& direction, aiVector3D const& origin);

	void Build(OctNode*& octNode, std::vector<Triangle> const& triangles, aiAABB const& aabb, int depth);
	
	OctNode* root;

	std::vector<Triangle> triangles;

	size_t maxDepth = 10;

	size_t minNumberOfTriangles = 1;

};


