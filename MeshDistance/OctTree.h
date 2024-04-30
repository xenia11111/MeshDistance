#include<iostream>
#include <array>
#include <ranges>
#include <algorithm>
#include <queue>
#include "TriangledMesh.h"



class OctTree {

public:

	/* Структура OctTree принимает указатель на меш, на котором будет строиться октодерево, максимальное значение числа треугольников,
    содержащихся в bounding box листового узла октодерева, максимальную глубину октодерева, минимальное значение длины ребра bounding box листового узла октодерева*/ 
	OctTree(std::shared_ptr<TriangledMesh> const& mesh, size_t maxTrianglesPerNode, size_t maxDepth, float minBoxSize) : minNumberOfTriangles(maxTrianglesPerNode),
																									    maxDepth(maxDepth),
		                                                                                                minBoxSize_(minBoxSize)
	{
		mesh_ = mesh;
		auto aabb = mesh_->getAABB();
		root = new OctNode;
		Build(root, mesh->getTriangles(), {} , aabb, 0);
	}

	/* Узел октодерева. Содержит ограничивающий параллелипипед, массив треугольников, которые содержатся в нем, ссылки на потомков*/
	struct OctNode {

		aiAABB aabb_;
		bool isLeaf = true;
		std::vector<Triangle> triangles_;
		std::vector<OctNode*> children_;
	};

	float getDistanceFromMesh(TriangledMesh const& otherMesh, aiVector3D const& direction) const;
	
	OctNode* root;
	
	size_t mutable buildOperations = 0;
	
private:
	
	float getDistanceFromPoint(OctNode* octNode, aiVector3D const& direction, aiVector3D const& origin) const;
	
	void Build(OctNode*& node, std::vector<Triangle> const& trianglesInside, std::vector<Triangle> const& trianglesIntersect, aiAABB const& aabb, int depth) const;

	size_t maxDepth = 10;

	size_t minNumberOfTriangles = 1;

	float minBoxSize_ = 0.1f;

	std::shared_ptr<TriangledMesh> mesh_;

	static std::array<aiAABB, 8> getOctants(aiAABB const& aabb)
	{
		aiVector3D BboxMin = aabb.mMin;
		aiVector3D BboxMax = aabb.mMax;

		aiVector3D center = BboxMin + (BboxMax - BboxMin) / 2.0f;

		std::array<aiAABB, 8> octants
		{
			aiAABB{ BboxMin, center },
			{ { center.x, BboxMin.y, BboxMin.z }, { BboxMax.x, center.y, center.z } },
			{ { center.x, BboxMin.y, center.z }, { BboxMax.x, center.y, BboxMax.z } },
			{ { BboxMin.x, BboxMin.y, center.z }, { center.x, center.y, BboxMax.z } },
			{ { BboxMin.x, center.y, BboxMin.z }, { center.x, BboxMax.y, center.z } },
			{ { center.x, center.y, BboxMin.z }, { BboxMax.x, BboxMax.y, center.z } },
			{ center, BboxMax },
			{ { BboxMin.x, center.y, center.z }, { center.x, BboxMax.y, BboxMax.z } }
		};

		return octants;
	}
};


