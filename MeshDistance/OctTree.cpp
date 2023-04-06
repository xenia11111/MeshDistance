#include "OctTree.h"
#include <cmath>
#include <ranges>
#include <algorithm>
#include <queue>


void OctTree::Build(OctNode*& node, std::vector<Triangle> const& triangles, aiAABB const& aabb, int depth)
{
	node->aabb_ = aabb;
	if (depth >= maxDepth || triangles.size() <= minNumberOfTriangles)
	{
		node->triangles_ = triangles;
		return;
	}

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

    std::array<std::vector<Triangle>, 8> childNodeTriangles;

	for (auto const& tri : triangles)
	{
		bool triangleInChildBox = false;
		for (int i = 0; i < 8; i++)
		{
			if (tri.IsInsideBox(octants[i]))
			{
				childNodeTriangles[i].push_back(tri);
				triangleInChildBox = true;
				break;
			}
		}
		if (!triangleInChildBox)
		{
			node->triangles_.push_back(tri);
		}
	}
	
	for (int i = 0; i < 8; i++) {

		if (childNodeTriangles[i].size() > 0) 
		{
			OctNode* child = new OctNode{};
			Build(child, childNodeTriangles[i], octants[i], depth + 1);
			node->children_.push_back(child);
		}
	}
} 


float OctTree::getDistanceFromMesh(aiVector3D const& direction, TriangledMesh const& otherMesh) 
{
	float distance = std::numeric_limits<float>::max();
	size_t vertexCount = 0;
	for (auto const& vertex : otherMesh.getVertices()) 
	{
		/*++vertexCount;
		if(vertexCount%10000 == 0)
			std::cout << "Vertex " << vertexCount << "\n";*/

		distance = std::min(distance, getDistanceFromPoint(root, direction, vertex));
	}
	return distance;
}

float OctTree::getDistanceFromPoint(aiVector3D const& dir, aiVector3D const& origin)
{
	return getDistanceFromPoint(root, dir, origin);
}



/*inline std::ostream& operator << (std::ostream& out, aiVector3D const& vec)
{
	out << "{ "<<vec.x << " " << vec.y << " " << vec.z << " }";
	return out;
}

inline std::ostream& operator << (std::ostream& out, aiAABB const& aabb)
{
	out << "{ Min: " << aabb.mMin <<"    Max: " << aabb.mMax << " }";
	return out;
}

inline std::ostream& operator << (std::ostream& out, Triangle const& triangle)
{
	out << "{ v0: " << triangle.v0 << " v1: " << triangle.v1 << " v2: " << triangle.v2 << " }";
	return out;
}*/



float OctTree::getDistanceFromPoint(OctNode* node, aiVector3D const& dir, aiVector3D const& origin) 
{
	float distance = std::numeric_limits<float>::max();
	if (geometry::rayIntersectBox(origin, dir, node->aabb_).first)
	{
		for (auto const& triangle : node->triangles_)
		{
			auto d = geometry::getDistance(origin, triangle, dir);
			distance = std::min(distance, d);
		}

		for (auto& child : node->children_)
		{

			distance = std::min(distance, getDistanceFromPoint(child, dir, origin));
		}
	}
	
	return distance;
}




