#include "OctTree.h"
#include <cmath>

void OctTree::Build() 
{
	
	aiVector3D BboxMin = aabb.mMin;
	aiVector3D BboxMax = aabb.mMax;
	aiVector3D center = BboxMin + (BboxMax - BboxMin) / 2.0f;
	std::vector<aiAABB> octants;
	octants.push_back({ BboxMin, center });
	octants.push_back({ { center.x, BboxMin.y, BboxMin.z }, { BboxMax.x, center.y, center.z } });
	octants.push_back({ { center.x, BboxMin.y, center.z }, { BboxMax.x, center.y, BboxMax.z } });
	octants.push_back({ { BboxMin.x, BboxMin.y, center.z }, { center.x, center.y, BboxMax.z } });
	octants.push_back({ { BboxMin.x, center.y, BboxMin.z }, { center.x, BboxMax.y, center.z } });
	octants.push_back({ { center.x, center.y, BboxMin.z }, { BboxMax.x, BboxMax.y, center.z } });
	octants.push_back({ center, BboxMax });
	octants.push_back({ { BboxMin.x, center.y, center.z }, { center.x, BboxMax.y, BboxMax.z } });
	
	std::vector<Triangle> childTreeTriangles;
	for (int i = 0; i < 8; i++) 
	{
		for (int j = 0; j < triangles.size(); j++) 
		{
			if (IsTriangleInBox(triangles[j], octants[i])) 
			{
				childTreeTriangles.push_back(triangles[j]);
				triangles.erase(triangles.begin() + j);
			}
		}
		if (childTreeTriangles.size()) 
		{
			childNodes[i] = new OctTree(octants[i], childTreeTriangles);
			childTreeTriangles.clear();
		}
	}
	for (auto child : childNodes) {
		child.Build();
	}
} 



bool IsTriangleInBox(Triangle triangle, aiAABB box) 
{
	if (!(triangle.v0 < box.mMin || triangle.v0 == box.mMin) && triangle.v0 < box.mMax
		&& !(triangle.v1 < box.mMin || triangle.v1 == box.mMin) && triangle.v1 < box.mMax
		&& !(triangle.v2 < box.mMin || triangle.v2 == box.mMin) && triangle.v2 < box.mMax)
	{
		return true;
	}
	else
		return false;
}
