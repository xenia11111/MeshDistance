#include<iostream>
#include "TriangledMesh.hpp"
import Geometry;
using namespace geometry;


class OctTree {
public:
	OctTree(aiAABB aabb_, std::vector<Triangle> triangles_) 
	{
		std::vector<OctNode> childNodes;
		OctNode root{ childNodes, aabb_, triangles_ };
	};
	void Build();
private:	
	OctNode root;
	const unsigned int maxDepth = 100;
	const unsigned int minNumberOfTriangles = 1;
};

struct OctNode {
	std::vector<OctNode> childNodes;
	aiAABB aabb;
	std::vector<Triangle> triangles;
};
