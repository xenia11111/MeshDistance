#include "TriangledMesh.hpp"
#include "OctTree.h"
#include <iostream> 

import Geometry;
using namespace geometry;

class BVH
{
public:
	BVH(TriangledMesh first, TriangledMesh second);
	float GetDistance();
};

