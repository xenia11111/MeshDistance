#pragma once
#include "assimp/vector3.h"
#include "assimp/aabb.h"
#include "gtest/gtest.h"
#include <assimp/vector3.h>

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
//#include "OctTree.h"
import TestGeometry;


/*TEST(TestCaseName, triangleToTriangleDistance)
{
	geometry::Triangle first{ {0, 0, 0}, {2, 0, 0}, {2, 2, 0} };
	geometry::Triangle second{ {0, 0, 2}, {3, 0, 2}, {3, 3, 2} };
	aiVector3D dir(0, 0, 1);
	auto dist = geometry::getTriangleToTriangleDist(first, second, dir);
	EXPECT_EQ(dist, 2);
}*/

/*TEST(NaiveTest, NaiveTest)
{
	std::string resPath = "C:/Users/Frog/source/repos/MeshDistance/res/";
	aiVector3D dir{ 0, -1, 0 };

	TriangledMesh from { resPath + "model1.stl" };
	TriangledMesh to { resPath + "model2.stl" };
	
	float naiveDist = from.getDistance(to, dir);

	OctTree fromTree { from.getAABB(), from.getTriangles(), 1, 100};
	OctTree toTree{ to.getAABB(), to.getTriangles(), 1, 100 };

	float dist = fromTree.getDistance(toTree, dir);

	EXPECT_FLOAT_EQ(naiveDist, dist);
}*/



TEST(BoxTriangleTest, boxToBoxDistance)
{
	aiAABB aabb1{ {0,0,0}, {1, 1, 1} };
	aiAABB aabb2{ {2,0,0}, {3,1,1} };
	aiVector3D dir = { 1, 0, 0 };
	
	float distance = geometry::getBoxToBoxDistance(aabb1, aabb2, dir);
	EXPECT_TRUE( distance == 2);
}