import Geometry;
#include "OctTree.h"
#include <cmath>
#include <ranges>
#include <algorithm>
#include <queue>

/* Рекурсивное построение октодерева на меше. Метод принимает указатель на корневой узел, два массива треугольников которые пересекают 
   bounding box меша, bounding box меша (aabb), текущую глубину дерева */
void OctTree::Build(OctNode*& node, std::vector<Triangle> const& trianglesInside, std::vector<Triangle> const& trianglesIntersect, aiAABB const& aabb, int depth) const
{
	
	node->aabb_ = aabb;

	std::vector<Triangle> triangles;
	std::copy(trianglesInside.begin(), trianglesInside.end(), std::back_inserter(triangles));
	std::copy(trianglesIntersect.begin(), trianglesIntersect.end(), std::back_inserter(triangles));
	
	if (depth >= maxDepth || triangles.size() <= minNumberOfTriangles // Если текущая глубина дерева больше максимальной, или 
		|| aabb.mMax.x - aabb.mMin.x < minBoxSize_                    // или число треугльников в текущем bounding box меньше
		|| aabb.mMax.y - aabb.mMin.y < minBoxSize_                    // минимального числа треугольников для узла, или 
		|| aabb.mMax.z - aabb.mMin.z < minBoxSize_                    // размеры bounding box меньше минимального размера, то
		)                                                             // заканчиваем построение октодерева.
	{
		buildOperations++;
		node->triangles_ = triangles;
		return;
	}
	
	aiVector3D BboxMin = aabb.mMin;
	aiVector3D BboxMax = aabb.mMax;
	
	aiVector3D center = BboxMin + (BboxMax - BboxMin) / 2.0f;
	

	
	std::array<aiAABB, 8> octants  // Разбиваем текущий bounding box на 8 одинаковых bb меньшего размера
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

	std::array<std::vector<Triangle>, 8> childrenTrianglesInside;
	std::array<std::vector<Triangle>, 8> childrenTrianglesIntersect;

	for (auto const& tri : triangles)
	{
		
		buildOperations++;
		aiAABB octant;
		size_t boxIntersectCount = 0;
		
		for (int i = 0; i < 8; i++)
		{
			if (tri.isInsideBox(octants[i]))
			{
				childrenTrianglesInside[i].push_back(tri);
				octant = octants[i];
			}
			else if (BoxIntersectsTriangle(octants[i], tri))
			{
				childrenTrianglesIntersect[i].push_back(tri);
			}
			
		}

	}

	for (int i = 0; i < 8; i++) 
	{
		if (childrenTrianglesInside[i].size() > 0 || childrenTrianglesIntersect[i].size() > 0)
		{
			OctNode* child = new OctNode{};
			Build(child, childrenTrianglesInside[i], childrenTrianglesIntersect[i], octants[i], depth + 1);
			node->children_.push_back(child);
		}
	}

	if (node->children_.size() > 0)
		node->isLeaf = false;
}

/* Метод принимает меш и координаты вектора направления движения. 
   Осуществялется перебор всех вершин первого меша, от каждой из которых ищется расстояние до поверхности второго меша */
float OctTree::getDistanceFromMesh(TriangledMesh const& otherMesh, aiVector3D const& direction) const
{
	float distance = std::numeric_limits<float>::max();
	
	for (auto const& vertex : otherMesh.getVertices())
	{
		distance = std::min(distance, getDistanceFromPoint(root, direction, vertex));
	}

	return distance < 0 ? distance * (-1) : distance;
}

/* Метод принимает корневой узел октодерева, построенного на втором меше, координаты вектора направления движения и вершину первого меша,
   вычисляется расстояние от вершины первого меша до второго меша в направлении заданного вектора */
float OctTree::getDistanceFromPoint(OctNode* node, aiVector3D const& direction, aiVector3D const& origin) const
{
	constexpr float maxFloat = std::numeric_limits<float>::max();
	float pointToMeshDistance = maxFloat;
	
	if (geometry::getDistanceToAABB(origin, direction, node->aabb_) != maxFloat) // Проверяем, пересекает ли луч с началом в заданной
		                                                            // вершине origin и направлением direction ограничивающий паралелипипед второго меша
	{
		// Спускаемся по октодереву второго меша вдоль узлов, которые пересекает луч, пока не достигнут листовой узел,
		// далее ищем минимальное расстояние до треугольников, содержащихся в bounding box найденого листового узла

		if (node->isLeaf) {

			for (auto const& triangle : node->triangles_)
			{

				auto pointToTriagleDistance = geometry::getDistance(origin, triangle, direction);
				pointToMeshDistance = std::min(pointToMeshDistance, pointToTriagleDistance);
			}
			return pointToMeshDistance;
		}
		
		for (auto& child : node->children_)
		{
			pointToMeshDistance = std::min(pointToMeshDistance, getDistanceFromPoint(child, direction, origin));
		}
	}
	return pointToMeshDistance;
}
