#pragma once
#include "assimp/scene.h"
#include <assimp/types.h>
#include <optional>
#include <vector>
#include <thread>
#include <algorithm>
#include <execution>
#include <ranges>
#include <array>
#include <list>
#include <cassert>
#include "assimp/vector3.h"
#include "assimp/Importer.hpp"
#include <iostream>
#include <fstream>

export module Geometry;

std::ofstream geom_logs("geom_logs.txt");

export namespace geometry
{
    inline std::ostream& operator << (std::ostream& out, aiVector3D const& vec)
    {
        out << "{ " << vec.x << " " << vec.y << " " << vec.z << " }";
        return out;
    }
    inline std::ostream& operator << (std::ostream& out, aiAABB const& aabb)
    {
        out << "{ Min: " << aabb.mMin << "    Max: " << aabb.mMax << " }";
        return out;
    }



    export float dotProduct(aiVector3D first, aiVector3D second)
    {
        return first.x * second.x + first.y * second.y + first.z * second.z;
    }

    export aiVector3D crossProduct(aiVector3D first, aiVector3D second)
    {
        return aiVector3D(
            first.y * second.z - second.y * first.z,
            first.z * second.x - second.z * first.x,
            first.x * second.y - second.x * first.y);
    }

    export struct Plane
    {
        aiVector3D normal;
        aiVector3D pointOnPlane;
        Plane(aiVector3D const& n, aiVector3D const& p) : normal(n), pointOnPlane(p) {};
    };

    export float getPointToPlaneDistance(aiVector3D const& origin, Plane const& plane, aiVector3D const& rayDir)
    {
        auto N = plane.normal;
        auto p = plane.pointOnPlane;
        auto NdotRayDirection = dotProduct(N, rayDir);
        auto epsilon = std::numeric_limits<float>::epsilon();
        auto maxFloat = std::numeric_limits<float>::max();

        return (fabs(NdotRayDirection) < epsilon) ? maxFloat : (dotProduct(N, p) - dotProduct(N, origin)) / NdotRayDirection;
    }

    export struct Triangle
    {
        aiVector3D v0, v1, v2;
        aiVector3D normal;

        friend std::ostream& operator << (std::ostream&, Triangle const&);
        bool isInsideBox(aiAABB const& aabb) const
        {
            return (v0.x >= aabb.mMin.x && v0.x <= aabb.mMax.x &&
                v0.y >= aabb.mMin.y && v0.y <= aabb.mMax.y &&
                v0.z >= aabb.mMin.z && v0.z <= aabb.mMax.z &&
                v1.x >= aabb.mMin.x && v1.x <= aabb.mMax.x &&
                v1.y >= aabb.mMin.y && v1.y <= aabb.mMax.y &&
                v1.z >= aabb.mMin.z && v1.z <= aabb.mMax.z &&
                v2.x >= aabb.mMin.x && v2.x <= aabb.mMax.x &&
                v2.y >= aabb.mMin.y && v2.y <= aabb.mMax.y &&
                v2.z >= aabb.mMin.z && v2.z <= aabb.mMax.z);
        }

        aiVector3D getCentroid() const
        {
            float oneThird = 0.333333333f;
            return aiVector3D{ oneThird * (v0.z + v1.z + v2.z), oneThird * (v0.z + v1.z + v2.z), oneThird * (v0.z + v1.z + v2.z) };
        }

        bool isPointInTriangle(aiVector3D const& p) const
        {
            aiVector3D edge0 = v1 - v0;
            aiVector3D vp0 = p - v0;
            auto N = crossProduct(v1 - v0, v2 - v0);
            auto C = crossProduct(edge0, vp0); // vector perpendicular to triangle's plane
            if (dotProduct(N, C) < 0) return false; // P is on the right side

            // edge 2
            aiVector3D edge1 = v2 - v1;
            aiVector3D vp1 = p - v1;

            C = crossProduct(edge1, vp1); // vector perpendicular to triangle's plane
            if (dotProduct(N, C) < 0) return false; // P is on the right side

            // edge 3
            aiVector3D edge2 = v0 - v2;
            aiVector3D vp2 = p - v2;

            C = crossProduct(edge2, vp2); // vector perpendicular to triangle's plane
            if (dotProduct(N, C) < 0) return false; // P is on the right side;

            return true;
        }

        bool isInsideTriangle(Triangle const& other) const
        {
            return other.isPointInTriangle(v0) && other.isPointInTriangle(v1) && other.isPointInTriangle(v2);
        }

    };


    export float getDistance(aiVector3D const& origin, Triangle const& triangle, aiVector3D const& direction)
    {
        aiVector3D v0 = triangle.v0;
        aiVector3D v1 = triangle.v1;
        aiVector3D v2 = triangle.v2;
        aiVector3D N = crossProduct(v1 - v0, v2 - v0);
        // Step 1: finding intersection point P

        // check if the ray and plane are parallel.

        float NdotRayDirection = dotProduct(N, direction);
        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon()) // almost 0
            return std::numeric_limits<float>::max(); // they are parallel, so they don't intersect!

        float t = (dotProduct(N, v0) - dotProduct(N, origin)) / NdotRayDirection;

        //check if the triangle is behind the ray
        //if (t < 0) return std::numeric_limits<float>::max(); // the triangle is behind

        // compute the intersection point
        aiVector3D P = origin + t * direction;
        aiVector3D C;

        // Step 2: inside-outside test

        // edge 1
        aiVector3D edge0 = v1 - v0;
        aiVector3D vp0 = P - v0;

        C = crossProduct(edge0, vp0); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 2
        aiVector3D edge1 = v2 - v1;
        aiVector3D vp1 = P - v1;

        C = crossProduct(edge1, vp1); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 3
        aiVector3D edge2 = v0 - v2;
        aiVector3D vp2 = P - v2;

        C = crossProduct(edge2, vp2); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side;

        return t;
    }

    export float getDistanceToAABB(aiVector3D const& orig, aiVector3D const& dir, aiAABB const& aabb)
    {
        auto maxFloat = std::numeric_limits<float>::max();

        auto min = aabb.mMin;
        auto max = aabb.mMax;

        float tmin = (min.x - orig.x) / dir.x;
        float tmax = (max.x - orig.x) / dir.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        float tymin = (min.y - orig.y) / dir.y;
        float tymax = (max.y - orig.y) / dir.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return maxFloat;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        float tzmin = (min.z - orig.z) / dir.z;
        float tzmax = (max.z - orig.z) / dir.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return maxFloat;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        return tmin;

    }


    inline std::ostream& operator << (std::ostream& out, Triangle const& triangle)
    {
        out << "{ v0: " << triangle.v0 << " v1: " << triangle.v1 << " v2: " << triangle.v2 << " }";
        return out;
    }


    float Max(float a, float b, float c)
    {
        return std::max(std::max(a, b), c);
    }

    float Min(float a, float b, float c)
    {
        return std::min(std::min(a, b), c);
    }

    export bool BoxIntersectsTriangle(aiAABB const& aabb, Triangle const& triangle)
    {
        auto m_Center = aabb.mMin + (aabb.mMax - aabb.mMin) / 2.0f;
        auto m_Extent = aiVector3D{ aabb.mMax.x - m_Center.x, aabb.mMax.y - m_Center.y , aabb.mMax.z - m_Center.z };

        auto v0 = triangle.v0 - m_Center;
        auto v1 = triangle.v1 - m_Center;
        auto v2 = triangle.v2 - m_Center;

        // Compute edge vectors for triangle
        auto f0 = triangle.v1 - triangle.v0;
        auto f1 = triangle.v2 - triangle.v1;
        auto f2 = triangle.v0 - triangle.v2;

        //// region Test axes a00..a22 (category 3)

        // Test axis a00
        auto a00 = aiVector3D(0, -f0.z, f0.y);
        auto p0 = dotProduct(v0, a00);
        auto p1 = dotProduct(v1, a00);
        auto p2 = dotProduct(v2, a00);
        auto r = m_Extent.y * fabs(f0.z) + m_Extent.z * fabs(f0.y);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a01
        auto a01 = aiVector3D(0, -f1.z, f1.y);
        p0 = dotProduct(v0, a01);
        p1 = dotProduct(v1, a01);
        p2 = dotProduct(v2, a01);
        r = m_Extent.y * fabs(f1.z) + m_Extent.z * fabs(f1.y);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a02
        auto a02 = aiVector3D(0, -f2.z, f2.y);
        p0 = dotProduct(v0, a02);
        p1 = dotProduct(v1, a02);
        p2 = dotProduct(v2, a02);
        r = m_Extent.y * fabs(f2.z) + m_Extent.z * fabs(f2.y);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a10
        auto a10 = aiVector3D(f0.z, 0, -f0.x);
        p0 = dotProduct(v0, a10);
        p1 = dotProduct(v1, a10);
        p2 = dotProduct(v2, a10);
        r = m_Extent.x * fabs(f0.z) + m_Extent.z * fabs(f0.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a11
        auto a11 = aiVector3D(f1.z, 0, -f1.x);
        p0 = dotProduct(v0, a11);
        p1 = dotProduct(v1, a11);
        p2 = dotProduct(v2, a11);
        r = m_Extent.x * fabs(f1.z) + m_Extent.z * fabs(f1.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a12
        auto a12 = aiVector3D(f2.z, 0, -f2.x);
        p0 = dotProduct(v0, a12);
        p1 = dotProduct(v1, a12);
        p2 = dotProduct(v2, a12);
        r = m_Extent.x * fabs(f2.z) + m_Extent.z * fabs(f2.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a20
        auto a20 = aiVector3D(-f0.y, f0.x, 0);
        p0 = dotProduct(v0, a20);
        p1 = dotProduct(v1, a20);
        p2 = dotProduct(v2, a20);
        r = m_Extent.x * fabs(f0.y) + m_Extent.y * fabs(f0.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a21
        auto a21 = aiVector3D(-f1.y, f1.x, 0);
        p0 = dotProduct(v0, a21);
        p1 = dotProduct(v1, a21);
        p2 = dotProduct(v2, a21);
        r = m_Extent.x * fabs(f1.y) + m_Extent.y * fabs(f1.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        // Test axis a22
        auto a22 = aiVector3D(-f2.y, f2.x, 0);
        p0 = dotProduct(v0, a22);
        p1 = dotProduct(v1, a22);
        p2 = dotProduct(v2, a22);
        r = m_Extent.x * fabs(f2.y) + m_Extent.y * fabs(f2.x);
        if (std::max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r)
            return false;

        //// endregion

        //// region Test the three axes corresponding to the face normals of AABB b (category 1)

        // Exit if...
        // ... [-extents.X, extents.X] and [Min(v0.X,v1.X,v2.X), Max(v0.X,v1.X,v2.X)] do not overlap
        if (Max(v0.x, v1.x, v2.x) < -m_Extent.x || Min(v0.x, v1.x, v2.x) > m_Extent.x)
            return false;

        // ... [-extents.Y, extents.Y] and [Min(v0.Y,v1.Y,v2.Y), Max(v0.Y,v1.Y,v2.Y)] do not overlap
        if (Max(v0.y, v1.y, v2.y) < -m_Extent.y || Min(v0.y, v1.y, v2.y) > m_Extent.y)
            return false;

        // ... [-extents.Z, extents.Z] and [Min(v0.Z,v1.Z,v2.Z), Max(v0.Z,v1.Z,v2.Z)] do not overlap
        if (Max(v0.z, v1.z, v2.z) < -m_Extent.z || Min(v0.z, v1.z, v2.z) > m_Extent.z)
            return false;

        //// endregion

        //// region Test separating axis corresponding to triangle face normal (category 2)

        auto plane_normal = crossProduct(f0, f1);
        auto plane_distance = fabs(dotProduct(plane_normal, v0));

        // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
        r = m_Extent.x * fabs(plane_normal.x) + m_Extent.y * fabs(plane_normal.y) + m_Extent.z * fabs(plane_normal.z);

        // Intersection occurs when plane distance falls within [-r,+r] interval
        if (plane_distance > r)
            return false;

        //// endregion

        return true;
    }

}


