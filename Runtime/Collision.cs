// Copyright (c) 2010-2014 SharpDX - Alexandre Mutel
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------
// Original code from SlimMath project. http://code.google.com/p/slimmath/
// Greetings to SlimDX Group. Original code published with the following license:
// -----------------------------------------------------------------------------
/*
* Copyright (c) 2007-2011 SlimDX Group
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

using System;
using UnityEngine;

namespace SharpDX
{
    /*
     * This class is organized so that the least complex objects come first so that the least
     * complex objects will have the most methods in most cases. Note that not all shapes exist
     * at this time and not all shapes have a corresponding struct. Only the objects that have
     * a corresponding struct should come first in naming and in parameter order. The order of
     * complexity is as follows:
     * 
     * 1. Point
     * 2. Ray
     * 3. Segment
     * 4. Plane
     * 5. Triangle
     * 6. Polygon
     * 7. Box
     * 8. Sphere
     * 9. Ellipsoid
     * 10. Cylinder
     * 11. Cone
     * 12. Capsule
     * 13. Torus
     * 14. Polyhedron
     * 15. Frustum
    */

    /// <summary>
    /// Contains static methods to help in determining intersections, containment, etc.
    /// </summary>
    public static class Collision
    {
        /// <summary>
        /// Determines the closest point between a point and a triangle.
        /// </summary>
        /// <param name="point">The point to test.</param>
        /// <param name="vertex1">The first vertex to test.</param>
        /// <param name="vertex2">The second vertex to test.</param>
        /// <param name="vertex3">The third vertex to test.</param>
        /// <param name="result">When the method completes, contains the closest point between the two objects.</param>
        public static void ClosestPointPointTriangle(ref Vector3 point, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3, out Vector3 result)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 136

            //Check if P in vertex region outside A
            Vector3 ab = vertex2 - vertex1;
            Vector3 ac = vertex3 - vertex1;
            Vector3 ap = point - vertex1;

            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                result = vertex1; //Barycentric coordinates (1,0,0)
                return;
            }

            //Check if P in vertex region outside B
            Vector3 bp = point - vertex2;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                result = vertex2; // Barycentric coordinates (0,1,0)
                return;
            }

            //Check if P in edge region of AB, if so return projection of P onto AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                float v = d1 / (d1 - d3);
                result = vertex1 + v * ab; //Barycentric coordinates (1-v,v,0)
                return;
            }

            //Check if P in vertex region outside C
            Vector3 cp = point - vertex3;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                result = vertex3; //Barycentric coordinates (0,0,1)
                return;
            }

            //Check if P in edge region of AC, if so return projection of P onto AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                float w = d2 / (d2 - d6);
                result = vertex1 + w * ac; //Barycentric coordinates (1-w,0,w)
                return;
            }

            //Check if P in edge region of BC, if so return projection of P onto BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                result = vertex2 + w * (vertex3 - vertex2); //Barycentric coordinates (0,1-w,w)
                return;
            }

            //P inside face region. Compute Q through its Barycentric coordinates (u,v,w)
            float denom = 1.0f / (va + vb + vc);
            float v2 = vb * denom;
            float w2 = vc * denom;
            result = vertex1 + ab * v2 + ac * w2; //= u*vertex1 + v*vertex2 + w*vertex3, u = va * denom = 1.0f - v - w
        }

        /// <summary>
        /// Determines the closest point between a <see cref="Plane"/> and a point.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="point">The point to test.</param>
        /// <param name="result">When the method completes, contains the closest point between the two objects.</param>
        public static void ClosestPointPlanePoint(ref Plane plane, ref Vector3 point, out Vector3 result)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 126

            float dot = Vector3.Dot(plane.normal, point);
            float t = dot - plane.distance;

            result = point - (t * plane.normal);
        }

        /// <summary>
        /// Determines the closest point between a <see cref="BoundingBox"/> and a point.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="point">The point to test.</param>
        /// <param name="result">When the method completes, contains the closest point between the two objects.</param>
        public static void ClosestPointBoxPoint(ref BoundingBox box, ref Vector3 point, out Vector3 result)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 130

            Vector3 temp = Vector3.Max(point, box.min);
            result = Vector3.Min(temp, box.max);
        }

        /// <summary>
        /// Determines the closest point between a <see cref="BoundingSphere"/> and a point.
        /// </summary>
        /// <param name="sphere"></param>
        /// <param name="point">The point to test.</param>
        /// <param name="result">When the method completes, contains the closest point between the two objects;
        /// or, if the point is directly in the center of the sphere, contains <see cref="Vector3.Zero"/>.</param>
        public static void ClosestPointSpherePoint(ref BoundingSphere sphere, ref Vector3 point, out Vector3 result)
        {
            //Source: Jorgy343
            //Reference: None

            //Get the unit direction from the sphere's center to the point.
            result = point - sphere.center;
            result.Normalize();

            //Multiply the unit direction by the sphere's radius to get a vector
            //the length of the sphere.
            result *= sphere.radius;

            //Add the sphere's center to the direction to get a point on the sphere.
            result += sphere.center;
        }

        /// <summary>
        /// Determines the closest point between a <see cref="BoundingSphere"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="sphere1">The first sphere to test.</param>
        /// <param name="sphere2">The second sphere to test.</param>
        /// <param name="result">When the method completes, contains the closest point between the two objects;
        /// or, if the point is directly in the center of the sphere, contains <see cref="Vector3.Zero"/>.</param>
        /// <remarks>
        /// If the two spheres are overlapping, but not directly on top of each other, the closest point
        /// is the 'closest' point of intersection. This can also be considered is the deepest point of
        /// intersection.
        /// </remarks>
        public static void ClosestPointSphereSphere(ref BoundingSphere sphere1, ref BoundingSphere sphere2, out Vector3 result)
        {
            //Source: Jorgy343
            //Reference: None

            //Get the unit direction from the first sphere's center to the second sphere's center.
            result = sphere2.center - sphere1.center;
            result.Normalize();

            //Multiply the unit direction by the first sphere's radius to get a vector
            //the length of the first sphere.
            result *= sphere1.radius;

            //Add the first sphere's center to the direction to get a point on the first sphere.
            result += sphere1.center;
        }

        /// <summary>
        /// Determines the distance between a <see cref="Plane"/> and a point.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>The distance between the two objects.</returns>
        public static float DistancePlanePoint(ref Plane plane, ref Vector3 point)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 127

            float dot = Vector3.Dot(plane.normal, point);
            return dot - plane.distance;
        }

        /// <summary>
        /// Determines the distance between a <see cref="BoundingBox"/> and a point.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>The distance between the two objects.</returns>
        public static float DistanceBoxPoint(ref BoundingBox box, ref Vector3 point)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 131

            float distance = 0f;

            if (point.x < box.min.x)
                distance += (box.min.x - point.x) * (box.min.x - point.x);
            if (point.x > box.max.x)
                distance += (point.x - box.max.x) * (point.x - box.max.x);

            if (point.y < box.min.y)
                distance += (box.min.y - point.y) * (box.min.y - point.y);
            if (point.y > box.max.y)
                distance += (point.y - box.max.y) * (point.y - box.max.y);

            if (point.z < box.min.z)
                distance += (box.min.z - point.z) * (box.min.z - point.z);
            if (point.z > box.max.z)
                distance += (point.z - box.max.z) * (point.z - box.max.z);

            return Mathf.Sqrt(distance);
        }

        /// <summary>
        /// Determines the distance between a <see cref="BoundingBox"/> and a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="box1">The first box to test.</param>
        /// <param name="box2">The second box to test.</param>
        /// <returns>The distance between the two objects.</returns>
        public static float DistanceBoxBox(ref BoundingBox box1, ref BoundingBox box2)
        {
            //Source:
            //Reference:

            float distance = 0f;

            //Distance for X.
            if (box1.min.x > box2.max.x)
            {
                float delta = box2.max.x - box1.min.x;
                distance += delta * delta;
            }
            else if (box2.min.x > box1.max.x)
            {
                float delta = box1.max.x - box2.min.x;
                distance += delta * delta;
            }

            //Distance for Y.
            if (box1.min.y > box2.max.y)
            {
                float delta = box2.max.y - box1.min.y;
                distance += delta * delta;
            }
            else if (box2.min.y > box1.max.y)
            {
                float delta = box1.max.y - box2.min.y;
                distance += delta * delta;
            }

            //Distance for Z.
            if (box1.min.z > box2.max.z)
            {
                float delta = box2.max.z - box1.min.z;
                distance += delta * delta;
            }
            else if (box2.min.z > box1.max.z)
            {
                float delta = box1.max.z - box2.min.z;
                distance += delta * delta;
            }

            return Mathf.Sqrt(distance);
        }

        /// <summary>
        /// Determines the distance between a <see cref="BoundingSphere"/> and a point.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>The distance between the two objects.</returns>
        public static float DistanceSpherePoint(ref BoundingSphere sphere, ref Vector3 point)
        {
            //Source: Jorgy343
            //Reference: None

            float distance = Vector3.Distance(sphere.center, point);
            distance -= sphere.radius;

            return Math.Max(distance, 0f);
        }

        /// <summary>
        /// Determines the distance between a <see cref="BoundingSphere"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="sphere1">The first sphere to test.</param>
        /// <param name="sphere2">The second sphere to test.</param>
        /// <returns>The distance between the two objects.</returns>
        public static float DistanceSphereSphere(ref BoundingSphere sphere1, ref BoundingSphere sphere2)
        {
            //Source: Jorgy343
            //Reference: None

            float distance = Vector3.Distance(sphere1.center, sphere2.center);
            distance -= sphere1.radius + sphere2.radius;

            return Math.Max(distance, 0f);
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a point.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>Whether the two objects intersect.</returns>
        public static bool RayIntersectsPoint(ref Ray ray, ref Vector3 point)
        {
            //Source: RayIntersectsSphere
            //Reference: None

            Vector3 m = ray.origin - point;

            //Same thing as RayIntersectsSphere except that the radius of the sphere (point)
            //is the epsilon for zero.
            float b = Vector3.Dot(m, ray.direction);
            float c = Vector3.Dot(m, m) - Mathf.Epsilon;

            if (c > 0f && b > 0f)
                return false;

            float discriminant = b * b - c;

            if (discriminant < 0f)
                return false;

            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="Ray"/>.
        /// </summary>
        /// <param name="ray1">The first ray to test.</param>
        /// <param name="ray2">The second ray to test.</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.Zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersect.</returns>
        /// <remarks>
        /// This method performs a ray vs ray intersection test based on the following formula
        /// from Goldman.
        /// <code>s = det([o_2 - o_1, d_2, d_1 x d_2]) / ||d_1 x d_2||^2</code>
        /// <code>t = det([o_2 - o_1, d_1, d_1 x d_2]) / ||d_1 x d_2||^2</code>
        /// Where o_1 is the position of the first ray, o_2 is the position of the second ray,
        /// d_1 is the normalized direction of the first ray, d_2 is the normalized direction
        /// of the second ray, det denotes the determinant of a matrix, x denotes the cross
        /// product, [ ] denotes a matrix, and || || denotes the length or magnitude of a vector.
        /// </remarks>
        public static bool RayIntersectsRay(ref Ray ray1, ref Ray ray2, out Vector3 point)
        {
            //Source: Real-Time Rendering, Third Edition
            //Reference: Page 780

            Vector3 cross = Vector3.Cross(ray1.direction, ray2.direction);
            float denominator = cross.magnitude;

            // Lines are parallel.
            if (Mathf.Approximately(denominator, 0))
            {
                //Lines are parallel and on top of each other.
                if (Mathf.Approximately(ray2.origin.x, ray1.origin.x) &&
                    Mathf.Approximately(ray2.origin.y, ray1.origin.y) &&
                    Mathf.Approximately(ray2.origin.z, ray1.origin.z))
                {
                    point = Vector3.zero;
                    return true;
                }
            }

            denominator = denominator * denominator;

            //3x3 matrix for the first ray.
            float m11 = ray2.origin.x - ray1.origin.x;
            float m12 = ray2.origin.y - ray1.origin.y;
            float m13 = ray2.origin.z - ray1.origin.z;
            float m21 = ray2.direction.x;
            float m22 = ray2.direction.y;
            float m23 = ray2.direction.z;
            float m31 = cross.x;
            float m32 = cross.y;
            float m33 = cross.z;

            //Determinant of first matrix.
            float dets =
                m11 * m22 * m33 +
                m12 * m23 * m31 +
                m13 * m21 * m32 -
                m11 * m23 * m32 -
                m12 * m21 * m33 -
                m13 * m22 * m31;

            //3x3 matrix for the second ray.
            m21 = ray1.direction.x;
            m22 = ray1.direction.y;
            m23 = ray1.direction.z;

            //Determinant of the second matrix.
            float dett =
                m11 * m22 * m33 +
                m12 * m23 * m31 +
                m13 * m21 * m32 -
                m11 * m23 * m32 -
                m12 * m21 * m33 -
                m13 * m22 * m31;

            //t values of the point of intersection.
            float s = dets / denominator;
            float t = dett / denominator;

            //The points of intersection.
            Vector3 point1 = ray1.origin + (s * ray1.direction);
            Vector3 point2 = ray2.origin + (t * ray2.direction);

            //If the points are not equal, no intersection has occurred.
            if (!Mathf.Approximately(point2.x, point1.x) ||
                !Mathf.Approximately(point2.y, point1.y) ||
                !Mathf.Approximately(point2.z, point1.z))
            {
                point = Vector3.zero;
                return false;
            }

            point = point1;
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="Plane"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="plane">The plane to test.</param>
        /// <param name="distance">When the method completes, contains the distance of the intersection,
        /// or 0 if there was no intersection.</param>
        /// <returns>Whether the two objects intersect.</returns>
        public static bool RayIntersectsPlane(ref Ray ray, ref Plane plane, out float distance)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 175

            float direction = Vector3.Dot(plane.normal, ray.direction);

            if (Mathf.Approximately(direction, 0))
            {
                distance = 0f;
                return false;
            }

            float position = Vector3.Dot(plane.normal, ray.origin);
            distance = (-plane.distance - position) / direction;

            if (distance < 0f)
            {
                distance = 0f;
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="Plane"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="plane">The plane to test</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.Zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool RayIntersectsPlane(ref Ray ray, ref Plane plane, out Vector3 point)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 175

            float distance;
            if (!RayIntersectsPlane(ref ray, ref plane, out distance))
            {
                point = Vector3.zero;
                return false;
            }

            point = ray.origin + (ray.direction * distance);
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a triangle.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <param name="distance">When the method completes, contains the distance of the intersection,
        /// or 0 if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        /// <remarks>
        /// This method tests if the ray intersects either the front or back of the triangle.
        /// If the ray is parallel to the triangle's plane, no intersection is assumed to have
        /// happened. If the intersection of the ray and the triangle is behind the origin of
        /// the ray, no intersection is assumed to have happened. In both cases of assumptions,
        /// this method returns false.
        /// </remarks>
        public static bool RayIntersectsTriangle(ref Ray ray, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3, out float distance)
        {
            //Source: Fast Minimum Storage Ray / Triangle Intersection
            //Reference: http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf

            //Compute vectors along two edges of the triangle.

            //Edge 1
            var edge1 = vertex2 = vertex1;

            //Edge2
            var edge2 = vertex3 = vertex1;

            //Cross product of ray direction and edge2 - first part of determinant.
            Vector3 directioncrossedge2;
            directioncrossedge2.x = (ray.direction.y * edge2.z) - (ray.direction.z * edge2.y);
            directioncrossedge2.y = (ray.direction.z * edge2.x) - (ray.direction.x * edge2.z);
            directioncrossedge2.z = (ray.direction.x * edge2.y) - (ray.direction.y * edge2.x);

            //Compute the determinant.
            float determinant;
            //Dot product of edge1 and the first part of determinant.
            determinant = (edge1.x * directioncrossedge2.x) + (edge1.y * directioncrossedge2.y) + (edge1.z * directioncrossedge2.z);

            //If the ray is parallel to the triangle plane, there is no collision.
            //This also means that we are not culling, the ray may hit both the
            //back and the front of the triangle.
            if (Mathf.Approximately(determinant, 0))
            {
                distance = 0f;
                return false;
            }

            float inversedeterminant = 1.0f / determinant;

            //Calculate the U parameter of the intersection point.
            Vector3 distanceVector = ray.origin - vertex1;

            float triangleU;
            triangleU = (distanceVector.x * directioncrossedge2.x) + (distanceVector.y * directioncrossedge2.y) + (distanceVector.z * directioncrossedge2.z);
            triangleU *= inversedeterminant;

            //Make sure it is inside the triangle.
            if (triangleU < 0f || triangleU > 1f)
            {
                distance = 0f;
                return false;
            }

            //Calculate the V parameter of the intersection point.
            Vector3 distancecrossedge1;
            distancecrossedge1.x = (distanceVector.y * edge1.z) - (distanceVector.z * edge1.y);
            distancecrossedge1.y = (distanceVector.z * edge1.x) - (distanceVector.x * edge1.z);
            distancecrossedge1.z = (distanceVector.x * edge1.y) - (distanceVector.y * edge1.x);

            float triangleV;
            triangleV = ((ray.direction.x * distancecrossedge1.x) + (ray.direction.y * distancecrossedge1.y)) + (ray.direction.z * distancecrossedge1.z);
            triangleV *= inversedeterminant;

            //Make sure it is inside the triangle.
            if (triangleV < 0f || triangleU + triangleV > 1f)
            {
                distance = 0f;
                return false;
            }

            //Compute the distance along the ray to the triangle.
            float raydistance;
            raydistance = (edge2.x * distancecrossedge1.x) + (edge2.y * distancecrossedge1.y) + (edge2.z * distancecrossedge1.z);
            raydistance *= inversedeterminant;

            //Is the triangle behind the ray origin?
            if (raydistance < 0f)
            {
                distance = 0f;
                return false;
            }

            distance = raydistance;
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a triangle.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.Zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        // public static bool RayIntersectsTriangle(ref Ray ray, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3, out Vector3 point)
        // {
        //     float distance;
        //     if (!RayIntersectsTriangle(ref ray, ref vertex1, ref vertex2, ref vertex3, out distance))
        //     {
        //         point = Vector3.Zero;
        //         return false;
        //     }
        //
        //     point = ray.origin + (ray.direction * distance);
        //     return true;
        // }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="box">The box to test.</param>
        /// <param name="distance">When the method completes, contains the distance of the intersection,
        /// or 0 if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool RayIntersectsBox(ref Ray ray, ref BoundingBox box, out float distance)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 179
        
            distance = 0f;
            float tmax = float.MaxValue;
        
            if (Mathf.Approximately(ray.direction.x, 0))
            {
                if (ray.origin.x < box.min.x || ray.origin.x > box.max.x)
                {
                    distance = 0f;
                    return false;
                }
            }
            else
            {
                float inverse = 1.0f / ray.direction.x;
                float t1 = (box.min.x - ray.origin.x) * inverse;
                float t2 = (box.max.x - ray.origin.x) * inverse;
        
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
        
                distance = Math.Max(t1, distance);
                tmax = Math.Min(t2, tmax);
        
                if (distance > tmax)
                {
                    distance = 0f;
                    return false;
                }
            }
        
            if (Mathf.Approximately(ray.direction.y, 0))
            {
                if (ray.origin.y < box.min.y || ray.origin.y > box.max.y)
                {
                    distance = 0f;
                    return false;
                }
            }
            else
            {
                float inverse = 1.0f / ray.direction.y;
                float t1 = (box.min.y - ray.origin.y) * inverse;
                float t2 = (box.max.y - ray.origin.y) * inverse;
        
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
        
                distance = Math.Max(t1, distance);
                tmax = Math.Min(t2, tmax);
        
                if (distance > tmax)
                {
                    distance = 0f;
                    return false;
                }
            }
        
            if (Mathf.Approximately(ray.direction.z, 0))
            {
                if (ray.origin.z < box.min.z || ray.origin.z > box.max.z)
                {
                    distance = 0f;
                    return false;
                }
            }
            else
            {
                float inverse = 1.0f / ray.direction.z;
                float t1 = (box.min.z - ray.origin.z) * inverse;
                float t2 = (box.max.z - ray.origin.z) * inverse;
        
                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }
        
                distance = Math.Max(t1, distance);
                tmax = Math.Min(t2, tmax);
        
                if (distance > tmax)
                {
                    distance = 0f;
                    return false;
                }
            }
        
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="Plane"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="box">The box to test.</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.Zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool RayIntersectsBox(ref Ray ray, ref BoundingBox box, out Vector3 point)
        {
            float distance;
            if (!RayIntersectsBox(ref ray, ref box, out distance))
            {
                point = Vector3.zero;
                return false;
            }
        
            point = ray.origin + (ray.direction * distance);
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="distance">When the method completes, contains the distance of the intersection,
        /// or 0 if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool RayIntersectsSphere(ref Ray ray, ref BoundingSphere sphere, out float distance)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 177
        
            Vector3 m = ray.origin - sphere.center;
        
            float b = Vector3.Dot(m, ray.direction);
            float c = Vector3.Dot(m, m) - (sphere.radius * sphere.radius);
        
            if (c > 0f && b > 0f)
            {
                distance = 0f;
                return false;
            }
        
            float discriminant = b * b - c;
        
            if (discriminant < 0f)
            {
                distance = 0f;
                return false;
            }
        
            distance = -b - Mathf.Sqrt(discriminant);
        
            if (distance < 0f)
                distance = 0f;
        
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="BoundingSphere"/>. 
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.Zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool RayIntersectsSphere(ref Ray ray, ref BoundingSphere sphere, out Vector3 point)
        {
            float distance;
            if (!RayIntersectsSphere(ref ray, ref sphere, out distance))
            {
                point = Vector3.zero;
                return false;
            }
        
            point = ray.origin + (ray.direction * distance);
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a point.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static PlaneIntersectionType PlaneIntersectsPoint(ref Plane plane, ref Vector3 point)
        {
            float distance = Vector3.Dot(plane.normal, point);
            distance += plane.distance;
        
            if (distance > 0f)
                return PlaneIntersectionType.Front;
        
            if (distance < 0f)
                return PlaneIntersectionType.Back;
        
            return PlaneIntersectionType.Intersecting;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a <see cref="Plane"/>.
        /// </summary>
        /// <param name="plane1">The first plane to test.</param>
        /// <param name="plane2">The second plane to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool PlaneIntersectsPlane(ref Plane plane1, ref Plane plane2)
        {
            Vector3 direction = Vector3.Cross(plane1.normal, plane2.normal);
        
            //If direction is the zero vector, the planes are parallel and possibly
            //coincident. It is not an intersection. The dot product will tell us.
            float denominator = Vector3.Dot(direction, direction);
        
            if (Mathf.Approximately(denominator, 0))
                return false;
        
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a <see cref="Plane"/>.
        /// </summary>
        /// <param name="plane1">The first plane to test.</param>
        /// <param name="plane2">The second plane to test.</param>
        /// <param name="line">When the method completes, contains the line of intersection
        /// as a <see cref="Ray"/>, or a zero ray if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        /// <remarks>
        /// Although a ray is set to have an origin, the ray returned by this method is really
        /// a line in three dimensions which has no real origin. The ray is considered valid when
        /// both the positive direction is used and when the negative direction is used.
        /// </remarks>
        public static bool PlaneIntersectsPlane(ref Plane plane1, ref Plane plane2, out Ray line)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 207
        
            Vector3 direction = Vector3.Cross(plane1.normal, plane2.normal);
        
            //If direction is the zero vector, the planes are parallel and possibly
            //coincident. It is not an intersection. The dot product will tell us.
            float denominator = Vector3.Dot(direction, direction);
        
            //We assume the planes are normalized, therefore the denominator
            //only serves as a parallel and coincident check. Otherwise we need
            //to divide the point by the denominator.
            if (Mathf.Approximately(denominator, 0))
            {
                line = new Ray();
                return false;
            }
        
            Vector3 temp = plane1.distance * plane2.normal - plane2.distance * plane1.normal;
            Vector3 point = Vector3.Cross(temp, direction);

            line = new Ray(point, direction.normalized);
        
            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a triangle.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static PlaneIntersectionType PlaneIntersectsTriangle(ref Plane plane, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 207
        
            PlaneIntersectionType test1 = PlaneIntersectsPoint(ref plane, ref vertex1);
            PlaneIntersectionType test2 = PlaneIntersectsPoint(ref plane, ref vertex2);
            PlaneIntersectionType test3 = PlaneIntersectsPoint(ref plane, ref vertex3);
        
            if (test1 == PlaneIntersectionType.Front && test2 == PlaneIntersectionType.Front && test3 == PlaneIntersectionType.Front)
                return PlaneIntersectionType.Front;
        
            if (test1 == PlaneIntersectionType.Back && test2 == PlaneIntersectionType.Back && test3 == PlaneIntersectionType.Back)
                return PlaneIntersectionType.Back;
        
            return PlaneIntersectionType.Intersecting;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="box">The box to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static PlaneIntersectionType PlaneIntersectsBox(ref Plane plane, ref BoundingBox box)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 161
        
            Vector3 min;
            Vector3 max;
        
            max.x = (plane.normal.x >= 0.0f) ? box.min.x : box.max.x;
            max.y = (plane.normal.y >= 0.0f) ? box.min.y : box.max.y;
            max.z = (plane.normal.z >= 0.0f) ? box.min.z : box.max.z;
            min.x = (plane.normal.x >= 0.0f) ? box.max.x : box.min.x;
            min.y = (plane.normal.y >= 0.0f) ? box.max.y : box.min.y;
            min.z = (plane.normal.z >= 0.0f) ? box.max.z : box.min.z;
        
            float distance = Vector3.Dot(plane.normal, max);
        
            if (distance + plane.distance > 0.0f)
                return PlaneIntersectionType.Front;
        
            distance = Vector3.Dot(plane.normal, min);
        
            if (distance + plane.distance < 0.0f)
                return PlaneIntersectionType.Back;
        
            return PlaneIntersectionType.Intersecting;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Plane"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="plane">The plane to test.</param>
        /// <param name="sphere">The sphere to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static PlaneIntersectionType PlaneIntersectsSphere(ref Plane plane, ref BoundingSphere sphere)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 160
        
            float distance = Vector3.Dot(plane.normal, sphere.center);
            distance += plane.distance;
        
            if (distance > sphere.radius)
                return PlaneIntersectionType.Front;
        
            if (distance < -sphere.radius)
                return PlaneIntersectionType.Back;
        
            return PlaneIntersectionType.Intersecting;
        }

        /* This implementation is wrong
        /// <summary>
        /// Determines whether there is an intersection between a <see cref="SharpDX.BoundingBox"/> and a triangle.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool BoxIntersectsTriangle(ref BoundingBox box, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3)
        {
            if (BoxContainsPoint(ref box, ref vertex1) == ContainmentType.Contains)
                return true;

            if (BoxContainsPoint(ref box, ref vertex2) == ContainmentType.Contains)
                return true;

            if (BoxContainsPoint(ref box, ref vertex3) == ContainmentType.Contains)
                return true;

            return false;
        }
        */

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="BoundingBox"/> and a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="box1">The first box to test.</param>
        /// <param name="box2">The second box to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool BoxIntersectsBox(ref BoundingBox box1, ref BoundingBox box2)
        {
            if (box1.min.x > box2.max.x || box2.min.x > box1.max.x)
                return false;

            if (box1.min.y > box2.max.y || box2.min.y > box1.max.y)
                return false;

            if (box1.min.z > box2.max.z || box2.min.z > box1.max.z)
                return false;

            return true;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="BoundingBox"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="sphere">The sphere to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool BoxIntersectsSphere(ref BoundingBox box, ref BoundingSphere sphere)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 166

            Vector3 vector = sphere.center.Clamp(box.min, box.max);
            float distance = Vector3.SqrMagnitude(sphere.center - vector);

            return distance <= sphere.radius * sphere.radius;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="BoundingSphere"/> and a triangle.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool SphereIntersectsTriangle(ref BoundingSphere sphere, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3)
        {
            //Source: Real-Time Collision Detection by Christer Ericson
            //Reference: Page 167

            Vector3 point;
            ClosestPointPointTriangle(ref sphere.center, ref vertex1, ref vertex2, ref vertex3, out point);
            Vector3 v = point - sphere.center;

            float dot = Vector3.Dot(v, v);

            return dot <= sphere.radius * sphere.radius;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="BoundingSphere"/> and a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="sphere1">First sphere to test.</param>
        /// <param name="sphere2">Second sphere to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public static bool SphereIntersectsSphere(ref BoundingSphere sphere1, ref BoundingSphere sphere2)
        {
            float radiisum = sphere1.radius + sphere2.radius;
            return Vector3.SqrMagnitude(sphere1.center - sphere2.center) <= radiisum * radiisum;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingBox"/> contains a point.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType BoxContainsPoint(ref BoundingBox box, ref Vector3 point)
        {
            if (box.min.x <= point.x && box.max.x >= point.x &&
                box.min.y <= point.y && box.max.y >= point.y &&
                box.min.z <= point.z && box.max.z >= point.z)
            {
                return ContainmentType.Contains;
            }

            return ContainmentType.Disjoint;
        }

        /* This implementation is wrong
        /// <summary>
        /// Determines whether a <see cref="SharpDX.BoundingBox"/> contains a triangle.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType BoxContainsTriangle(ref BoundingBox box, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3)
        {
            ContainmentType test1 = BoxContainsPoint(ref box, ref vertex1);
            ContainmentType test2 = BoxContainsPoint(ref box, ref vertex2);
            ContainmentType test3 = BoxContainsPoint(ref box, ref vertex3);

            if (test1 == ContainmentType.Contains && test2 == ContainmentType.Contains && test3 == ContainmentType.Contains)
                return ContainmentType.Contains;

            if (test1 == ContainmentType.Contains || test2 == ContainmentType.Contains || test3 == ContainmentType.Contains)
                return ContainmentType.Intersects;

            return ContainmentType.Disjoint;
        }
        */

        /// <summary>
        /// Determines whether a <see cref="BoundingBox"/> contains a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="box1">The first box to test.</param>
        /// <param name="box2">The second box to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType BoxContainsBox(ref BoundingBox box1, ref BoundingBox box2)
        {
            if (box1.max.x < box2.min.x || box1.min.x > box2.max.x)
                return ContainmentType.Disjoint;

            if (box1.max.y < box2.min.y || box1.min.y > box2.max.y)
                return ContainmentType.Disjoint;

            if (box1.max.z < box2.min.z || box1.min.z > box2.max.z)
                return ContainmentType.Disjoint;

            if (box1.min.x <= box2.min.x && (box2.max.x <= box1.max.x &&
                box1.min.y <= box2.min.y && box2.max.y <= box1.max.y) &&
                box1.min.z <= box2.min.z && box2.max.z <= box1.max.z)
            {
                return ContainmentType.Contains;
            }

            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingBox"/> contains a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="box">The box to test.</param>
        /// <param name="sphere">The sphere to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType BoxContainsSphere(ref BoundingBox box, ref BoundingSphere sphere)
        {
            Vector3 vector = sphere.center.Clamp(box.min, box.max);
            float distance = Vector3.SqrMagnitude(sphere.center - vector);

            if (distance > sphere.radius * sphere.radius)
                return ContainmentType.Disjoint;

            if ((((box.min.x + sphere.radius <= sphere.center.x) && (sphere.center.x <= box.max.x - sphere.radius)) && ((box.max.x - box.min.x > sphere.radius) &&
                (box.min.y + sphere.radius <= sphere.center.y))) && (((sphere.center.y <= box.max.y - sphere.radius) && (box.max.y - box.min.y > sphere.radius)) &&
                (((box.min.z + sphere.radius <= sphere.center.z) && (sphere.center.z <= box.max.z - sphere.radius)) && (box.max.z - box.min.z > sphere.radius))))
            {
                return ContainmentType.Contains;
            }

            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingSphere"/> contains a point.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="point">The point to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType SphereContainsPoint(ref BoundingSphere sphere, ref Vector3 point)
        {
            if (Vector3.SqrMagnitude(point - sphere.center) <= sphere.radius * sphere.radius)
                return ContainmentType.Contains;

            return ContainmentType.Disjoint;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingSphere"/> contains a triangle.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="vertex1">The first vertex of the triangle to test.</param>
        /// <param name="vertex2">The second vertex of the triangle to test.</param>
        /// <param name="vertex3">The third vertex of the triangle to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType SphereContainsTriangle(ref BoundingSphere sphere, ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3)
        {
            //Source: Jorgy343
            //Reference: None

            ContainmentType test1 = SphereContainsPoint(ref sphere, ref vertex1);
            ContainmentType test2 = SphereContainsPoint(ref sphere, ref vertex2);
            ContainmentType test3 = SphereContainsPoint(ref sphere, ref vertex3);

            if (test1 == ContainmentType.Contains && test2 == ContainmentType.Contains && test3 == ContainmentType.Contains)
                return ContainmentType.Contains;

            if (SphereIntersectsTriangle(ref sphere, ref vertex1, ref vertex2, ref vertex3))
                return ContainmentType.Intersects;

            return ContainmentType.Disjoint;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingSphere"/> contains a <see cref="BoundingBox"/>.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="box">The box to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType SphereContainsBox(ref BoundingSphere sphere, ref BoundingBox box)
        {
            Vector3 vector;

            if (!BoxIntersectsSphere(ref box, ref sphere))
                return ContainmentType.Disjoint;

            float radiussquared = sphere.radius * sphere.radius;
            vector.x = sphere.center.x - box.min.x;
            vector.y = sphere.center.y - box.max.y;
            vector.z = sphere.center.z - box.max.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.max.x;
            vector.y = sphere.center.y - box.max.y;
            vector.z = sphere.center.z - box.max.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.max.x;
            vector.y = sphere.center.y - box.min.y;
            vector.z = sphere.center.z - box.max.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.min.x;
            vector.y = sphere.center.y - box.min.y;
            vector.z = sphere.center.z - box.max.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.min.x;
            vector.y = sphere.center.y - box.max.y;
            vector.z = sphere.center.z - box.min.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.max.x;
            vector.y = sphere.center.y - box.max.y;
            vector.z = sphere.center.z - box.min.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.max.x;
            vector.y = sphere.center.y - box.min.y;
            vector.z = sphere.center.z - box.min.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            vector.x = sphere.center.x - box.min.x;
            vector.y = sphere.center.y - box.min.y;
            vector.z = sphere.center.z - box.min.z;

            if (vector.sqrMagnitude > radiussquared)
                return ContainmentType.Intersects;

            return ContainmentType.Contains;
        }

        /// <summary>
        /// Determines whether a <see cref="BoundingSphere"/> contains a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="sphere1">The first sphere to test.</param>
        /// <param name="sphere2">The second sphere to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public static ContainmentType SphereContainsSphere(ref BoundingSphere sphere1, ref BoundingSphere sphere2)
        {
            float distance = Vector3.Distance(sphere1.center, sphere2.center);

            if (sphere1.radius + sphere2.radius < distance)
                return ContainmentType.Disjoint;

            if (sphere1.radius - sphere2.radius < distance)
                return ContainmentType.Intersects;

            return ContainmentType.Contains;
        }
    }
}
