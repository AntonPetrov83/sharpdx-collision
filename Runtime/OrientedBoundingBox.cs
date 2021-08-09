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

using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace SharpDX
{
    /// <summary>
    /// OrientedBoundingBox (OBB) is a rectangular block, much like an AABB (BoundingBox) but with an arbitrary orientation.
    /// </summary>
    public struct OrientedBoundingBox : IEquatable<OrientedBoundingBox>, IFormattable
    {
        /// <summary>
        /// Half lengths of the box along each axis.
        /// </summary>
        public Vector3 extents;

        /// <summary>
        /// The matrix which aligns and scales the box, and its translation vector represents the center of the box.
        /// </summary>
        public Matrix4x4 transformation;

        /// <summary>
        /// The size of the <see cref="OrientedBoundingBox"/> if no scaling is applied to the transformation matrix.
        /// </summary>
        /// <remarks>
        /// The property will return the actual size even if the scaling is applied using Scale method, 
        /// but if the scaling is applied to transformation matrix, use GetSize Function instead.
        /// </remarks>
        public Vector3 size
        {
            get
            {
                return extents * 2;
            }
        }
        
        /// <summary>
        /// Returns the center of the <see cref="OrientedBoundingBox"/>.
        /// </summary>
        public Vector3 center
        {
            get
            {
                return transformation.GetT();
            }
        }
        

        /// <summary>
        /// Creates an <see cref="OrientedBoundingBox"/> from a BoundingBox.
        /// </summary>
        /// <param name="bb">The BoundingBox to create from.</param>
        /// <remarks>
        /// Initially, the OBB is axis-aligned box, but it can be rotated and transformed later.
        /// </remarks>
        public OrientedBoundingBox(BoundingBox bb)
        {
            var Center = bb.min + (bb.max - bb.min) / 2f;
            extents = bb.max - Center;
            transformation = Matrix4x4.Translate(Center);
        }
        
        /// <summary>
        /// Creates an <see cref="OrientedBoundingBox"/> from Bounds.
        /// </summary>
        /// <param name="bounds">Bounds to create from.</param>
        /// <remarks>
        /// Initially, the OBB is axis-aligned box, but it can be rotated and transformed later.
        /// </remarks>
        public OrientedBoundingBox(Bounds bounds)
        {
            extents = bounds.extents;
            transformation = Matrix4x4.Translate(bounds.center);
        }

        /// <summary>
        /// Creates an <see cref="OrientedBoundingBox"/> which contained between two minimum and maximum points.
        /// </summary>
        /// <param name="minimum">The minimum vertex of the bounding box.</param>
        /// <param name="maximum">The maximum vertex of the bounding box.</param>
        /// <remarks>
        /// Initially, the OrientedBoundingBox is axis-aligned box, but it can be rotated and transformed later.
        /// </remarks>
        public OrientedBoundingBox(Vector3 minimum, Vector3 maximum)
        {
            var Center = minimum + (maximum - minimum) / 2f;
            extents = maximum - Center;
            transformation = Matrix4x4.Translate(Center);
        }

        /// <summary>
        /// Creates an <see cref="OrientedBoundingBox"/> that fully contains the given points.
        /// </summary>
        /// <param name="points">The points that will be contained by the box.</param>
        /// <remarks>
        /// This method is not for computing the best tight-fitting OrientedBoundingBox.
        /// And initially, the OrientedBoundingBox is axis-aligned box, but it can be rotated and transformed later.
        /// </remarks>
        public OrientedBoundingBox(Vector3[] points)
        {
            if (points == null || points.Length == 0)
                throw new ArgumentNullException("points");

            Vector3 minimum = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 maximum = new Vector3(float.MinValue, float.MinValue, float.MinValue);

            for (int i = 0; i < points.Length; ++i)
            {
                minimum = Vector3.Min(minimum, points[i]);
                maximum = Vector3.Max(maximum, points[i]);
            }

            var Center = minimum + (maximum - minimum) / 2f;
            extents = maximum - Center;
            transformation = Matrix4x4.Translate(Center);
        }

        /// <summary>
        /// Retrieves the eight corners of the bounding box.
        /// </summary>
        /// <returns>An array of points representing the eight corners of the bounding box.</returns>
        public Vector3[] GetCorners()
        {
            var xv = new Vector3(extents.x, 0, 0);
            var yv = new Vector3(0, extents.y, 0);
            var zv = new Vector3(0, 0, extents.z);

            xv = transformation.MultiplyVector(xv);
            yv = transformation.MultiplyVector(yv);
            zv = transformation.MultiplyVector(zv);
            
            var center = transformation.GetT();

            var corners = new Vector3[8];
            corners[0] = center + xv + yv + zv;
            corners[1] = center + xv + yv - zv;
            corners[2] = center - xv + yv - zv;
            corners[3] = center - xv + yv + zv;
            corners[4] = center + xv - yv + zv;
            corners[5] = center + xv - yv - zv;
            corners[6] = center - xv - yv - zv;
            corners[7] = center - xv - yv + zv;

            return corners;
        }

        /// <summary>
        /// Transforms this box using a transformation matrix.
        /// </summary>
        /// <param name="mat">The transformation matrix.</param>
        /// <remarks>
        /// While any kind of transformation can be applied, it is recommended to apply scaling using scale method instead, which
        /// scales the Extents and keeps the Transformation matrix for rotation only, and that preserves collision detection accuracy.
        /// </remarks>
        public void Transform(Matrix4x4 mat)
        {
            transformation *= mat;
        }

        /// <summary>
        /// Scales the <see cref="OrientedBoundingBox"/> by scaling its Extents without affecting the Transformation matrix,
        /// By keeping Transformation matrix scaling-free, the collision detection methods will be more accurate.
        /// </summary>
        /// <param name="scaling"></param>
        public void Scale(Vector3 scaling)
        {
            extents.Scale(scaling);
        }

        /// <summary>
        /// Scales the <see cref="OrientedBoundingBox"/> by scaling its Extents without affecting the Transformation matrix,
        /// By keeping Transformation matrix scaling-free, the collision detection methods will be more accurate.
        /// </summary>
        /// <param name="scaling"></param>
        public void Scale(float scaling)
        {
            extents *= scaling;
        }

        /// <summary>
        /// Translates the <see cref="OrientedBoundingBox"/> to a new position using a translation vector;
        /// </summary>
        /// <param name="translation">the translation vector.</param>
        public void Translate(Vector3 translation)
        {
            transformation.SetT(transformation.GetT() + translation);
        }

        /// <summary>
        /// Returns the size of the <see cref="OrientedBoundingBox"/> taking into consideration the scaling applied to the transformation matrix.
        /// </summary>
        /// <returns>The size of the consideration</returns>
        /// <remarks>
        /// This method is computationally expensive, so if no scale is applied to the transformation matrix
        /// use <see cref="size"/> property instead.
        /// </remarks>
        public Vector3 GetSize()
        {
            var xv = new Vector3(extents.x * 2, 0, 0);
            var yv = new Vector3(0, extents.y * 2, 0);
            var zv = new Vector3(0, 0, extents.z * 2);

            xv = transformation.MultiplyVector(xv);
            yv = transformation.MultiplyVector(yv);
            zv = transformation.MultiplyVector(zv);            

            return new Vector3(xv.magnitude, yv.magnitude, zv.magnitude);
        }

        /// <summary>
        /// Returns the square size of the <see cref="OrientedBoundingBox"/> taking into consideration the scaling applied to the transformation matrix.
        /// </summary>
        /// <returns>The size of the consideration</returns>
        public Vector3 GetSizeSquared()
        {
            var xv = new Vector3(extents.x * 2, 0, 0);
            var yv = new Vector3(0, extents.y * 2, 0);
            var zv = new Vector3(0, 0, extents.z * 2);

            xv = transformation.MultiplyVector(xv);
            yv = transformation.MultiplyVector(yv);
            zv = transformation.MultiplyVector(zv);            

            return new Vector3(xv.sqrMagnitude, yv.sqrMagnitude, zv.sqrMagnitude);
        }

        /// <summary>
        /// Determines whether a <see cref="OrientedBoundingBox"/> contains a point. 
        /// </summary>
        /// <param name="point">The point to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        public ContainmentType Contains(Vector3 point)
        {
            // Transform the point into the obb coordinates
            Matrix4x4 invTrans = transformation.inverse;

            Vector3 locPoint = invTrans.MultiplyPoint(point);

            locPoint.x = Math.Abs(locPoint.x);
            locPoint.y = Math.Abs(locPoint.y);
            locPoint.z = Math.Abs(locPoint.z);

            //Simple axes-aligned BB check
            if (Mathf.Approximately(locPoint.x, extents.x) && Mathf.Approximately(locPoint.y, extents.y) && Mathf.Approximately(locPoint.z, extents.z))
                return ContainmentType.Intersects;
            if (locPoint.x < extents.x && locPoint.y < extents.y && locPoint.z < extents.z)
                return ContainmentType.Contains;
            else
                return ContainmentType.Disjoint;
        }

        /// <summary>
        /// Determines whether a <see cref="OrientedBoundingBox"/> contains an array of points>.
        /// </summary>
        /// <param name="points">The points array to test.</param>
        /// <returns>The type of containment.</returns>
        public ContainmentType Contains(Vector3[] points)
        {
            Matrix4x4 invTrans = transformation.inverse;

            var containsAll = true;
            var containsAny = false;

            for (int i = 0; i < points.Length; i++)
            {
                Vector3 locPoint = invTrans.MultiplyPoint(points[i]);

                locPoint.x = Math.Abs(locPoint.x);
                locPoint.y = Math.Abs(locPoint.y);
                locPoint.z = Math.Abs(locPoint.z);

                //Simple axes-aligned BB check
                if (Mathf.Approximately(locPoint.x, extents.x) &&
                    Mathf.Approximately(locPoint.y, extents.y) &&
                    Mathf.Approximately(locPoint.z, extents.z))
                    containsAny = true;
                if (locPoint.x < extents.x && locPoint.y < extents.y && locPoint.z < extents.z)
                    containsAny = true;
                else
                    containsAll = false;
            }

            if (containsAll)
                return ContainmentType.Contains;
            else if (containsAny)
                return ContainmentType.Intersects;
            else
                return ContainmentType.Disjoint;
        }

        /// <summary>
        /// Determines whether a <see cref="OrientedBoundingBox"/> contains a <see cref="BoundingSphere"/>.
        /// </summary>
        /// <param name="sphere">The sphere to test.</param>
        /// <param name="ignoreScale">Optimize the check operation by assuming that <see cref="OrientedBoundingBox"/> has no scaling applied</param>
        /// <returns>The type of containment the two objects have.</returns>
        /// <remarks>
        /// This method is not designed for <see cref="OrientedBoundingBox"/> which has a non-uniform scaling applied to its transformation matrix.
        /// But any type of scaling applied using Scale method will keep this method accurate.
        /// </remarks>
        public ContainmentType Contains(BoundingSphere sphere, bool ignoreScale = false)
        {
            Matrix4x4 invTrans = transformation.inverse;

            // Transform sphere center into the obb coordinates
            Vector3 locCenter = invTrans.MultiplyPoint(sphere.center);

            float locRadius;
            if (ignoreScale)
            {
                locRadius = sphere.radius;
            }
            else
            {
                // Transform sphere radius into the obb coordinates
                Vector3 vRadius = Vector3.right * sphere.radius;
                vRadius = invTrans.MultiplyVector(vRadius);
                locRadius = vRadius.magnitude;
            }

            //Perform regular BoundingBox to BoundingSphere containment check
            Vector3 minusExtens = -extents;
            Vector3 vector = locCenter.Clamp(minusExtens, extents);
            float distance = Vector3.SqrMagnitude(locCenter - vector);

            if (distance > locRadius * locRadius)
                return ContainmentType.Disjoint;

            if ((((minusExtens.x + locRadius <= locCenter.x) && (locCenter.x <= extents.x - locRadius)) && ((extents.x - minusExtens.x > locRadius) &&
                (minusExtens.y + locRadius <= locCenter.y))) && (((locCenter.y <= extents.y - locRadius) && (extents.y - minusExtens.y > locRadius)) &&
                (((minusExtens.z + locRadius <= locCenter.z) && (locCenter.z <= extents.z - locRadius)) && (extents.z - minusExtens.z > locRadius))))
            {
                return ContainmentType.Contains;
            }

            return ContainmentType.Intersects;
        }

        private static Vector3[] GetRows(ref Matrix4x4 mat)
        {
            return new Vector3[] {
                new Vector3(mat.m00,mat.m01,mat.m02),
                new Vector3(mat.m10,mat.m11,mat.m12),
                new Vector3(mat.m20,mat.m21,mat.m22)
            };
        }

        /// <summary>
        /// Check the intersection between two <see cref="OrientedBoundingBox"/>
        /// </summary>
        /// <param name="obb">The OrientedBoundingBoxs to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        /// <remarks>
        /// For accuracy, The transformation matrix for both <see cref="OrientedBoundingBox"/> must not have any scaling applied to it.
        /// Anyway, scaling using Scale method will keep this method accurate.
        /// </remarks>
        public ContainmentType Contains(OrientedBoundingBox obb)
        {
            var cornersCheck = Contains(obb.GetCorners());
            if (cornersCheck != ContainmentType.Disjoint)
                return cornersCheck;

            //http://www.3dkingdoms.com/weekly/bbox.cpp
            var SizeA = extents;
            var SizeB = obb.extents;
            var RotA = GetRows(ref transformation);
            var RotB = GetRows(ref obb.transformation);

            var R = new Matrix4x4();       // Rotation from B to A
            var AR = new Matrix4x4();      // absolute values of R matrix, to use with box extents

            float ExtentA, ExtentB, Separation;
            int i, k;

            // Calculate B to A rotation matrix
            for (i = 0; i < 3; i++)
                for (k = 0; k < 3; k++)
                {
                    R[i, k] = Vector3.Dot(RotA[i], RotB[k]);
                    AR[i, k] = Math.Abs(R[i, k]);
                }


            // Vector separating the centers of Box B and of Box A	
            var vSepWS = obb.center - center;
            // Rotated into Box A's coordinates
            var vSepA = new Vector3(Vector3.Dot(vSepWS, RotA[0]), Vector3.Dot(vSepWS, RotA[1]), Vector3.Dot(vSepWS, RotA[2]));

            // Test if any of A's basis vectors separate the box
            for (i = 0; i < 3; i++)
            {
                ExtentA = SizeA[i];
                ExtentB = Vector3.Dot(SizeB, new Vector3(AR[i, 0], AR[i, 1], AR[i, 2]));
                Separation = Math.Abs(vSepA[i]);

                if (Separation > ExtentA + ExtentB)
                    return ContainmentType.Disjoint;
            }

            // Test if any of B's basis vectors separate the box
            for (k = 0; k < 3; k++)
            {
                ExtentA = Vector3.Dot(SizeA, new Vector3(AR[0, k], AR[1, k], AR[2, k]));
                ExtentB = SizeB[k];
                Separation = Math.Abs(Vector3.Dot(vSepA, new Vector3(R[0, k], R[1, k], R[2, k])));

                if (Separation > ExtentA + ExtentB)
                    return ContainmentType.Disjoint;
            }

            // Now test Cross Products of each basis vector combination ( A[i], B[k] )
            for (i = 0; i < 3; i++)
                for (k = 0; k < 3; k++)
                {
                    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
                    int k1 = (k + 1) % 3, k2 = (k + 2) % 3;
                    ExtentA = SizeA[i1] * AR[i2, k] + SizeA[i2] * AR[i1, k];
                    ExtentB = SizeB[k1] * AR[i, k2] + SizeB[k2] * AR[i, k1];
                    Separation = Math.Abs(vSepA[i2] * R[i1, k] - vSepA[i1] * R[i2, k]);
                    if (Separation > ExtentA + ExtentB)
                        return ContainmentType.Disjoint;
                }

            // No separating axis found, the boxes overlap	
            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Check the intersection between an <see cref="OrientedBoundingBox"/> and a line defined by two points
        /// </summary>
        /// <param name="L1">The first point in the line.</param>
        /// <param name="L2">The second point in the line.</param>
        /// <returns>The type of containment the two objects have.</returns>
        /// <remarks>
        /// For accuracy, The transformation matrix for the <see cref="OrientedBoundingBox"/> must not have any scaling applied to it.
        /// Anyway, scaling using Scale method will keep this method accurate.
        /// </remarks>
        public ContainmentType ContainsLine(Vector3 L1, Vector3 L2)
        {
            var cornersCheck = Contains(new Vector3[] { L1, L2 });
            if (cornersCheck != ContainmentType.Disjoint)
                return cornersCheck;

            //http://www.3dkingdoms.com/weekly/bbox.cpp
            // Put line in box space
            Matrix4x4 invTrans = transformation.inverse;

            Vector3 LB1 = invTrans.MultiplyPoint(L1);
            Vector3 LB2 = invTrans.MultiplyPoint(L2);

            // Get line midpoint and extent
            var LMid = (LB1 + LB2) * 0.5f;
            var L = (LB1 - LMid);
            var LExt = new Vector3(Math.Abs(L.x), Math.Abs(L.y), Math.Abs(L.z));

            // Use Separating Axis Test
            // Separation vector from box center to line center is LMid, since the line is in box space
            if (Math.Abs(LMid.x) > extents.x + LExt.x) return ContainmentType.Disjoint;
            if (Math.Abs(LMid.y) > extents.y + LExt.y) return ContainmentType.Disjoint;
            if (Math.Abs(LMid.z) > extents.z + LExt.z) return ContainmentType.Disjoint;
            // Cross products of line and each axis
            if (Math.Abs(LMid.y * L.z - LMid.z * L.y) > (extents.y * LExt.z + extents.z * LExt.y)) return ContainmentType.Disjoint;
            if (Math.Abs(LMid.x * L.z - LMid.z * L.x) > (extents.x * LExt.z + extents.z * LExt.x)) return ContainmentType.Disjoint;
            if (Math.Abs(LMid.x * L.y - LMid.y * L.x) > (extents.x * LExt.y + extents.y * LExt.x)) return ContainmentType.Disjoint;
            // No separating axis, the line intersects
            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Check the intersection between an <see cref="OrientedBoundingBox"/> and <see cref="BoundingBox"/>
        /// </summary>
        /// <param name="box">The BoundingBox to test.</param>
        /// <returns>The type of containment the two objects have.</returns>
        /// <remarks>
        /// For accuracy, The transformation matrix for the <see cref="OrientedBoundingBox"/> must not have any scaling applied to it.
        /// Anyway, scaling using Scale method will keep this method accurate.
        /// </remarks>
        public ContainmentType Contains(BoundingBox box)
        {
            var cornersCheck = Contains(box.GetCorners());
            if (cornersCheck != ContainmentType.Disjoint)
                return cornersCheck;

            var boxCenter = box.min + (box.max - box.min) / 2f;
            var boxExtents = box.max - boxCenter;

            var sizeA = extents;
            var sizeB = boxExtents;
            var rotA = GetRows(ref transformation);

            float ExtentA, ExtentB, Separation;
            int i, k;

            Matrix4x4 R = transformation.inverse;   // Rotation from B to A
            var AR = new Matrix4x4();               // absolute values of R matrix, to use with box extents

            for (i = 0; i < 3; i++)
                for (k = 0; k < 3; k++)
                    AR[i, k] = Math.Abs(R[i, k]);

            // Vector separating the centers of Box B and of Box A	
            var vSepWS = boxCenter - center;
            // Rotated into Box A's coordinates
            var vSepA = new Vector3(Vector3.Dot(vSepWS, rotA[0]), Vector3.Dot(vSepWS, rotA[1]), Vector3.Dot(vSepWS, rotA[2]));

            // Test if any of A's basis vectors separate the box
            for (i = 0; i < 3; i++)
            {
                ExtentA = sizeA[i];
                ExtentB = Vector3.Dot(sizeB, new Vector3(AR[i, 0], AR[i, 1], AR[i, 2]));
                Separation = Math.Abs(vSepA[i]);

                if (Separation > ExtentA + ExtentB)
                    return ContainmentType.Disjoint;
            }

            // Test if any of B's basis vectors separate the box
            for (k = 0; k < 3; k++)
            {
                ExtentA = Vector3.Dot(sizeA, new Vector3(AR[0, k], AR[1, k], AR[2, k]));
                ExtentB = sizeB[k];
                Separation = Math.Abs(Vector3.Dot(vSepA, new Vector3(R[0, k], R[1, k], R[2, k])));

                if (Separation > ExtentA + ExtentB)
                    return ContainmentType.Disjoint;
            }

            // Now test Cross Products of each basis vector combination ( A[i], B[k] )
            for (i = 0; i < 3; i++)
                for (k = 0; k < 3; k++)
                {
                    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
                    int k1 = (k + 1) % 3, k2 = (k + 2) % 3;
                    ExtentA = sizeA[i1] * AR[i2, k] + sizeA[i2] * AR[i1, k];
                    ExtentB = sizeB[k1] * AR[i, k2] + sizeB[k2] * AR[i, k1];
                    Separation = Math.Abs(vSepA[i2] * R[i1, k] - vSepA[i1] * R[i2, k]);
                    if (Separation > ExtentA + ExtentB)
                        return ContainmentType.Disjoint;
                }

            // No separating axis found, the boxes overlap	
            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="OrientedBoundingBox"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <param name="point">When the method completes, contains the point of intersection,
        /// or <see cref="Vector3.zero"/> if there was no intersection.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public bool Intersects(Ray ray, out Vector3 point)
        {
            // Put ray in box space
            Matrix4x4 invTrans = transformation.inverse;

            Ray bRay = new Ray(invTrans.MultiplyPoint(ray.origin), invTrans.MultiplyVector(ray.direction));

            //Perform a regular ray to BoundingBox check
            var bb = new BoundingBox(-extents, extents);
            var intersects = Collision.RayIntersectsBox(ref bRay, ref bb, out point);

            //Put the result intersection back to world
            if (intersects)
                point = transformation.MultiplyPoint(point);

            return intersects;
        }

        /// <summary>
        /// Determines whether there is an intersection between a <see cref="Ray"/> and a <see cref="OrientedBoundingBox"/>.
        /// </summary>
        /// <param name="ray">The ray to test.</param>
        /// <returns>Whether the two objects intersected.</returns>
        public bool Intersects(Ray ray)
        {
            return Intersects(ray, out _);
        }

        private Vector3[] GetLocalCorners()
        {
            var xv = new Vector3(extents.x, 0, 0);
            var yv = new Vector3(0, extents.y, 0);
            var zv = new Vector3(0, 0, extents.z);

            var corners = new Vector3[8];
            corners[0] = xv + yv + zv;
            corners[1] = xv + yv - zv;
            corners[2] = -xv + yv - zv;
            corners[3] = -xv + yv + zv;
            corners[4] = xv - yv + zv;
            corners[5] = xv - yv - zv;
            corners[6] = -xv - yv - zv;
            corners[7] = -xv - yv + zv;

            return corners;
        }

        /// <summary>
        /// Get the axis-aligned <see cref="BoundingBox"/> which contains all <see cref="OrientedBoundingBox"/> corners.
        /// </summary>
        /// <returns>The axis-aligned BoundingBox of this OrientedBoundingBox.</returns>
        public BoundingBox GetBoundingBox()
        {
            return BoundingBox.FromPoints(GetCorners());
        }

        /// <summary>
        /// Calculates the matrix required to transfer any point from one <see cref="OrientedBoundingBox"/> local coordinates to another.
        /// </summary>
        /// <param name="A">The source OrientedBoundingBox.</param>
        /// <param name="B">The target OrientedBoundingBox.</param>
        /// <param name="NoMatrixScaleApplied">
        /// If true, the method will use a fast algorithm which is inapplicable if a scale is applied to the transformation matrix of the OrientedBoundingBox.
        /// </param>
        /// <returns></returns>
        public static Matrix4x4 GetBoxToBoxMatrix(OrientedBoundingBox A, OrientedBoundingBox B, bool NoMatrixScaleApplied = false)
        {
            Matrix4x4 AtoB_Matrix;

            // Calculate B to A transformation matrix
            if (NoMatrixScaleApplied)
            {
                var RotA = GetRows(ref A.transformation);
                var RotB = GetRows(ref B.transformation);
                AtoB_Matrix = new Matrix4x4();
                int i, k;
                for (i = 0; i < 3; i++)
                    for (k = 0; k < 3; k++)
                        AtoB_Matrix[i, k] = Vector3.Dot(RotB[i], RotA[k]);
                var v = B.center - A.center;
                AtoB_Matrix.m03 = Vector3.Dot(v, RotA[0]);
                AtoB_Matrix.m13 = Vector3.Dot(v, RotA[1]);
                AtoB_Matrix.m23 = Vector3.Dot(v, RotA[2]);
                AtoB_Matrix.m33 = 1;
            }
            else
            {
                Matrix4x4 AInvMat = A.transformation.inverse;
                AtoB_Matrix = B.transformation * AInvMat;
            }

            return AtoB_Matrix;
        }

        /// <summary>
        /// Merge an OrientedBoundingBox B into another OrientedBoundingBox A, by expanding A to contain B and keeping A orientation.
        /// </summary>
        /// <param name="A">The <see cref="OrientedBoundingBox"/> to merge into it.</param>
        /// <param name="B">The <see cref="OrientedBoundingBox"/> to be merged</param>
        /// <param name="NoMatrixScaleApplied">
        /// If true, the method will use a fast algorithm which is inapplicable if a scale is applied to the transformation matrix of the OrientedBoundingBox.
        /// </param>
        /// <remarks>
        /// Unlike merging axis aligned boxes, The operation is not interchangeable, because it keeps A orientation and merge B into it.
        /// </remarks>
        public static void Merge(ref OrientedBoundingBox A, ref OrientedBoundingBox B, bool NoMatrixScaleApplied = false)
        {
            Matrix4x4 AtoB_Matrix = GetBoxToBoxMatrix(A, B, NoMatrixScaleApplied);

            //Get B corners in A Space
            var bCorners = AtoB_Matrix.MultiplyPoints(B.GetLocalCorners());

            //Get A local Bounding Box
            var A_LocalBB = new BoundingBox(-A.extents, A.extents);

            //Find B BoundingBox in A Space
            var B_LocalBB = BoundingBox.FromPoints(bCorners);

            //Merger A and B local Bounding Boxes
            BoundingBox mergedBB = BoundingBox.Merge(B_LocalBB, A_LocalBB);

            //Find the new Extents and Center, Transform Center back to world
            var newCenter = mergedBB.min + (mergedBB.max - mergedBB.min) / 2f;
            A.extents = mergedBB.max - newCenter;
            newCenter = A.transformation.MultiplyPoint(newCenter);
            A.transformation.SetT(newCenter);
        }

        /// <summary>
        /// Merge this OrientedBoundingBox into another OrientedBoundingBox, keeping the other OrientedBoundingBox orientation.
        /// </summary>
        /// <param name="OBB">The other <see cref="OrientedBoundingBox"/> to merge into.</param>
        /// <param name="NoMatrixScaleApplied">
        /// If true, the method will use a fast algorithm which is inapplicable if a scale is applied to the transformation matrix of the OrientedBoundingBox.
        /// </param>
        public void MergeInto(ref OrientedBoundingBox OBB, bool NoMatrixScaleApplied = false)
        {
            Merge(ref OBB, ref this, NoMatrixScaleApplied);
        }

        /// <summary>
        /// Merge another OrientedBoundingBox into this OrientedBoundingBox.
        /// </summary>
        /// <param name="OBB">The other <see cref="OrientedBoundingBox"/> to merge into this OrientedBoundingBox.</param>
        /// <param name="NoMatrixScaleApplied">
        /// If true, the method will use a fast algorithm which is inapplicable if a scale is applied to the transformation matrix of the OrientedBoundingBox.
        /// </param>
        public void Add(ref OrientedBoundingBox OBB, bool NoMatrixScaleApplied = false)
        {
            Merge(ref this, ref OBB, NoMatrixScaleApplied);
        }

        /// <summary>
        /// Determines whether the specified <see cref="Vector4"/> is equal to this instance.
        /// </summary>
        /// <param name="value">The <see cref="Vector4"/> to compare with this instance.</param>
        /// <returns>
        /// <c>true</c> if the specified <see cref="Vector4"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(OrientedBoundingBox value)
        {
            return extents == value.extents && transformation == value.transformation;
        }

        /// <summary>
        /// Determines whether the specified <see cref="System.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="value">The <see cref="System.Object"/> to compare with this instance.</param>
        /// <returns>
        /// <c>true</c> if the specified <see cref="System.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public override bool Equals(object value)
        {
            if (!(value is OrientedBoundingBox))
                return false;

            var strongValue = (OrientedBoundingBox)value;
            return Equals(strongValue);
        }

        /// <summary>
        /// Tests for equality between two objects.
        /// </summary>
        /// <param name="left">The first value to compare.</param>
        /// <param name="right">The second value to compare.</param>
        /// <returns><c>true</c> if <paramref name="left"/> has the same value as <paramref name="right"/>; otherwise, <c>false</c>.</returns>
        public static bool operator ==(OrientedBoundingBox left, OrientedBoundingBox right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Tests for inequality between two objects.
        /// </summary>
        /// <param name="left">The first value to compare.</param>
        /// <param name="right">The second value to compare.</param>
        /// <returns><c>true</c> if <paramref name="left"/> has a different value than <paramref name="right"/>; otherwise, <c>false</c>.</returns>
        public static bool operator !=(OrientedBoundingBox left, OrientedBoundingBox right)
        {
            return !left.Equals(right);
        }

        /// <summary>
        /// Returns a hash code for this instance.
        /// </summary>
        /// <returns>
        /// A hash code for this instance, suitable for use in hashing algorithms and data structures like a hash table. 
        /// </returns>
        public override int GetHashCode()
        {
            return extents.GetHashCode() + transformation.GetHashCode();
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public override string ToString()
        {
            return String.Format(CultureInfo.CurrentCulture, "Center: {0}, Extents: {1}", center, extents);
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public string ToString(string format)
        {
            if (format == null)
                return ToString();

            return string.Format(CultureInfo.CurrentCulture, "Center: {0}, Extents: {1}", center.ToString(format, CultureInfo.CurrentCulture),
                extents.ToString(format, CultureInfo.CurrentCulture));
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <param name="formatProvider">The format provider.</param>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public string ToString(IFormatProvider formatProvider)
        {
            return string.Format(formatProvider, "Center: {0}, Extents: {1}", center.ToString(), extents.ToString());
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="formatProvider">The format provider.</param>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public string ToString(string format, IFormatProvider formatProvider)
        {
            if (format == null)
                return ToString(formatProvider);

            return string.Format(formatProvider, "Center: {0}, Extents: {1}", center.ToString(format, formatProvider),
                extents.ToString(format, formatProvider));
        }
    }
}

