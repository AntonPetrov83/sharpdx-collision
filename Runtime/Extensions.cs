using System.CodeDom.Compiler;
using UnityEngine;

namespace SharpDX
{
    static class Extensions
    {
        public static Vector3[] MultiplyPoints(this Matrix4x4 matrix, Vector3[] points)
        {
            var result = new Vector3[points.Length];

            for (int i = 0; i < points.Length; i++)
                result[i] = matrix.MultiplyPoint(points[i]);

            return result;
        }

        public static Vector3 Clamp(this Vector3 value, Vector3 min, Vector3 max)
        {
            return new Vector3(
                Mathf.Clamp(value.x, min.x, max.x),
                Mathf.Clamp(value.y, min.y, max.y),
                Mathf.Clamp(value.z, min.z, max.z));
        }
        
        public static Vector3 GetT(this Matrix4x4 matrix)
        {
            // Matrices in Unity are column major; i.e. the position of a transformation matrix is in the last column.
            return new Vector3(matrix.m03, matrix.m13, matrix.m23);
        }
        
        public static void SetT(this Matrix4x4 matrix, Vector3 translation)
        {
            matrix.m03 = translation.x;
            matrix.m13 = translation.y;
            matrix.m23 = translation.z;
        }
    }
}