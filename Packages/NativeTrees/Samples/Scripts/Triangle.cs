using System;
using NativeTrees;
using Unity.Mathematics;
using UnityEngine;

namespace NativeTrees.Samples
{
    /// <summary>
    /// Represents a triangle in 3D space that supports raycasting
    /// </summary>
    public struct Triangle : IEquatable<Triangle>
    {
        public float3 a;
        public float3 b;
        public float3 c;
        public int id;
            
        public float3 Centroid => (a + b + c) / 3;
        public float3 Normal => math.normalize(math.cross(b - a, c - a));

        public Triangle(Vector3 a, Vector3 b, Vector3 c, int id)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.id = id;
        }
        
        public AABB GetAABB() => new AABB(math.min(math.min(a, b), c), math.max(math.max(a, b), c));

        /// <summary>
        /// Checks if the specified ray hits the triangle.
        /// Möller–Trumbore ray-triangle intersection algorithm implementation.
        /// </summary>
        /// <param name="rayOrigin">origin of the ray</param>
        /// <param name="rayDirection">direction of the ray</param>
        /// <param name="distance">How for along the ray the hit occured</param>
        /// <returns><c>true</c> when the ray hits the triangle, otherwise <c>false</c></returns>
        public bool IntersectsRay(float3 rayOrigin, float3 rayDirection, out float distance)
        {
            // Vectors from p1 to p2/p3 (edges)
            float3 e1, e2;  
 
            float3 p, q, t;
            float det, invDet, u, v;

            //Find vectors for two edges sharing vertex/point p1
            e1 = b - a;
            e2 = c - a;
 
            // calculating determinant 
            p = math.cross(rayDirection, e2);
 
            //Calculate determinat
            det = math.dot(e1, p);
 
            //if determinant is near zero, ray lies in plane of triangle otherwise not
            if (det > -float.Epsilon && det < float.Epsilon)
            {
                distance = 0;
                return false;
            }
            invDet = 1.0f / det;
 
            //calculate distance from p1 to ray origin
            t = rayOrigin - a;
 
            //Calculate u parameter
            u = math.dot(t, p) * invDet;
 
            //Check for ray hit
            if (u < 0 || u > 1)
            {
                distance = 0;
                return false;
            }
 
            //Prepare to test v parameter
            q = math.cross(t, e1);
 
            //Calculate v parameter
            v =  math.dot(rayDirection, q) * invDet;
 
            //Check for ray hit
            if (v < 0 || u + v > 1)
            {
                distance = 0;
                return false;
            }

            distance = math.dot(e2, q) * invDet;
            return distance > float.Epsilon;
        }

        public bool Equals(Triangle other)
        {
            return id == other.id;
        }

        public override bool Equals(object obj)
        {
            return obj is Triangle other && Equals(other);
        }

        public override int GetHashCode()
        {
            return id;
        }
    }
}