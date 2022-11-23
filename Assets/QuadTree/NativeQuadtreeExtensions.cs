using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

// https://bartvandesande.nl
// https://github.com/bartofzo

namespace NativeTrees
{
    /// <summary>
    /// Convenience queries that operate just on the object's bounding boxes
    /// </summary>
    public static class NativeQuadtreeExtensions
    {
        /// <summary>
        /// Performs a raycast on the octree just using the bounds of the objects in it
        /// </summary>
        public static bool RaycastAABB<T>(this NativeQuadtree<T> quadtree, Ray2D ray, out QuadtreeRaycastHit<T> hit, float maxDistance = float.PositiveInfinity) where T : unmanaged
        {
            return quadtree.Raycast<RayAABBIntersecter<T>>(ray, out hit, maxDistance: maxDistance);
        }

        struct RayAABBIntersecter<T> : IQuadtreeRayIntersecter<T> where T : unmanaged
        {
            public bool IntersectRay(in PrecomputedRay2D ray, T obj, AABB2D objBounds, out float distance)
            {
                return objBounds.IntersectsRay(ray.origin, ray.invDir, out distance);
            }
        }

        /// <summary>
        /// Appends all objects for which their AABB overlaps with the input range to the results list.
        /// </summary>
        /// <param name="quadtree"></param>
        /// <param name="range"></param>
        /// <param name="results"></param>
        /// <typeparam name="T"></typeparam>
        /// <remarks>Note that objects can be added to the list more than once if their bounds overlap multiple octree leafs</remarks>
        public static void RangeAABB<T>(this NativeQuadtree<T> quadtree, AABB2D range, NativeList<T> results) where T : unmanaged
        {
            var vistor = new RangeAABBVisitor<T>()
            {
                results = results
            };
         
            quadtree.Range(range, ref vistor);
        }

        struct RangeAABBVisitor<T> : IQuadtreeRangeVisitor<T> where T : unmanaged
        {
            public NativeList<T> results;
            
            public bool OnVisit(T obj, AABB2D objBounds, AABB2D queryRange)
            {
                if (objBounds.Overlaps(queryRange))
                    results.Add(obj);

                return true; // always keep iterating, we want to catch all objects
            }
        }
        
        /// <summary>
        /// Appends all objects for which their AABB overlaps with the input range to the results set.
        /// This guarantuees that each object will only appear once.
        /// </summary>
        /// <param name="quadtree"></param>
        /// <param name="range"></param>
        /// <param name="results"></param>
        /// <typeparam name="T"></typeparam>
        public static void RangeAABBUnique<T>(this NativeQuadtree<T> quadtree, AABB2D range, NativeParallelHashSet<T> results) where T : unmanaged, IEquatable<T>
        {
            var vistor = new RangeAABBUniqueVisitor<T>()
            {
                results = results
            };
         
            quadtree.Range(range, ref vistor);
        }

        struct RangeAABBUniqueVisitor<T> : IQuadtreeRangeVisitor<T> where T : unmanaged, IEquatable<T>
        {
            public NativeParallelHashSet<T> results;
            
            public bool OnVisit(T obj, AABB2D objBounds, AABB2D queryRange)
            {
                if (objBounds.Overlaps(queryRange))
                    results.Add(obj);

                return true; // always keep iterating, we want to catch all objects
            }
        }
        
        /// <summary>
        /// Finds the nearest object to a given point (based on it's bounding box)
        /// </summary>
        /// <param name="quadtree">The tree</param>
        /// <param name="point">Point to find nearest neighbour from</param>
        /// <param name="maxDistance">Max distance to limit the search</param>
        /// <param name="nearest">The nearest object found</param>
        /// <typeparam name="T"></typeparam>
        /// <returns>If an object was found within the given maximum distance</returns>
        public static bool TryGetNearestAABB<T>(this NativeQuadtree<T> quadtree, float2 point, float maxDistance, out T nearest) where T : unmanaged
        {
            var visitor = new QuadtreeNearestAABBVisitor<T>();
            quadtree.Nearest(point, maxDistance, ref visitor, default(AABBDistanceSquaredProvider<T>));
            nearest = visitor.nearest;
            return visitor.found;
        }
        
        /// <summary>
        /// Finds the nearest object to a given point (based on it's bounding box)
        /// </summary>
        /// <param name="queryCache">The already allocated query cache</param>
        /// <param name="quadtree">The tree</param>
        /// <param name="point">Point to find nearest neighbour from</param>
        /// <param name="maxDistance">Max distance to limit the search</param>
        /// <param name="nearest">The nearest object found</param>
        /// <typeparam name="T"></typeparam>
        /// <returns>If an object was found within the given maximum distance</returns>
        public static bool TryGetNearestAABB<T>(this NativeQuadtree<T>.NearestNeighbourQuery queryCache, ref NativeQuadtree<T> quadtree, float2 point, float maxDistance, out T nearest) where T : unmanaged
        {
            var visitor = new QuadtreeNearestAABBVisitor<T>();
            queryCache.Nearest(ref quadtree, point, maxDistance, ref visitor, default(AABBDistanceSquaredProvider<T>));
            nearest = visitor.nearest;
            return visitor.found;
        }

        public struct AABBDistanceSquaredProvider<T> : IQuadtreeDistanceProvider<T> where T : unmanaged
        {
            public float DistanceSquared(float2 point, T obj, AABB2D bounds) => bounds.DistanceSquared(point);
        }

        struct QuadtreeNearestAABBVisitor<T> : IQuadtreeNearestVisitor<T> where T : unmanaged
        {
            public T nearest;
            public bool found;
            
            public bool OnVist(T obj)
            {
                this.found = true;
                this.nearest = obj;
                
                return false; // immediately stop iterating at first hit
            }
        }
    }
}