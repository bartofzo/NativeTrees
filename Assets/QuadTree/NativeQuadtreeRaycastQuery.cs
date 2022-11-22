using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace NativeTrees
{
    public partial struct NativeQuadtree<T> : INativeDisposable where T : unmanaged 
    {
        /// <summary>
        /// Perform a raycast against the quadtree
        /// </summary>
        /// <param name="ray">Input ray</param>
        /// <param name="hit">The resulting hit</param>
        /// <param name="intersecter">Delegate to compute ray intersections against the objects or AABB's</param>
        /// <typeparam name="U">Type of intersecter</typeparam>
        /// <returns>True when a hit has occured</returns>
        public bool Raycast<U>(Ray2D ray, out QuadtreeRaycastHit<T> hit, U intersecter = default) where U : struct, IQuadtreeRayIntersecter<T>
        {
            var computedRay = new PrecomputedRay2D(ray);

            // check if ray even hits the boundary, and if so, we use the intersectin point to transpose our ray
            if (!bounds.IntersectsRay(computedRay, out float2 rayPos))
            {
                hit = default;
                return false;
            }

            // Note: transpose computed ray to boundary and go
            return RaycastNext(
                ray: new PrecomputedRay2D(computedRay, rayPos), 
                nodeId: 1, 
                extentsBounds: new ExtentsBounds(boundsCenter, boundsExtents), 
                hit: out hit, 
                visitor: ref intersecter, 
                parentDepth: 0);
        }
        
        bool RaycastNext<U>(
            in PrecomputedRay2D ray,
            uint nodeId, in ExtentsBounds extentsBounds,
            out QuadtreeRaycastHit<T> hit, 
            ref U visitor, 
            int parentDepth) 
            where U : struct, IQuadtreeRayIntersecter<T>
        {
            parentDepth++;
            
            // Reference for the method used to determine the order of octants to visit
            // https://daeken.svbtle.com/a-stupidly-simple-fast-quadtree-traversal-for-ray-intersection
            
            // Compute the bounds of the parent node we're in, we use it to check if a plane intersection is valid
            var parentBounds = ExtentsBounds.GetBounds(extentsBounds);
            
            // compute the plane intersections
            float2 planeHits = PlaneHits(ray, extentsBounds.nodeCenter);
            
            // for our first (closest) octant, it must be the position the ray entered the parent node
            int octantIndex = PointToQuadIndex(ray.origin, extentsBounds.nodeCenter);
            float2 octantRayIntersection = ray.origin;
            UnityEngine.Gizmos.DrawWireSphere((Vector2)octantRayIntersection, .5f);

            for (int i = 0; i < 3; i++)
            {
                uint octantId = GetQuadId(nodeId, octantIndex);

                #if DEBUG_RAYCAST_GIZMO
                var debugExt = ExtentsBounds.GetOctant(extentsBounds, octantIndex);
                var color = new Color(0, 0, 0, .25f);
                color[i] = 1f;
                UnityEngine.Gizmos.color = color;
                UnityEngine.Gizmos.DrawCube((Vector2)debugExt.nodeCenter, (Vector2)debugExt.nodeExtents * 1.75f);
                #endif
                
                if (nodes.TryGetValue(octantId, out int objectCount) && 
                    Raycast(
                        ray: new PrecomputedRay2D(ray, octantRayIntersection), 
                        nodeId: octantId, 
                        extentsBounds: ExtentsBounds.GetQuad(extentsBounds, octantIndex), 
                        objectCount: objectCount,
                        hit: out hit,
                        visitor: ref visitor, 
                        depth: parentDepth))
                {
                    return true;
                }

                // find next octant to test:
                float closestDistance = float.PositiveInfinity;
                int closestPlaneIndex = -1;
                
                for (int j = 0; j < 2; j++)
                {
                    float t = planeHits[j];
                    if (t > closestDistance || t < 0) continue; // negative t is backwards

                    float2 planeRayIntersection = ray.origin + t * ray.dir;
                    if (parentBounds.Contains(planeRayIntersection))
                    {
                        octantRayIntersection = planeRayIntersection;
                        closestPlaneIndex = j;
                        closestDistance = t;
                        
                        #if DEBUG_RAYCAST_GIZMO
                        var debugExt2 = ExtentsBounds.GetOctant(extentsBounds, octantIndex);
                        var color2 = new Color(0, 0, 0, .25f);
                        color2[i] = 1f;
                        UnityEngine.Gizmos.color = color2;
                        UnityEngine.Gizmos.DrawSphere((Vector2)debugExt2.nodeCenter, .25f);
                        #endif
                    }
                }

                // No valid octant intersections left, bail
                if (closestPlaneIndex == -1)
                    break;
                
                // get next octant from plane index
                octantIndex ^= 1 << closestPlaneIndex;
                planeHits[closestPlaneIndex] = float.PositiveInfinity;
            }
            
            hit = default;
            return false;
        }
        
        bool Raycast<U>(in PrecomputedRay2D ray, 
            uint nodeId, 
            in ExtentsBounds extentsBounds, 
            int objectCount,
            out QuadtreeRaycastHit<T> hit,
            ref U visitor, 
            int depth) where U : struct, IQuadtreeRayIntersecter<T>
        {
            // Are we in a leaf node?
            if (objectCount <= objectsPerNode || depth == maxDepth)
            {
                hit = default;
                float closest = float.PositiveInfinity;
                bool didHit = false;

                if (objects.TryGetFirstValue(nodeId, out var wrappedObj, out var it))
                {
                    do
                    {
                        if (visitor.IntersectRay(ray, wrappedObj.obj, wrappedObj.bounds, out float t) && t < closest)
                        {
                            closest = t;
                            hit.obj = wrappedObj.obj;
                            didHit = true;
                        }
                    } while (objects.TryGetNextValue(out wrappedObj, ref it));
                }
                
                if (didHit)
                {
                    hit.point = ray.origin + ray.dir * closest;
                    return true;
                }
                
                return false;
            }

            return RaycastNext(
                ray: ray, 
                nodeId: nodeId, 
                extentsBounds: extentsBounds, 
                hit: out hit, 
                visitor: ref visitor, 
                parentDepth: depth);
        }
        
        /// <summary>
        /// Computes ray plane intersections compute of YZ, XZ and XY respectively stored in xyz
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float2 PlaneHits(in PrecomputedRay2D ray, float2 nodeCenter) => (nodeCenter - ray.origin) * ray.invDir;
    }
}