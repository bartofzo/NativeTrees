using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// ReSharper disable StaticMemberInGenericType

namespace NativeTrees
{
    /// <summary>
    /// Generic Burst/ECS compatible sparse octree that stores objects together with their axis aligned bounding boxes (AABB's)
    /// <para>
    /// Supported queries:
    ///     - Raycast
    ///     - Range (AABB overlap)
    ///     - N-nearest neighbour
    /// </para>
    /// <para>
    /// Other features:
    ///     - Implemented as a sparse octree, so only stores nodes that are occupied,
    ///       allowing it to go to a max depth of 10 (this could be more if the nodeId's are stored as long values
    ///     - Supports insertion of AABB's
    ///     - Fast path insertino for points
    ///     - Employs an extremely fast technique of checking for AABB / octant overlaps, see comment near the end
    ///     - Optimized with SIMD instructions so greatly benefits from burst compilation
    /// </para>
    /// <para>
    /// Limitations:
    ///     - No remove or update. Tried several approaches but they either left an unbalanced tree or doing a
    ///       full clear and re-insert was faster.
    /// </para>
    /// <para>
    /// Future todo's:
    ///     - Frustrum query
    ///     - 'Fat' raycast (virtually expand AABB's of nodes and objects when testing for ray intersections)
    /// </para>
    /// </summary>
    public struct NativeOctree<T> : INativeDisposable where T : unmanaged 
    {
        private readonly int maxDepth;
        private readonly int objectsPerNode;
        
        private AABB bounds;
        private readonly float3 boundsCenter;  // Precomputed bounds values as they are used often
        private readonly float3 boundsExtents;
        private readonly float3 boundsQuarterSize;
        
        /// <summary>
        /// Mapping from nodeId to the amount of objects that are in it. Once that limit surpasses <see cref="objectsPerNode"/>
        /// the node subdivides and the count remains at <see cref="objectsPerNode"/> + 1.
        /// To check if a node is a leaf is therefore as simple as checking that the count is less or equal than <see cref="objectsPerNode"/>,
        /// or whether depth is <see cref="maxDepth"/>
        /// </summary>
        private NativeParallelHashMap<int, int> nodes;
        private NativeParallelMultiHashMap<int, ObjWrapper> objects;
        
        /// <summary>
        /// Constructs an octree with a max depth of 8
        /// </summary>
        public NativeOctree(AABB bounds, Allocator allocator) : this(bounds, 16, 8, allocator)
        {
        }
        
        /// <summary>
        /// Constructs an octree
        /// </summary>
        /// <param name="bounds"></param>
        /// <param name="objectsPerNode">The max amount of objects per octant until max depth is reached</param>
        /// <param name="maxDepth">Max split depth of the tree.</param>
        /// <param name="initialCapacity">Hint at the initial capacity of the tree</param>
        /// <param name="allocator"></param>
        public NativeOctree(AABB bounds, int objectsPerNode, int maxDepth, Allocator allocator, int initialCapacity = 0)
        {
            if (maxDepth <= 1 || maxDepth > MaxDepth)
                throw new ArgumentOutOfRangeException(nameof(maxDepth), "Max depth is " + MaxDepth);
            if (!bounds.IsValid)
                throw new ArgumentException("Bounds max must be greater than min");
            
            objects = new NativeParallelMultiHashMap<int, ObjWrapper>(initialCapacity, allocator);
            nodes = new NativeParallelHashMap<int, int>(initialCapacity / objectsPerNode, allocator);
  
            this.objectsPerNode = objectsPerNode;
            this.maxDepth = maxDepth;
            this.bounds = bounds;
            this.boundsCenter = bounds.Center;
            this.boundsExtents = bounds.Size / 2;
            this.boundsQuarterSize = boundsExtents / 2;
            
            Clear();
        }

        /// <summary>
        /// Clear the tree
        /// </summary>
        public void Clear()
        {
            objects.Clear();
            nodes.Clear();
        }

        /// <summary>
        /// Bounds of the tree
        /// </summary>
        public AABB Bounds => bounds;
        
        #region Insertion
        
        /// <summary>
        /// Insert an object into the tree.
        /// </summary>
        /// <param name="obj">The object to insert</param>
        /// <param name="bounds">The axis aligned bounding box of the object</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(T obj, AABB bounds)
        {
            // Root node id = 1
            // we always start at depth one (assume one split)
            // our root node has a non zero id so that octants with bits 000 never get confused with the root node
            // since we have 32 bits of room and our max depth is 10 (30 bits) we have room for that one root node bit
            // at position 31
            var objWrapper = new ObjWrapper(obj, bounds);
            InsertNext(1, new QuarterSizeBounds(boundsCenter, boundsQuarterSize), objWrapper, 0);
        }

        /// <summary>
        /// Insert a point into the octree (AABB's min == max).
        /// This insertion method is significantly faster than the insertion with an AABB.
        /// </summary>
        public void InsertPoint(T obj, float3 point)
        {
            var objWrapper = new ObjWrapper(obj, new AABB(point, point));

            // We can (mainly) do without recursion for this insertion method. Except for when an octant needs to subdivide.
            QuarterSizeBounds extents = new QuarterSizeBounds(boundsCenter, boundsQuarterSize);
            int depth = 0;
            int nodeId = 1;
            while (depth <= maxDepth)
            {
                // We can get the one octant the point is in with one operation
                int octantIndex = PointToOctantIndex(point, extents.nodeCenter);
                extents = QuarterSizeBounds.GetOctant(extents, octantIndex);
                nodeId = GetOctantId(nodeId, octantIndex);
                
                if (TryInsert(nodeId, extents, objWrapper, depth))
                    return;

                depth++;
            }
        }

        void InsertNext(int nodeid, in QuarterSizeBounds quarterSizeBounds, in ObjWrapper objWrapper, int parentDepth)
        {
            parentDepth++;
            int objMask = GetBoundsMask(quarterSizeBounds.nodeCenter, objWrapper.bounds);

            for (int i = 0; i < 8; i++)
            {
                int octantMask = OctantMasks[i];
                if ((objMask & octantMask) != octantMask)
                    continue;
                
                int ocantId = GetOctantId(nodeid, i);
                var octantCenterQuarterSize = QuarterSizeBounds.GetOctant(quarterSizeBounds, i);
                
                if (!TryInsert(ocantId, octantCenterQuarterSize, objWrapper, parentDepth))
                    InsertNext(ocantId, octantCenterQuarterSize, objWrapper, parentDepth);
            }
        }

        /// <summary>
        /// Inserts the object if the node is a leaf. Otherwise, returns false and the tree should be traversed deeper.
        /// </summary>
        bool TryInsert(int nodeId, in QuarterSizeBounds extents, in ObjWrapper objWrapper, int depth)
        {
            nodes.TryGetValue(nodeId, out int objectCount);
           
            // a node is considered a leaf as long as it's (prev) objectcount <= max objects per node
            // or, when it is at the max tree depth
            if (objectCount <= objectsPerNode || depth == maxDepth)
            {
                objectCount++;
                objects.Add(nodeId, objWrapper);
                nodes[nodeId] = objectCount;
                if (objectCount > objectsPerNode && depth < maxDepth)
                    Subdivide(nodeId, in extents, depth);

                return true;
            }

            return false;
        }

        void Subdivide(int nodeId, in QuarterSizeBounds quarterSizeBounds, int depth)
        {
            int objectCount = 0;
            NativeArray<ObjWrapper> tempObjects = new NativeArray<ObjWrapper>(objectsPerNode + 1, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            foreach (var tempObj in objects.GetValuesForKey(nodeId))
                tempObjects[objectCount++] = tempObj;

            FixedList64Bytes<int> countPerOctant = new FixedList64Bytes<int>();
            countPerOctant.Length = 8;

            objects.Remove(nodeId); // remove all occurances of objects in our original
            for (int i = 0; i < objectCount; i++)
            {
                var moveObject = tempObjects[i];
                int aabbMask = GetBoundsMask(quarterSizeBounds.nodeCenter, moveObject.bounds);

                // Can't make the point optimization here because we can't be certain the node only contained points
                // Also, benchmarking turned out that this does not yield a significant performance boost anyway
              
                for (int j = 0; j < 8; j++)
                {
                    int octantMask = OctantMasks[j];
                    if ((aabbMask & octantMask) == octantMask)
                    {
                        objects.Add(GetOctantId(nodeId, j), moveObject);
                        countPerOctant[j] = countPerOctant[j] + 1; // ++ ?
                    }
                }
            }

            tempObjects.Dispose();

            // Update counts, create nodes when neccessary
            depth++;
            for (int i = 0; i < 8; i++)
            {
                int count = countPerOctant[i];
                if (count > 0)
                {
                    int octantId = GetOctantId(nodeId, i);
                    nodes[octantId] = count; // mark our node as being used
                    
                    // could be that we need to subdivide again if all of the objects went to the same octant
                    if (count > objectsPerNode && depth < maxDepth) // todo: maxDepth check can be hoisted
                        Subdivide(octantId, QuarterSizeBounds.GetOctant(quarterSizeBounds, i), depth);
                }
            }
        }
       
        #endregion
        
        #region Raycast

        /// <summary>
        /// Visits all leaf nodes that are intersected by a ray, in the order in which they are intersected.
        /// </summary>
        /// <param name="ray">The ray to test against</param>
        /// <param name="visitor">Delegate to test for ray intersections on the objects</param>
        /// <param name="hit">The resulting hit</param>
        /// <returns>True if a hit was made</returns>
        public bool Raycast<U>(Ray ray, out OctreeRaycastHit<T> hit, U visitor = default) where U : struct, IOctreeRayIntersecter<T>
        {
            var computedRay = new PrecomputedRay(ray);

            // check if ray even hits the boundary, and if so, we use the intersectin point to transpose our ray
            if (!bounds.IntersectsRay(computedRay, out float3 rayPos))
            {
                hit = default;
                return false;
            }
            
            // Note: transpose computed ray to boundary and go
            return RaycastNext(
                ray: new PrecomputedRay(computedRay, rayPos), 
                nodeId: 1, 
                extentsBounds: new ExtentsBounds(boundsCenter, boundsExtents), 
                hit: out hit, 
                visitor: ref visitor, 
                parentDepth: 0);
        }
        
        bool RaycastNext<U>(
            in PrecomputedRay ray,
            int nodeId, in ExtentsBounds extentsBounds,
            out OctreeRaycastHit<T> hit, 
            ref U visitor, 
            int parentDepth) 
            where U : struct, IOctreeRayIntersecter<T>
        {
            parentDepth++;
            
            // Reference for the method used to determine the order of octants to visit
            // https://daeken.svbtle.com/a-stupidly-simple-fast-octree-traversal-for-ray-intersection
            
            // Compute the bounds of the parent node we're in, we use it to check if a plane intersection is valid
            var parentBounds = ExtentsBounds.GetBounds(extentsBounds);
            
            // compute the plane intersections of YZ, XZ and XY
            float3 planeHits = PlaneHits(ray, extentsBounds.nodeCenter);
            
            // for our first (closest) octant, it must be the position the ray entered the parent node
            int octantIndex = PointToOctantIndex(ray.origin, extentsBounds.nodeCenter);
            float3 octantRayIntersection = ray.origin;

            for (int i = 0; i < 4; i++)
            {
                int octantId = GetOctantId(nodeId, octantIndex);
                if (nodes.TryGetValue(octantId, out int objectCount) && 
                    Raycast(
                        ray: new PrecomputedRay(ray, octantRayIntersection), 
                        nodeId: octantId, 
                        extentsBounds: ExtentsBounds.GetOctant(extentsBounds, octantIndex), 
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
                
                for (int j = 0; j < 3; j++)
                {
                    float t = planeHits[j];
                    if (t > closestDistance) continue;
                    
                    float3 planeRayIntersection = ray.origin + t * ray.dir;
                    if (parentBounds.Contains(planeRayIntersection))
                    {
                        octantRayIntersection = planeRayIntersection;
                        closestPlaneIndex = j;
                        closestDistance = t;
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
        
        bool Raycast<U>(in PrecomputedRay ray, 
            int nodeId, 
            in ExtentsBounds extentsBounds, 
            int objectCount,
            out OctreeRaycastHit<T> hit,
            ref U visitor, 
            int depth) where U : struct, IOctreeRayIntersecter<T>
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
        
        #endregion
        
        #region Range
        
        /// <summary>
        /// Visits all objects that are contained in the octree leafs that overlap with a range.
        /// Does not check if the object's bounds overlap, that should be implemented on the visitor delegate.
        /// </summary>
        /// <param name="range"></param>
        /// <param name="visitor"></param>
        /// <typeparam name="U"></typeparam>
        /// <remarks>It's possible for objects to be visited multiple times if their bounds span multiple leafs</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void VisitRange<U>(in AABB range, ref U visitor) where U : struct, IOctreeRangeVisitor<T>
        {
            VisitRangeNext(
                range: range,
                nodeId: 1,
                quarterSizeBounds: new QuarterSizeBounds(boundsCenter, boundsQuarterSize),
                visitor: ref visitor,
                parentDepth: 0);
        }
        
        bool VisitRangeNext<U>(in AABB range, int nodeId, in QuarterSizeBounds quarterSizeBounds, ref U visitor, int parentDepth) where U : struct, IOctreeRangeVisitor<T>
        {
            parentDepth++;
            int rangeMask = GetBoundsMask(quarterSizeBounds.nodeCenter, range);
            
            for (int i = 0; i < 8; i++)
            {
                int octantMask = OctantMasks[i];
                if ((rangeMask & octantMask) == octantMask)
                {
                    int octantId = GetOctantId(nodeId, i);
                    if (nodes.TryGetValue(octantId, out int objectCount) &&
                        !VisitRange(
                            range: range,
                            nodeId: octantId,
                            quarterSizeBounds: QuarterSizeBounds.GetOctant(quarterSizeBounds, i),
                            objectCount: objectCount,
                            visitor: ref visitor,
                            depth: parentDepth))
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        
        bool VisitRange<U>(in AABB range, int nodeId, in QuarterSizeBounds quarterSizeBounds, int objectCount, ref U visitor, int depth)
            where U : struct, IOctreeRangeVisitor<T>
        {
            // Are we in a leaf node?
            if (objectCount <= objectsPerNode || depth == maxDepth)
            {
                if (objects.TryGetFirstValue(nodeId, out var wrappedObj, out var it))
                {
                    do
                    {
                        if (!visitor.OnVisit(wrappedObj.obj, wrappedObj.bounds, range))
                            return false; // stop traversing if visitor says so
                    } while (objects.TryGetNextValue(out wrappedObj, ref it));
                }
            }
            
            return VisitRangeNext(
                range: range,
                nodeId: nodeId,
                quarterSizeBounds: quarterSizeBounds,
                visitor: ref visitor,
                parentDepth: depth);
        }
        
        #endregion

        #region Nearest Neighbour
        
        /// <summary>
        /// Struct to perform an N-nearest neighbour query on the tree.
        /// </summary>
        /// <remarks>Implemented as a struct because this type of query requires the use of some extra native containers.
        /// You can cache this struct and re-use it to circumvent the extra cost associated with allocating the internal containers.</remarks>
        public struct NearestNeighbourQuery : INativeDisposable
        {
            // objects and nodes are stored in a separate list, as benchmarking turned out,
            // putting everything in one big struct was much, much slower because of the large struct size
            // we want to keep the struct in the minheap as small as possibly as many comparisons and swaps take place there
            private NativeList<ObjWrapper> objList;
            private NativeList<NodeWrapper> nodeList;
            private NativeMinHeap<DistanceAndIndexWrapper, NearestComp> minHeap;

            public NearestNeighbourQuery(Allocator allocator) : this(0, allocator)
            {
            }

            public NearestNeighbourQuery(int initialCapacity, Allocator allocator)
            {
                nodeList = new NativeList<NodeWrapper>(initialCapacity, allocator);
                objList = new NativeList<ObjWrapper>(initialCapacity, allocator);
                minHeap = new NativeMinHeap<DistanceAndIndexWrapper, NearestComp>(default, allocator);
            }

            public void Dispose()
            {
                objList.Dispose();
                nodeList.Dispose();
                minHeap.Dispose();
            }

            public JobHandle Dispose(JobHandle inputDeps)
            {
                return JobHandle.CombineDependencies(
                    objList.Dispose(inputDeps),
                    nodeList.Dispose(inputDeps),
                    minHeap.Dispose(inputDeps));
            }

            /// <summary>
            /// Perform a nearest neighbour query. 
            /// </summary>
            /// <param name="octree">Octree to perform the query on</param>
            /// <param name="point">Point to get nearest neighbours for</param>
            /// <param name="maxDistance">Maximum distance to look</param>
            /// <param name="visitor">Handler for when a neighbour is encountered</param>
            /// <param name="distanceSquaredProvider">Provide a calculation for the distance</param>
            /// <typeparam name="U">Handler type for when a neighbour is encountered</typeparam>
            /// <typeparam name="V">Provide a calculation for the distance</typeparam>
            public void Nearest<U, V>(ref NativeOctree<T> octree, float3 point, float maxDistance, ref U visitor, V distanceSquaredProvider = default)
                where U : struct, IOctreeNearestVisitor<T>
                where V : struct, IOctreeDistanceProvider<T>
            {
                float maxDistanceSquared = maxDistance * maxDistance;
                
                // reference for the method used:
                // https://stackoverflow.com/questions/41306122/nearest-neighbor-search-in-octree
                // - add root to priority queue
                // - pop queue, if it's an object, it's the closest one, if it's a node, add it's children to the queue
                // - repeat
                
                minHeap.Clear();
                nodeList.Clear();
                objList.Clear();

                var root = new NodeWrapper(
                    nodeId: 1, 
                    nodeDepth: 0, 
                    nodeCounter: 0,
                    extentsBounds: new ExtentsBounds(octree.boundsCenter, octree.boundsExtents));
                
                // Add our first octants to the heap
                NearestNodeNext(
                    octree: ref octree,
                    point: point,
                    nodeWrapper: ref root,
                    maxDistanceSquared: maxDistanceSquared,
                    parentDepth: 0);
                
                while (minHeap.TryPop(out var nearestWrapper))
                {
                    if (nearestWrapper.isNode)
                    {
                        NearestNode(
                            octree: ref octree,
                            point: point,
                            distanceAndIndexWrapper: nearestWrapper,
                            maxDistanceSquared: maxDistanceSquared,
                            distanceProvider: distanceSquaredProvider);
                    }
                    else if (!visitor.OnVist(objList[nearestWrapper.objIndex].obj, point))
                    {
                        break;
                    }
                }
            }

            void NearestNode<V>(ref NativeOctree<T> octree, float3 point, float maxDistanceSquared, in DistanceAndIndexWrapper distanceAndIndexWrapper, V distanceProvider = default)
                where V : struct, IOctreeDistanceProvider<T>
            {
                ref var node = ref nodeList.ElementAt(distanceAndIndexWrapper.nodeIndex);
                ref var objects = ref octree.objects;
                
                // Leaf?
                if (node.nodeCounter <= octree.objectsPerNode || node.nodeDepth == octree.maxDepth)
                {
                    if (objects.TryGetFirstValue(node.nodeId, out var objWrapper, out var it))
                    {
                        do
                        {
                            float objDistanceSquared = distanceProvider.DistanceSquared(point, objWrapper.obj, objWrapper.bounds);
                            if (objDistanceSquared > maxDistanceSquared)
                                continue;

                            int objIndex = objList.Length;
                            objList.Add(objWrapper);

                            minHeap.Push(new DistanceAndIndexWrapper(
                                distanceSquared: objDistanceSquared, 
                                objIndex: objIndex,
                                nodeIndex: 0, 
                                isNode: false));

                        } while (objects.TryGetNextValue(out objWrapper, ref it));
                    }

                    return;
                }

                // Add child nodes
                NearestNodeNext(
                    octree: ref octree,
                    point: point,
                    nodeWrapper: ref node,
                    maxDistanceSquared: maxDistanceSquared,
                    parentDepth: node.nodeDepth);
            }

            void NearestNodeNext(ref NativeOctree<T> octree, float3 point, ref NodeWrapper nodeWrapper, float maxDistanceSquared, int parentDepth)
            {
                parentDepth++;
                for (int i = 0; i < 8; i++)
                {
                    int octantId = GetOctantId(nodeWrapper.nodeId, i);
                    if (!octree.nodes.TryGetValue(octantId, out int octantObjectCount))
                        continue;

                    var octantCenterExtents = ExtentsBounds.GetOctant(nodeWrapper.ExtentsBounds, i);
                    float distanceSquared = ExtentsBounds.GetBounds(octantCenterExtents).DistanceSquared(point);

                    if (distanceSquared > maxDistanceSquared)
                        continue;

                    int nodeIndex = nodeList.Length;
                    nodeList.Add(
                        new NodeWrapper(
                            nodeId: octantId, 
                            nodeDepth: parentDepth, 
                            nodeCounter:octantObjectCount, 
                            extentsBounds: octantCenterExtents));

                    minHeap.Push(new DistanceAndIndexWrapper(
                        distanceSquared: distanceSquared, 
                        objIndex: 0, 
                        nodeIndex: nodeIndex, 
                        isNode: true));
                }
            }
            
            /// <summary>
            /// Goes in the priority queue
            /// </summary>
            readonly struct DistanceAndIndexWrapper
            {
                public readonly float distanceSquared;
                
                // There's no polymorphism with HPC#, so this is our way around that
                public readonly int objIndex;
                public readonly int nodeIndex;
                public readonly bool isNode;

                public DistanceAndIndexWrapper(float distanceSquared, int objIndex, int nodeIndex, bool isNode)
                {
                    this.distanceSquared = distanceSquared;
                    this.objIndex = objIndex;
                    this.nodeIndex = nodeIndex;
                    this.isNode = isNode;
                }
            }

            readonly struct NodeWrapper
            {
                public readonly int nodeId;
                public readonly int nodeDepth;
                public readonly int nodeCounter;
                public readonly ExtentsBounds ExtentsBounds;

                public NodeWrapper(int nodeId, int nodeDepth, int nodeCounter, in ExtentsBounds extentsBounds)
                {
                    this.nodeId = nodeId;
                    this.nodeDepth = nodeDepth;
                    this.nodeCounter = nodeCounter;
                    this.ExtentsBounds = extentsBounds;
                }
            }
            
            struct NearestComp : IComparer<DistanceAndIndexWrapper>
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public int Compare(DistanceAndIndexWrapper x, DistanceAndIndexWrapper y) => x.distanceSquared.CompareTo(y.distanceSquared);
            }
        }

        /// <summary>
        /// Perform a nearest neighbour query. 
        /// </summary>
        /// <param name="point">Point to get nearest neighbours for</param>
        /// <param name="maxDistance">Maximum distance to look</param>
        /// <param name="visitor">Handler for when a neighbour is encountered</param>
        /// <param name="distanceSquaredProvider">Provide a calculation for the distance</param>
        /// <typeparam name="U">Handler type for when a neighbour is encountered</typeparam>
        /// <typeparam name="V">Provide a calculation for the distance</typeparam>
        /// <remarks>Allocates native containers. To prevent reallocating for every query, create a <see cref="NearestNeighbourQuery"/> struct
        /// and re-use it.</remarks>
        public void Nearest<U, V>(float3 point, float maxDistance, ref U visitor, V distanceSquaredProvider = default)
            where U : struct, IOctreeNearestVisitor<T>
            where V : struct, IOctreeDistanceProvider<T>
        {
            var query = new NearestNeighbourQuery(Allocator.Temp);
            query.Nearest(ref this, point, maxDistance, ref visitor, distanceSquaredProvider);
            query.Dispose();
        }

        #endregion
        
        #region Util

        /// <summary>
        /// Returns the octant's index of a point. Which translated to bits means if the point is on the negative (0) side or positve (1)
        /// side of the node's center.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int PointToOctantIndex(float3 point, float3 nodeCenter) =>
            math.bitmask((point >= nodeCenter).xxyz) >> 1;
        
        /// <summary>
        /// Computes ray plane intersections compute of YZ, XZ and XY respectively stored in xyz
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 PlaneHits(in PrecomputedRay ray, float3 nodeCenter) => (nodeCenter - ray.origin) * ray.invDir;

        /// <summary>
        /// Max depth of the octree, we need 3 bits for each level we go down
        /// </summary>
        public const int MaxDepth = 8 * sizeof(int) / 3;
        
        /// <summary>
        /// Gets a unique identifier for a node.
        /// The octants are identified using the following pattern, where a zero stands for the negative side:
        /// 
        ///         110     111
        ///     010     011
        ///     
        ///         100     101
        ///     000    001
        /// 
        /// For each level we go down, we shift these bits 3 spaces to the left
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetOctantId(int parent, int octantIndex) => (parent << 3) | octantIndex;
        
        /*
         * AABB - Octant overlap technique explanation:
         * We use a clever technique to determine if bounds belong in an octant by computing one mask value for the object's bounds
         * and comparing it to a predefined bitmask for each octant. This avoids needing to compute the actual overlap eight times
         * and instead boils down to one bitwise AND comparison for each octant. This boosts performance for AABB insertion and range queries.
         *
         * How it works:
         * First, we compare the object's bounds to the center of the parent node we're currently in and convert it to a 6 bit wide bitmask where
         * the lower 3 bits are set to 1 if the bounds min was on a negative side for that axis.
         * The upper 3 bits are set to 1 if the bounds max was on the positive side for that axis.
         *
         * We have predefined masks for each octant, where a bitwise AND against the object's mask tells you if the object belongs in that octant,
         * in which case the result must be equal the the octant mask value.
         * 
         * The octant mask says: if the octant is negative for an axis, the min bit for that axis must be set on the object.
         * If the octant is positive for an axis, the max bit for that axis must be set on the object.
         *
         * This works because for positive axes, the object's bounds max must be in there (it's impossible for max to be negative and min to be positive)
         * Same goes for negative axis overlaps, it's impossible for the max to be on the negative side if min is not there as well
         *
         * Also note that, we don't need to check against the outer bounds of the parent node, as this inherently happens because
         * every object is inserted top down from the root node.
         */

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int GetBoundsMask(float3 nodeCenter, AABB aabb)
        {
            int offMin = math.bitmask(aabb.min.xxyz <= nodeCenter.xxyz) >> 1; // leaves xyz
            // for the max we have one bit too many but that's OK, since we AND it with the octant mask, we lose that bit anyway
            return offMin | (math.bitmask(aabb.max.xyzz >= nodeCenter.xyzz) << 3);  
            
            // non-SIMD version:
            /*
            float3 offMin = aabb.min - nodeCenter;
            float3 offMax = aabb.max - nodeCenter;
            return (offMin.x <= 0 ? 0b000_001 : 0) |
                   (offMin.y <= 0 ? 0b000_010 : 0) |
                   (offMin.z <= 0 ? 0b000_100 : 0) |
                   (offMax.x >= 0 ? 0b001_000 : 0) |
                   (offMax.y >= 0 ? 0b010_000 : 0) |
                   (offMax.z >= 0 ? 0b100_000 : 0);
                   */
        }
        
        private static readonly int[] OctantMasks = new[]
        {
            //             ZYX
            0b000_111,  // 000  all negative, so the entire object's min should be negative
            0b001_110,  // 001  YZ are negative so they compare with min, but X is positive so compares with max
            0b010_101,  // 010  XZ negative so compare them with min, Y is positive so compares with max 
            0b011_100,  // 011  Z is negative, compare that with min. Compare XY with max.
            0b100_011,  // 100  etc...
            0b101_010,  // 101
            0b110_001,  // 110
            0b111_000   // 111 all axes are positive, so the entire objects max should be positive as well
        };

        // Offset from parent node's center multipliers for each octant
        private static readonly float3[] OctantCenterOffsets = new[]
        {
            //                         ZYX
            new float3(-1),         // 000
            new float3(1, -1, -1),  // 001
            new float3(-1, 1, -1),  // 010
            new float3(1, 1, -1),   // 011
            new float3(-1, -1, 1),  // 100
            new float3(1, -1, 1),   // 101
            new float3(-1, 1, 1),   // 110
            new float3(1)           // 111
        };
        
        #endregion
        
        /// <summary>
        /// Stores an object together with it's bounds
        /// </summary>
        readonly struct ObjWrapper
        {
            public readonly AABB bounds;
            public readonly T obj;

            public ObjWrapper(T obj, AABB bounds)
            {
                this.obj = obj;
                this.bounds = bounds;
            }
        }
        
        // This turned out to be the most efficient way of passing down dimensions of nodes
        // for insertion and range queries
        readonly struct QuarterSizeBounds
        {
            public readonly float3 nodeCenter;
            public readonly float3 nodeQuarterSize;

            public QuarterSizeBounds(float3 nodeCenter, float3 nodeQuarterSize)
            {
                this.nodeCenter = nodeCenter;
                this.nodeQuarterSize = nodeQuarterSize;
            }
            
            /// <summary>
            /// Returns the insert node info for the next octant at index.
            /// </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static QuarterSizeBounds GetOctant(in QuarterSizeBounds parent, int index) => 
                new QuarterSizeBounds(parent.nodeCenter + OctantCenterOffsets[index] * parent.nodeQuarterSize, .5f * parent.nodeQuarterSize);
            
            // note: yes, new quarterSize can be precomputed but benchmarking showed no difference and it enhances readability
        }
        
        readonly struct ExtentsBounds
        {
            public readonly float3 nodeCenter;
            public readonly float3 nodeExtents;

            public ExtentsBounds(float3 nodeCenter, float3 nodeExtents)
            {
                this.nodeCenter = nodeCenter;
                this.nodeExtents = nodeExtents;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ExtentsBounds GetOctant(in ExtentsBounds parent, int index)
            {
                float3 octantExtents = .5f * parent.nodeExtents;
                return new ExtentsBounds(parent.nodeCenter + OctantCenterOffsets[index] * octantExtents, octantExtents);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static AABB GetBounds(in ExtentsBounds ce) => new AABB(ce.nodeCenter - ce.nodeExtents, ce.nodeCenter + ce.nodeExtents);
        }
        
        #region Disposal and Copying
        
        /// <summary>
        /// Clears and copies the contents of the source tree into this one
        /// </summary>
        /// <param name="source">The source tree to copy</param>
        public void CopyFrom(NativeOctree<T> source)
        {
            if (this.maxDepth != source.maxDepth || this.objectsPerNode != source.objectsPerNode)
                throw new ArgumentException("Source maxDepth and objectsPerNode must be the same as destination");
            if (!this.boundsCenter.Equals(source.boundsCenter) || ! this.boundsExtents.Equals(source.boundsExtents))
                throw new ArgumentException("Source bounds must be equal");

            objects.Clear();
            nodes.Clear();

            var kvs = source.objects.GetKeyValueArrays(Allocator.Temp);
            for (int i = 0; i < kvs.Length; i++)
                this.objects.Add(kvs.Keys[i], kvs.Values[i]);
           
            var kvs2 = source.nodes.GetKeyValueArrays(Allocator.Temp);
            for (int i = 0; i < kvs2.Length; i++)
                this.nodes.Add(kvs2.Keys[i], kvs2.Values[i]);
            
            kvs.Dispose();
            kvs2.Dispose();
        }
        
        /// <summary>
        /// Dispose the NativeOctree
        /// </summary>
        public void Dispose()
        {
            nodes.Dispose();
            objects.Dispose();
        }
        
        /// <summary>
        /// Dispose the NativeOctree
        /// </summary>
        public JobHandle Dispose(JobHandle inputDeps)
        {
            return JobHandle.CombineDependencies(nodes.Dispose(inputDeps), objects.Dispose(inputDeps));
        }
        
        #endregion
    }
}