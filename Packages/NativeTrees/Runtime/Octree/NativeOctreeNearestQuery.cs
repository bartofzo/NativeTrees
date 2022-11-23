using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// https://bartvandesande.nl
// https://github.com/bartofzo
// ReSharper disable StaticMemberInGenericType

namespace NativeTrees
{
    public partial struct NativeOctree<T> : INativeDisposable
        where T : unmanaged 
    {
        /*
         * Note: while it's possible to have implemented this as an enumerator to support foreach, I decided against it
         * because the query cache couldn't be reused and for consistency with the range query (which would seriously suffer from
         * performance if implemented as an enumerator)
         */
       
        /// <summary>
        /// Perform a nearest neighbour query. 
        /// </summary>
        /// <param name="point">Point to get nearest neighbours for</param>
        /// <param name="maxDistance">Maximum distance to look</param>
        /// <param name="visitor">Handler for when a neighbour is encountered</param>
        /// <param name="distanceSquaredProvider">Provide a calculation for the distance</param>
        /// <typeparam name="U">Handler type for when a neighbour is encountered</typeparam>
        /// <typeparam name="V">Provide a calculation for the distance</typeparam>
        /// <remarks>Allocates native containers. To prevent reallocating for every query, create a <see cref="NearestNeighbourCache"/> struct
        /// and re-use it.</remarks>
        public void Nearest<U, V>(float3 point, float maxDistance, ref U visitor, V distanceSquaredProvider = default)
            where U : struct, IOctreeNearestVisitor<T>
            where V : struct, IOctreeDistanceProvider<T>
        {
            var query = new NearestNeighbourCache(Allocator.Temp);
            query.Nearest(ref this, point, maxDistance, ref visitor, distanceSquaredProvider);
            query.Dispose();
        }
        
        /// <summary>
        /// Struct to perform an N-nearest neighbour query on the tree.
        /// </summary>
        /// <remarks>Implemented as a struct because this type of query requires the use of some extra native containers.
        /// You can cache this struct and re-use it to circumvent the extra cost associated with allocating the internal containers.</remarks>
        public struct NearestNeighbourCache : INativeDisposable
        {
            // objects and nodes are stored in a separate list, as benchmarking turned out,
            // putting everything in one big struct was much, much slower because of the large struct size
            // we want to keep the struct in the minheap as small as possibly as many comparisons and swaps take place there
            private NativeList<ObjWrapper> objList;
            private NativeList<NodeWrapper> nodeList;
            private NativeMinHeap<DistanceAndIndexWrapper, NearestComp> minHeap;

            public NearestNeighbourCache(Allocator allocator) : this(0, allocator)
            {
            }

            public NearestNeighbourCache(int initialCapacity, Allocator allocator)
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
                    else if (!visitor.OnVist(objList[nearestWrapper.objIndex].obj))
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
                    uint octantId = GetOctantId(nodeWrapper.nodeId, i);
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
                public readonly uint nodeId;
                public readonly int nodeDepth;
                public readonly int nodeCounter;
                public readonly ExtentsBounds ExtentsBounds;

                public NodeWrapper(uint nodeId, int nodeDepth, int nodeCounter, in ExtentsBounds extentsBounds)
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
    }
}