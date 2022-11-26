![NativeTrees](./Packages/NativeTrees/NativeTrees.png "NativeTrees")

Generic sparse octree and quadtree
that store objects together with their axis aligned bounding boxes (AABB's)

Written in C# for Unity's burst compiler and the ECS/DOTS framework.
Tested with Unity 2021.3.11f1

### Supported queries:
- Raycast
- Range (AABB overlap)
- K-Nearest neighbours

### Other features:
- Implemented as a sparse tree, so only stores nodes that are occupied. Memory usage is therefore relatively low.
  The octree has a max depth of 10 and the quad tree a max depth of 15.
- Supports insertion of AABB's
- Fast insertion path for points
- Optimized with SIMD instructions so greatly benefits from burst compilation

### Limitations:
- Objects must be structs (Burst limitation)
- No remove or update. Tried several approaches but they either left an unbalanced tree or doing a
  full clear and re-insert was faster.
- No foreach support, we leverage the stack and "struct" delegates, which suits the recursive
  nature of the tree better.

- Insertion can only be done on a single thread. Queries ofcourse can be across multiple threads.

### Future todo's:
- Frustrum query
- 'Fat' raycast (virtually expand AABB's of nodes and objects when testing for ray intersections)

## Installation

Using the Unity package manager, choose Add Package from git URL and enter:

    https://github.com/bartofzo/NativeTrees.git?path=/Packages/NativeTrees

## Performance

The trees are heaviliy optimized to make use of SIMD instructions.
Therefore performance is best when used in burst compiled code.

Queries are *very* fast.
The raycast never visits more nodes than absolutely neccessary.
The overlap (and insertion) use a technique where to test in which child nodes AABB's should go, only
two comparisons are made followed up by some bitwise operations. (See the source for an explanation).

Nearest neighbour is the slowest of the bunch (but still fast) as it has some overhead in keeping track of a priority queue.

Actual performance can vary wildly depending on the structure of your tree and it's
settings. For the sake of cool stats, here are some numbers on insertion times and raycasts for random points and AABB's,
Single thread burst compiled, maxDepth of 8. These numbers should not be taken too seriously because it's random data
and the tree will be divided almost equally everywhere, which in most realistic scenarios is not the case.

Run on a MBP 16" i9 2.3 GHz from 2019.

|Obj Count| QT Points Insert | QT AABB Insert| OT Points Insert | OT AABB Insert| OT 1000 Inf Raycasts| OT 1000 Max Dist Raycasts
|---------|-----------|---------|-----------|---------|---------------------|----------------------
|1000     |0.07ms     |0.08ms   |0.07ms     |0.10ms   |0.25ms (4M Rays/sec) |0.13ms (7M Rays/sec)
|5000     |0.34ms     |0.57ms   |0.25ms     |0.50ms   |0.38ms               |0.19ms
|10K      |0.69ms     |1.23ms   |0.82ms     |1.55ms   |0.45ms               |0.23ms
|25K      |2.18ms     |4.15ms   |1.91ms     |3.89ms   |0.51ms               |0.32ms
|50K      |3.86ms     |10.87ms  |4.04ms     |9.61ms   |0.49ms               |0.32ms
|100K     |9.64ms     |26.12ms  |11.46ms    |18.81ms  |0.51ms               |0.28ms
|250K     |24.13ms    |76.85ms  |22.47ms    |50.95ms  |0.64ms               |0.39ms
|500K     |51.00ms    |131.58ms |60.08ms    |164.17ms |0.84ms               |0.45ms
|1M       |129.03ms   |260.55ms |152.50ms   |301.99ms |1.04ms               |0.55ms

## Usage

There are two samples included that show how to use the octree and quadtree.
The extension classes provide readymade solutions for AABB only checking. For more complicated
shapes you must provide your own ray/overlap/distance calculations.

NOTE: If you've imported the package via the Unity Package Manager, you need to copy the
sample scenes to your Assets folder to open them. 
[See this thread](https://forum.unity.com/threads/it-is-not-allowed-to-open-a-scene-in-a-read-only-package-why.1148036/)

### Insertion
The objects can be of any unmanaged type, when inserting, an AABB must be provided:

    // Insert a bunch of triangles
    for (int i = 0; i < tris.Length; i++)
    {
        var triangle = tris[i];
        octree.Insert(triangle, triangle.GetAABB());
    }

Often times however, it's more efficient to insert Id's that map to something outside of
the tree (like DOTS entities).

If you know your objects are points, you can insert them faster by using:

    // Insert entities that are 'points'
    for (int i = 0; i < entities.Length; i++)
    {
        octree.InsertPoint(entities[i], positions[i]);
    }

Note that objects inserted as points only support range and nearest neighbour queries.

### Queries

All of the supported queries use the same pattern which is
to (ab)use structs as a sort of delegate. This separates collision/intersection
code from the type of objects, allowing you to insert even primitive types or types from another assembly.
This turned out to be the most efficent and easiest to implement while keeping things fully compatibly with Unity's Burst compiler.

### Raycast

A raycast query for example, requires you to implement IOctreeRayIntersecter which
acts as a delegate to determine if a ray intersects with an object that's in the tree.

    public static bool RaycastAABB<T>(this NativeOctree<T> octree, Ray ray, out OctreeRaycastHit<T> hit) where T : unmanaged
    { 
      return octree.Raycast<RayAABBIntersecter<T>>(ray, out hit);
    }

    struct RayAABBIntersecter<T> : IOctreeRayIntersecter<T>
    {
      public bool IntersectRay(in PrecomputedRay ray, T obj, AABB objBounds, out float distance)
      {
        return objBounds.IntersectsRay(ray, out distance);
      }
    }

The example above just tests the ray against the object's bounds. (See NativeOctreeExtensions) But you could go a step further
and test it against a triangle, a collider and so forth. Note that the tree itself
does not automatically test for Ray-AABB intersections on the objects, so it's usually a good decision to early exit
if the ray doesn't exit with the object's bounds since those checks are cheap.

### Nearest Neighbour
NativeTrees support nearest neighbour queries. You should implement IOctreeNearestVisitor and IOctreeDistanceProvider.

    struct AABBDistanceSquaredProvider<T> : IOctreeDistanceProvider<T> 
    {
        // Just return the distance squared to our bounds
        public float DistanceSquared(float3 point, T obj, AABB bounds) => bounds.DistanceSquared(point);
    }

    struct OctreeNearestAABBVisitor<T> : IOctreeNearestVisitor<T>
    {
        public T nearest;
        public bool found;
        
        public bool OnVist(T obj)
        {
            this.found = true;
            this.nearest = obj;
            
            return false; // immediately stop iterating at first hit
            // if we want the 2nd or 3rd neighbour, we could iterate on and keep track of the count!
        }
    }

The extensions classes show an example of these implementation. But only for AABB's.
If you need more detail on your distance, you can implement your type specific behaviour using these interfaces.

To get the nearest neighbour:

    var visitor = new OctreeNearestAABBVisitor<Entity>();
    octree.Nearest(point, maxDistance, ref visitor, default(AABBDistanceSquaredProvider<Entity>));
    Entity nearestEntity = visitor.nearest;

### Range

Here's an example that adds unique objects that overlap with a range to a hashset:

    public static void RangeAABBUnique<T>(this NativeOctree<T> octree, AABB range, NativeParallelHashSet<T> results) 
      where T : unmanaged, IEquatable<T>
    {
        var vistor = new RangeAABBUniqueVisitor<T>()
        {
            results = results
        };
     
        octree.Range(range, ref vistor);
    }

    struct RangeAABBUniqueVisitor<T> : IOctreeRangeVisitor<T> where T : unmanaged, IEquatable<T>
    {
        public NativeParallelHashSet<T> results;
        
        public bool OnVisit(T obj, AABB objBounds, AABB queryRange)
        {
            // check if our object's AABB overlaps with the query AABB
            if (objBounds.Overlaps(queryRange))
                results.Add(obj);

            return true; // always keep iterating, we want to catch all objects
        }
    }

It's important to note that the query itself iterates all of the objects that are in nodes that overlap with
the input range. An extra check should be performed to test if the object overlaps.
Also, if the objects aren't points, it's possible for them to be visited multiple times as they reside in more than one node.
A hashset can be used to only visit each object once.

### Support
Feel free to raise an issue or contact me for any questions.
The code is free to use in your project(s).
If this was helpful to you, consider buying me a coffee ;)

<a href='https://ko-fi.com/bartofzo' target='_blank'><img height='35' style='border:0px;height:46px;' src='https://az743702.vo.msecnd.net/cdn/kofi3.png?v=0' border='0' alt='Buy Me a Coffee at ko-fi.com' />


Thank you!

### Sources

Fast raycast traversal algorithm:
https://daeken.svbtle.com/a-stupidly-simple-fast-quadtree-traversal-for-ray-intersection

Ray box 'slab' intersection method:
https://tavianator.com/2011/ray_box.html

Nearest neighbour search:
https://stackoverflow.com/a/41306992

AnyPath, my pathfinding library on the Unity Asset store:
https://assetstore.unity.com/packages/tools/ai/anypath-213200
