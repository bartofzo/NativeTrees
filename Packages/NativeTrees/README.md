![NativeTrees](NativeTrees.png "NativeTrees")

Generic sparse octree and quadtree 
that store objects together with their axis aligned bounding boxes (AABB's)

Written in C# for Unity's burst compiler and the ECS/DOTS framework.
Tested with Unity 2021.1.3.11f1

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

### Support
Feel free to raise an issue or contact me for any questions.

### Donate
The code is free to use in your project(s). 
If this was helpful to you and you're feeling generous, you can support me with a donation to help me pay for my energy bills and making
more of these projects in the future ;)

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://paypal.me/bartofzo?country.x=NL&locale.x=nl_NL)

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
