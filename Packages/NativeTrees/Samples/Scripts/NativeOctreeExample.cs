using System;
using System.Diagnostics;
using NativeTrees;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

namespace NativeTrees.Samples
{
    public class NativeOctreeExample : MonoBehaviour
    {
        [BurstCompile(CompileSynchronously = true)]
        struct PopulateJob : IJob
        {
            public NativeArray<Triangle> actualTriangles;
            public NativeArray<AABB> actualBounds;

            public NativeOctree<Triangle> octree;

            public void Execute()
            {
                for (int i = 0; i < actualTriangles.Length; i++)
                    octree.Insert(actualTriangles[i], actualBounds[i]);
            }
        }

        [SerializeField] private MeshFilter meshFilter;
        [SerializeField] private new Camera camera;
        [SerializeField] private Transform hitTransform;
        [SerializeField] private Transform nearestTransform;

        private NativeOctree<Triangle> octree;
        private NativeList<Triangle> list;
        private NativeOctree<Triangle>.NearestNeighbourCache query;

        private void Start()
        {
            var mesh = meshFilter.mesh;

            octree = new NativeOctree<Triangle>(mesh.bounds, 32, 10, Allocator.Persistent);
            list = new NativeList<Triangle>(Allocator.Persistent);
            query = new NativeOctree<Triangle>.NearestNeighbourCache(Allocator.Persistent);

            var meshTriangles = mesh.triangles;
            var meshVertices = mesh.vertices;

            var sw = Stopwatch.StartNew();
            var triangles =
                new NativeArray<Triangle>(meshTriangles.Length / 3, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            var bounds = new NativeArray<AABB>(triangles.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            int n = 0;
            for (int i = 0; i < meshTriangles.Length; i += 3)
            {
                Triangle tri = new Triangle(
                    meshVertices[meshTriangles[i]],
                    meshVertices[meshTriangles[i + 1]],
                    meshVertices[meshTriangles[i + 2]], n);

                bounds[n] = tri.GetAABB();
                triangles[n++] = tri;
            }

            Debug.Log($"Triangles converted in {sw.Elapsed.TotalMilliseconds}ms");
            sw.Restart();
            var job = new PopulateJob()
            {
                octree = octree,
                actualBounds = bounds,
                actualTriangles = triangles
            };
            job.Run();

            Debug.Log($"Octree constructed in {sw.Elapsed.TotalMilliseconds}ms");

            triangles.Dispose();
            bounds.Dispose();
        }

        private void OnDestroy()
        {
            octree.Dispose();
            list.Dispose();
            query.Dispose();
        }

        private void Update()
        {
            var ray = camera.ScreenPointToRay(Input.mousePosition);

            var sw = Stopwatch.StartNew();
            bool didHit = octree.Raycast<TriangleRayIntersecter>(ray, out var hit);


            if (didHit)
            {
                Debug.Log($"(Non burst) Raycast performed in {sw.Elapsed.TotalMilliseconds}ms");
                hitTransform.position = hit.point;
                hitTransform.up = hit.obj.Normal;

                /*
                list.Clear();
                sw.Restart();
                octree.RangeAABB(new AABB(hit.point - new float3(5,5,5), hit.point + new float3(5,5,5)), list);
                Debug.Log($"(Non burst) Range performed in {sw.Elapsed.TotalMilliseconds}ms");
                Debug.Log($"Range count {list.Length}");
                */
            }

            // Pick a random point for our NN query
            /*
            float3 point = new float3(
                Random.Range(octree.Bounds.min.x, octree.Bounds.max.x),
                Random.Range(octree.Bounds.min.y, octree.Bounds.max.y),
                Random.Range(octree.Bounds.min.z, octree.Bounds.max.z));
    
            float maxDistance = 100;
    
            sw.Restart();
            if (query.TryGetNearestAABB(ref octree, point, maxDistance, out var nearestTri))
            {
                Debug.Log($"(Non burst) Nearest performed in {sw.Elapsed.TotalMilliseconds}ms");
                nearestTransform.position = nearestTri.Centroid;
                nearestTransform.up = nearestTri.Normal;
            }
    
            NativeReference<Triangle> tempResult = new NativeReference<Triangle>(Allocator.TempJob);
            var newJob = new NewNearestJob()
            {
                octree = octree,
                point = point,
                queryCache = query,
                nearest = tempResult
            };
            sw.Restart();
            newJob.Run();
            Debug.Log($"(Burst) nearest performed in {sw.Elapsed.TotalMilliseconds}ms");
            tempResult.Dispose();
            */
        }

        /*
        [BurstCompile]
        struct NewNearestJob : IJob
        {
            public float3 point;
            public NativeOctree<Triangle> octree;
            public NativeOctree<Triangle>.NearestNeighbourCache queryCache;
            public NativeReference<Triangle> nearest;
            
            public void Execute()
            {
                queryCache.TryGetNearestAABB(ref octree, point, 50, out var nearestTri);
                nearest.Value = nearestTri;
            }
        }
        */

        private void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                Gizmos.color = Color.black;
                octree.DrawGizmos();
            }
        }
    }
}