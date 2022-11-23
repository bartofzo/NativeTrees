using System;
using System.Collections;
using System.Diagnostics;
using System.Text;
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

    public class OctreeBenchmark : MonoBehaviour
    {
        private const int RaycastCount = 1000;
        private static readonly int[] counts = new[] {1000, 5000, 10000, 25000, 50000, 100000, 250000, 500000, 1000000};
        
        private NativeOctree<int> octree;
        private NativeArray<float3> randomPoints;
        private NativeArray<AABB> randomAABBs;
        private NativeArray<Ray> randomRays;

        private double[] pointTimes;
        private double[] aabbTimes;
        private double[] rayTimes;


        private static readonly float3 boundsExtents = new float3(100, 100, 100);

        private void Start()
        {
            octree = new NativeOctree<int>(new AABB(-boundsExtents, boundsExtents), Allocator.Persistent);
            randomPoints = new NativeArray<float3>(counts[counts.Length - 1], Allocator.Persistent);
            randomAABBs = new NativeArray<AABB>(counts[counts.Length - 1], Allocator.Persistent);
            randomRays = new NativeArray<Ray>(RaycastCount, Allocator.Persistent);

            pointTimes = new double[counts.Length];
            aabbTimes = new double[counts.Length];
            rayTimes = new double[counts.Length];

            for (int i = 0; i < randomPoints.Length; i++)
            {
                // Slightly more realistic division, such that not all octants are divided up the same
                randomPoints[i] = new float3(
                    Random.Range(0, boundsExtents.x),
                    Random.Range(-boundsExtents.y, 0),
                    Random.Range(0, boundsExtents.z));
            }

            for (int i = 0; i < randomAABBs.Length; i++)
            {
                var point = randomPoints[i];
                randomAABBs[i] = new AABB(
                    point - new float3(Random.value, Random.value, Random.value),
                    point + new float3(Random.value, Random.value, Random.value));
            }
            
            for (int i = 0; i < randomRays.Length; i++)
            {
                randomRays[i] = new Ray(new Vector3(
                    Random.Range(-boundsExtents.x, boundsExtents.x),
                    Random.Range(-boundsExtents.y, boundsExtents.y),
                    Random.Range(-boundsExtents.z, boundsExtents.z)), Random.onUnitSphere);
            }

            StartCoroutine(Run());
        }

        IEnumerator Run()
        {
            // warm up compilation
            new InsertPointsJob()
            {
                until = 0,
                positions = randomPoints,
                quadtree = octree
            }.Run();
            
            new InsertAABBsJob()
            {
                until = 0,
                aabbs = randomAABBs,
                octree = octree
            }.Run();
            
            StringBuilder sb = new StringBuilder();
            while (true)
            {
                sb.Clear();
                
                for (int i = 0; i < counts.Length; i++)
                {
                    octree.Clear();
                    var job = new InsertPointsJob()
                    {
                        until = counts[i],
                        positions = randomPoints,
                        quadtree = octree
                    };

                    var sw = Stopwatch.StartNew();
                    job.Run();
                    sw.Stop();
                    pointTimes[i] = sw.Elapsed.TotalMilliseconds;
            
                    octree.Clear();
                    var job2 = new InsertAABBsJob()
                    {
                        until = counts[i],
                        aabbs = randomAABBs,
                        octree = octree
                    };

                    sw.Restart();
                    job2.Run();
                    sw.Stop();
                    aabbTimes[i] = sw.Elapsed.TotalMilliseconds;

                    
                    // raycast
                    NativeReference<int> count = new NativeReference<int>(Allocator.TempJob);
                    var job3 = new RaycastJob()
                    {
                        rays = randomRays,
                        octree = octree,
                        count = count
                    };

                    sw.Restart();
                    job3.Run();
                    sw.Stop();
                    rayTimes[i] = sw.Elapsed.TotalMilliseconds;
                    count.Dispose();
                    
                    yield return null;
                }

                yield return null;

                sb.AppendLine("Point Insertion:");
                for (int i = 0; i < counts.Length; i++)
                    sb.AppendLine($"{counts[i]}: {pointTimes[i].ToString("N2")}ms");
                
                sb.AppendLine("AABB Insertion:");
                for (int i = 0; i < counts.Length; i++)
                    sb.AppendLine($"{counts[i]}: {aabbTimes[i].ToString("N2")}ms");
                
                sb.AppendLine($"{RaycastCount} Raycasts:");
                for (int i = 0; i < counts.Length; i++)
                    sb.AppendLine($"{counts[i]}: {rayTimes[i].ToString("N2")}ms");
                
                
                Debug.Log(sb);
            }
        }


        private void OnDestroy()
        {
            octree.Dispose();
            randomPoints.Dispose();
            randomAABBs.Dispose();
            randomRays.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        struct InsertPointsJob : IJob
        {
            public NativeOctree<int> quadtree;
            public NativeArray<float3> positions;
            public int until;
            
            public void Execute()
            {
                for (int i = 0; i < until; i++)
                {
                    quadtree.InsertPoint(i, positions[i]);
                }
            }
        }
        
        [BurstCompile(CompileSynchronously = true)]
        struct InsertAABBsJob : IJob
        {
            public NativeOctree<int> octree;
            public NativeArray<AABB> aabbs;
            public int until;
            
            public void Execute()
            {
                for (int i = 0; i < until; i++)
                {
                    octree.Insert(i, aabbs[i]);
                }
            }
        }
        
        [BurstCompile(CompileSynchronously = true)]
        struct RaycastJob : IJob
        {
            public NativeOctree<int> octree;
            public NativeArray<Ray> rays;
            public NativeReference<int> count;

            public void Execute()
            {
                int c = 0;
                for (int i = 0; i < rays.Length; i++)
                {
                    if (octree.RaycastAABB(rays[i], out _, maxDistance: 25))
                    {
                        c++;
                    }
                }

                count.Value = c;
            }
        }
    }
}