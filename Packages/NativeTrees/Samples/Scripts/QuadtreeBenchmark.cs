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

    public class QuadtreeBenchmark : MonoBehaviour
    {
        private static readonly int[] counts = new[] {1000, 5000, 10000, 25000, 50000, 100000, 250000, 500000, 1000000};
        
        private NativeQuadtree<int> quadtree;
        private NativeArray<float2> randomPoints;
        private NativeArray<AABB2D> randomAABBs;

        private double[] pointTimes;
        private double[] aabbTimes;


        private static readonly float2 boundsExtents = new float2(100, 100);

        private void Start()
        {
            quadtree = new NativeQuadtree<int>(new AABB2D(-boundsExtents, boundsExtents), Allocator.Persistent);
            randomPoints = new NativeArray<float2>(counts[counts.Length - 1], Allocator.Persistent);
            randomAABBs = new NativeArray<AABB2D>(counts[counts.Length - 1], Allocator.Persistent);
            pointTimes = new double[counts.Length];
            aabbTimes = new double[counts.Length];

            for (int i = 0; i < randomPoints.Length; i++)
            {
                randomPoints[i] = new float2(
                    Random.Range(-boundsExtents.x, boundsExtents.x),
                    Random.Range(-boundsExtents.y, boundsExtents.y));
            }

            for (int i = 0; i < randomAABBs.Length; i++)
            {
                var point = randomPoints[i];
                randomAABBs[i] = new AABB2D(
                    point - new float2(Random.value, Random.value),
                    point + new float2(Random.value, Random.value));
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
                quadtree = quadtree
            }.Run();
            
            new InsertAABBsJob()
            {
                until = 0,
                aabbs = randomAABBs,
                quadtree = quadtree
            }.Run();
            
            StringBuilder sb = new StringBuilder();
            while (true)
            {
                sb.Clear();
                
                for (int i = 0; i < counts.Length; i++)
                {
                    quadtree.Clear();
                    var job = new InsertPointsJob()
                    {
                        until = counts[i],
                        positions = randomPoints,
                        quadtree = quadtree
                    };

                    var sw = Stopwatch.StartNew();
                    job.Run();
                    sw.Stop();
                    pointTimes[i] = sw.Elapsed.TotalMilliseconds;
            
                    quadtree.Clear();
                    var job2 = new InsertAABBsJob()
                    {
                        until = counts[i],
                        aabbs = randomAABBs,
                        quadtree = quadtree
                    };

                    sw.Restart();
                    job2.Run();
                    sw.Stop();
                    aabbTimes[i] = sw.Elapsed.TotalMilliseconds;
                    
                    yield return null;
                }

                yield return null;

                sb.AppendLine("Point Insertion:");
                for (int i = 0; i < counts.Length; i++)
                    sb.AppendLine($"{counts[i]}: {pointTimes[i].ToString("N2")}ms");
                
                sb.AppendLine("AABB Insertion:");
                for (int i = 0; i < counts.Length; i++)
                    sb.AppendLine($"{counts[i]}: {aabbTimes[i].ToString("N2")}ms");
                
                Debug.Log(sb);
            }
        }


        private void OnDestroy()
        {
            quadtree.Dispose();
            randomPoints.Dispose();
            randomAABBs.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        struct InsertPointsJob : IJob
        {
            public NativeQuadtree<int> quadtree;
            public NativeArray<float2> positions;
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
            public NativeQuadtree<int> quadtree;
            public NativeArray<AABB2D> aabbs;
            public int until;
            
            public void Execute()
            {
                for (int i = 0; i < until; i++)
                {
                    quadtree.Insert(i, aabbs[i]);
                }
            }
        }
    }
}