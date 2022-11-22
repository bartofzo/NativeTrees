using System;
using System.Collections.Generic;
using NativeTrees;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Example
{
    public class NativeQuadtreeExample : MonoBehaviour
    {
        public int squareCount = 100;
        public GameObject squarePrefab;
        public Camera camera;
        
        private NativeQuadtree<int> quadtree;
        private Dictionary<int, QuadtreeSquare> squares = new Dictionary<int, QuadtreeSquare>();
        private NativeList<int> overlap = new NativeList<int>();
        private int prevNearest = -1;
        private Vector2 mouseDownPos;
        
        private void Start()
        {
            Vector2 camPos = camera.transform.position;
            Vector2 camExtents = .5f * new Vector2(camera.orthographicSize * camera.aspect, camera.orthographicSize);

            var bounds = new AABB2D(camPos - camExtents, camPos + camExtents);
            quadtree = new NativeQuadtree<int>(bounds, Allocator.Persistent);

            overlap = new NativeList<int>(Allocator.Persistent);
            
            for (int i = 0; i < squareCount; i++)
            {
                Vector2 pos = new Vector2(Random.Range(bounds.min.x, bounds.max.x), Random.Range(bounds.min.y, bounds.max.y));
                var square = Instantiate(squarePrefab, pos, Quaternion.identity).GetComponent<QuadtreeSquare>();
                
                quadtree.Insert(square.GetInstanceID(), square.Bounds);
                squares.Add(square.GetInstanceID(), square);
            }
        }

        private void OnDestroy()
        {
            overlap.Dispose();
            quadtree.Dispose();
        }

        private void Update()
        {
            Vector2 mousePos = camera.ScreenToWorldPoint(Input.mousePosition);
            if (Input.GetMouseButtonDown(0))
                mouseDownPos = mousePos;

            if (Input.GetMouseButton(0))
            {
                overlap.Clear();
                quadtree.RangeAABB(new AABB2D(Vector2.Min(mouseDownPos, mousePos), Vector2.Max(mouseDownPos, mousePos)), overlap);
            }

            if (quadtree.TryGetNearestAABB(mousePos, 100, out int nearestId))
            {
                if (squares.TryGetValue(prevNearest, out var prev))
                {
                    prev.Color = Color.white;
                }
                
                squares[nearestId].Color = Color.red;
                prevNearest = nearestId;
            }
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
                return;
            
            // Draw blue boxes around range overlap
            Gizmos.color = Color.blue;
            for (int i = 0; i < overlap.Length; i++)
            {
                if (!squares.TryGetValue(overlap[i], out var square))
                    continue;

                var size = square.Bounds.Size;

                Gizmos.DrawCube(square.transform.position, 1.25f * new Vector3(size.x, size.y));
            }
            
            // Green box for raycast
            Vector2 mousePos = camera.ScreenToWorldPoint(Input.mousePosition);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(mouseDownPos, mousePos);
            if (quadtree.RaycastAABB(new Ray2D(mouseDownPos, (mousePos - mouseDownPos).normalized), out var hit))
            {
                if (squares.TryGetValue(hit.obj, out var square))
                {
                    var size = square.Bounds.Size;
                    Gizmos.DrawWireCube(square.transform.position, 1.25f * new Vector3(size.x, size.y));
                }
            }

            Gizmos.color = Color.black;
            quadtree.DrawGizmos();
        }
    }
}