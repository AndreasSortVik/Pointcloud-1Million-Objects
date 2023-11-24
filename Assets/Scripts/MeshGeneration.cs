using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Burst.Intrinsics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;

public class MeshGeneration : MonoBehaviour
{
    // Stores maximum and minimum values for x and z
    private struct MaxAndMin
    {
        public float xMax, xMin;
        public float zMax, zMin;

        public MaxAndMin(Vector3[] points)
        {
            float xTempMax = float.MinValue;
            float xTempMin = float.MaxValue;
            float zTempMax = float.MinValue;
            float zTempMin = float.MaxValue;
        
            // Goes through points array to find the maximum and minimum values for x and z axis
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 tempPoint = points[i];

                if (tempPoint.x > xTempMax)
                    xTempMax = tempPoint.x;

                if (tempPoint.x < xTempMin)
                    xTempMin = tempPoint.x;

                if (tempPoint.z > zTempMax)
                    zTempMax = tempPoint.z;

                if (tempPoint.z < zTempMin)
                    zTempMin = tempPoint.z;
            }

            xMax = xTempMax;
            xMin = xTempMin;
            zMax = zTempMax;
            zMin = zTempMin;
        }
    }
    
    // Scripts
    private PointCloudLoader _pointCloudLoader;
    
    // World size
    [SerializeField] private int resolution;
    
    // Mesh
    private Mesh _mesh;
    
    // Vertices and triangles
    private List<Vector3> _vertices;
    private List<int> _triangles;

    private void Start()
    {
        _pointCloudLoader = FindObjectOfType<PointCloudLoader>();
        
        // Array of point cloud points from text file
        Vector3[] points = _pointCloudLoader.points;

        GenerateTerrain(points);
        UpdateMesh();
    }
    
    private void GenerateTerrain(Vector3[] points)
    {
        // Initializes mesh variable
        _mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = _mesh;
        
        // Gets struct
        MaxAndMin maxAndMin = new MaxAndMin(points);
        
        // Calculates range for x and z value
        float xRange = maxAndMin.xMax - maxAndMin.xMin;
        float zRange = maxAndMin.zMax - maxAndMin.zMin;
        
        // Calculates amount of steps for x and z range
        float xSteps = xRange / resolution;
        float zSteps = zRange / resolution;

        Debug.Log("xRange: " + xRange + " zRange" + zRange);
        
        // Creates vertices
        _vertices = new List<Vector3>();
        for (int i = 0, z = 0; z < resolution + 1; z++)
        {
            for (int x = 0; x < resolution + 1; x++)
            {
                // Creates vertex which height value will be the average height of points around it
                Vector2 vertex = new Vector2(x * xSteps, z * zSteps);
                float y = CalculateAverageHeight(points, vertex, xRange, zRange);
                
                _vertices.Add(new Vector3(vertex.x, y, vertex.y));
                
                Debug.Log("Vertices: " + _vertices[i]);
                i++;
            }
        }
        
        // Creates triangles
        _triangles = new List<int>();
        for (int row = 0; row < resolution; row++)
        {
            for (int column = 0; column < resolution; column++)
            {
                int i = (row * resolution) + row + column;
                
                // First triangle
                _triangles.Add(i);
                _triangles.Add(i + resolution + 1);
                _triangles.Add(i + resolution + 2);
                
                // Second triangle
                _triangles.Add(i);
                _triangles.Add(i + resolution + 2);
                _triangles.Add(i + 1);
            }
        }
    }

    // Finds average height in a quad and uses that height value for given vertex point
    private float CalculateAverageHeight(Vector3[] points, Vector2 vertex, float xRange, float zRange)
    {
        float xHalfStep = xRange * 0.5f;
        float zHalfStep = zRange * 0.5f;
        
        Vector2 topLeft = new Vector2(vertex.x - xHalfStep, vertex.y + zHalfStep);
        Vector2 topRight = new Vector2(vertex.x  + xHalfStep, vertex.y + zHalfStep);
        Vector2 bottomLeft = new Vector2(vertex.x - xHalfStep, vertex.y - zHalfStep);
        Vector2 bottomRight = new Vector2(vertex.x + xHalfStep, vertex.y - zHalfStep);

        List<float> heightValues = new List<float>();

        // Finds points that are inside the two triangles
        for (int i = 0; i < points.Length; i++)
        {
            // Finds point inside first triangle
            Vector3 barycentric = GetBarycentricCoordinates(bottomLeft, topLeft, topRight, new Vector2(points[i].x, points[i].z));

            if (barycentric is { x: >= 0, y: >= 0, z: >= 0 })
            {
                heightValues.Add(points[i].y);
            }
            // Finds point inside second triangle
            else
            {
                barycentric = GetBarycentricCoordinates(bottomLeft, topRight, bottomRight, new Vector2(points[i].x, points[i].z));
                
                if (barycentric is { x: >= 0, y: >= 0, z: >= 0 })
                    heightValues.Add(points[i].y);
            }
        }

        // Calculates average height value
        float average = 0;

        Debug.Log("Height values count: " + heightValues.Count);

        if (heightValues.Count == 0)
            return 0;
        
        for (int i = 0; i < heightValues.Count; i++)
        {
            average += heightValues[i];
        }

        average = average / heightValues.Count;
        
        // Returns average height value of the points that are inside the two triangles
        return average;
    }
    
    private Vector3 GetBarycentricCoordinates(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 vertex)
    {
        // Finds vectors in triangle
        Vector2 v21 = p2 - p1;
        Vector2 v31 = p3 - p1;
        Vector2 v1 = vertex - p1; // Vector between vertex point and first point in triangle

        // Calculates denom
        float d00 = Vector2.Dot(v21, v21);
        float d01 = Vector2.Dot(v21, v31);
        float d11 = Vector2.Dot(v31, v31);
        float d20 = Vector2.Dot(v1, v21);
        float d21 = Vector2.Dot(v1, v31);
        float denom = d00 * d11 - d01 * d01;
        
        // Calculates u, v, and w values
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1f - v - w;

        return new Vector3(u, v, w);
    }

    private void UpdateMesh()
    {
        // Makes sure no other meshes of mesh variable has been created
        _mesh.Clear();

        // Assigns triangles and vertices for mesh
        _mesh.vertices = _vertices.ToArray();
        _mesh.triangles = _triangles.ToArray();
        
        // Recalculates normals of triangles in mesh
        _mesh.RecalculateNormals();
    }
}
