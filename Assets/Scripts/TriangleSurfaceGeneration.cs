using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class TriangleSurfaceGeneration : MonoBehaviour
{
    public struct Triangle
    {
        public Vector3[] Vertices;
        public int[] Indices;
        public int[] Neighbours;
        public Vector3 NormalVector;
        public Vector3 BarycentricPosition;
        public float Height;
        
        public Triangle(Vector3 v0, Vector3 v1, Vector3 v2, int i0, int i1, int i2, int t0, int t1, int t2, Vector3 n)
        {
            Vertices = new[] { v0, v1, v2 };
            Indices = new[] { i0, i1, i2 };
            Neighbours = new[] { t0, t1, t2 };
            NormalVector = n;
            BarycentricPosition = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Height = float.MaxValue;
        }

        public void SetBaryAndHeight(Vector3 barycentricPosition, float height)
        {
            BarycentricPosition = barycentricPosition;
            Height = height;
        }
    }
    
    // Stores the maximum and minimum x and z values from point cloud text file
    private struct MaxAndMin
    {
        public float xMax, xMin;
        public float zMax, zMin;

        public MaxAndMin(Vector3[] points)
        {
            xMax = float.MinValue;
            xMin = float.MaxValue;
            zMax = float.MinValue;
            zMin = float.MaxValue;
        
            // Goes through points array to find the maximum and minimum values
            for (int i = 0; i < points.Length; i++)
            {
                if (points[i].x > xMax)
                    xMax = points[i].x;

                if (points[i].x < xMin)
                    xMin = points[i].x;

                if (points[i].z > zMax)
                    zMax = points[i].z;

                if (points[i].z < zMin)
                    zMin = points[i].z;
            }
        }
    }
    
    // Scripts
    [SerializeField] private PointCloudLoader pointCloudLoader;
    
    // World size
    [SerializeField] private int resolution;
    [SerializeField] private int amplitude;
    
    // Mesh
    private Mesh _mesh;
    
    // Triangle information
    private List<Vector3> _vertices;
    private List<int> _indices;
    private List<int> _neighbours;
    [HideInInspector] public List<Triangle> Triangles;
    
    // For finding ball collision
    [HideInInspector] public int currentTriangleIndex;
    [FormerlySerializedAs("height")] [HideInInspector] public float _height;

    private void Start()
    {
        // Array of point cloud points from text file
        Vector3[] points = pointCloudLoader.points;
        
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

        //Debug.Log("xRange: " + xRange + " zRange" + zRange);
        
        // Code for how to create vertices and indices has been taken from ZeroKelvinTutorials on YouTube
        // https://www.youtube.com/watch?v=-3ekimUWb9I&ab_channel=ZeroKelvinTutorials
        
        // Linus Nordbakken also explained to me his method of calculating average height for every quad,
        // so I am using his method to do that
        
        // Creates vertices
        _vertices = new List<Vector3>();
        for (int i = 0, z = 0; z < resolution + 1; z++)
        {
            for (int x = 0; x < resolution + 1; x++)
            {
                // Creates vertex which height value will be the average height of points around it
                Vector2 vertex = new Vector2(x * xSteps, z * zSteps);
                float y = CalculateAverageHeight(points, vertex, xRange, zRange);
                
                _vertices.Add(new Vector3(vertex.x, y * amplitude, vertex.y));
                
                //Debug.Log("Vertices: " + _vertices[i]);
                i++;
            }
        }
        
        // I am using both Bjørn Joachim Olsen's and Anders Petershaugen Åsbø's method for finding neighbouring triangles
        
        // Creates indices and triangle neighbours
        _indices = new List<int>();
        _neighbours = new List<int>();
        Triangles = new List<Triangle>();

        int rowTriangleAmount = 2 * resolution; // Amount of triangles in row
        int totalTriangles = rowTriangleAmount * resolution; // Total amount of triangles
        
        for (int row = 0; row < resolution; row++)
        {
            int currentAmountOfTriangles = 2 * row * resolution;
            
            for (int column = 0; column < resolution; column++)
            {
                int i = (row * resolution) + row + column;
                
                // First triangle
                int i0 = i;
                int i1 = i + resolution + 1;
                int i2 = i + resolution + 2;
                
                _indices.Add(i0);
                _indices.Add(i1);
                _indices.Add(i2);
                
                // This is Anders Petershaugen Åsbø's method of finding neighbours
                // The calculations of finding the neighbours are changed from Anders' because I don't the same indexing,
                // so I am using Linus Nordbakken's way of calculating since we have the same indexing
                int evenTriangle = 2 * (row * resolution + column);
                int oddTriangle = evenTriangle + 1;
                
                // The potential neighbours for first triangle
                int t0 = oddTriangle + rowTriangleAmount;
                int t1 = oddTriangle;
                int t2 = evenTriangle - 1;

                t0 = t0 < totalTriangles ? t0 : -1;
                t1 = t1 < totalTriangles ? t1 : -1;
                t2 = t2 > currentAmountOfTriangles ? t2 : -1;

                _neighbours.Add(t0);
                _neighbours.Add(t1);
                _neighbours.Add(t2);
                
                // Adds first triangle to list of triangles
                Vector3 tempNormalVector = CalculateNormalVector(_vertices[i0], _vertices[i1], _vertices[i2]);
                Triangles.Add(new Triangle(_vertices[i0], _vertices[i1], _vertices[i2], i0, i1, i2, t0, t1, t2, tempNormalVector));
                
                // Second triangle
                i0 = i;
                i1 = i + resolution + 2;
                i2 = i + 1;
                
                _indices.Add(i0);
                _indices.Add(i1);
                _indices.Add(i2);

                // The potential neighbours for second triangle
                t0 = oddTriangle + 1;
                t1 = evenTriangle - rowTriangleAmount;
                t2 = evenTriangle;
                
                t0 = t0 < currentAmountOfTriangles + rowTriangleAmount ? t0 : -1;
                t1 = t1 >= 0 ? t1 : -1;
                t2 = t2 >= 0 ? t2 : -1;

                _neighbours.Add(t0);
                _neighbours.Add(t1);
                _neighbours.Add(t2);
                
                // Adds second triangle in list of triangles
                tempNormalVector = CalculateNormalVector(_vertices[i0], _vertices[i1], _vertices[i2]);
                Triangles.Add(new Triangle(_vertices[i0], _vertices[i1], _vertices[i2], i0, i1, i2, t0, t1, t2, tempNormalVector));
            }
        }
        
        // Debug.Log("Triangle count: " + Triangles.Count);
        //
        // for (int i = 0; i < Triangles.Count; i++)
        // {
        //     for (int j = 0; j < Triangles[i].neighbours.Length; j++)
        //     {
        //         Debug.Log("Neighbour triangle index for triangle nr " + i + ": " + Triangles[i].neighbours[j]);
        //     }
        // }
    }
    
    private void UpdateMesh()
    {
        // Makes sure no other meshes of mesh variable has been created
        _mesh.Clear();

        // Assigns triangles and vertices for mesh
        _mesh.vertices = _vertices.ToArray();
        _mesh.triangles = _indices.ToArray();
        
        // Recalculates normals of triangles in mesh
        _mesh.RecalculateNormals();
    }

    // Finds average height in a quad and uses that height value for given vertex point
    // I am using Linus Nordbakken's method of calculating the average height
    private float CalculateAverageHeight(Vector3[] points, Vector2 vertex, float xRange, float zRange)
    {
        float xHalfStep = xRange * 0.5f;
        float zHalfStep = zRange * 0.5f;
        
        // The four vertices in one quad
        Vector2 topLeft = new Vector2(vertex.x - xHalfStep, vertex.y + zHalfStep);
        Vector2 topRight = new Vector2(vertex.x  + xHalfStep, vertex.y + zHalfStep);
        Vector2 bottomLeft = new Vector2(vertex.x - xHalfStep, vertex.y - zHalfStep);
        Vector2 bottomRight = new Vector2(vertex.x + xHalfStep, vertex.y - zHalfStep);

        // List of all height values in the points that are in the quad
        List<float> heightValues = new List<float>();

        // Finds points that are inside the two triangles
        for (int i = 0; i < points.Length; i++)
        {
            // Finds points inside first triangle
            Vector3 barycentric = GetBarycentricCoordinates(bottomLeft, topLeft, topRight, new Vector2(points[i].x, points[i].z));

            if (barycentric is { x: >= 0, y: >= 0, z: >= 0 })
            {
                heightValues.Add(points[i].y);
            }
            // Finds points inside second triangle
            else
            {
                barycentric = GetBarycentricCoordinates(bottomLeft, topRight, bottomRight, new Vector2(points[i].x, points[i].z));
                
                if (barycentric is { x: >= 0, y: >= 0, z: >= 0 })
                    heightValues.Add(points[i].y);
            }
        }

        // Calculates average height value
        float average = 0;

        //Debug.Log("Height values count: " + heightValues.Count);

        if (heightValues.Count == 0)
            return 0;
        
        for (int i = 0; i < heightValues.Count; i++)
            average += heightValues[i];

        average /= heightValues.Count;
        
        // Returns average height value of the points that are inside the two triangles
        return average;
    }

    // Finds what triangle the ball starts in
    public int FindFirstTriangleIndex(Vector2 ballPosition)
    {
        Vector3 barycentricCoordinates = new Vector3();
        Vector3 v0 = new Vector3();
        Vector3 v1 = new Vector3();
        Vector3 v2 = new Vector3();
        
        // Searches through all of the triangles until the ball is found
        //Debug.Log("Triangles amount: " + Triangles.Count);
        for (int i = 0; i < Triangles.Count; i++)
        {
            v0 = Triangles[i].Vertices[0];
            v1 = Triangles[i].Vertices[1];
            v2 = Triangles[i].Vertices[2];
            
            barycentricCoordinates = GetBarycentricCoordinates(new Vector2(v0.x, v0.z), new Vector2(v1.x, v1.z),
                new Vector2(v2.x, v2.z), ballPosition);
            
            // Sets the current triangle index value for the triangle that the ball is at the start
            if (barycentricCoordinates is { x: >= 0, y: >= 0, z: >= 0 })
            {
                //Debug.Log("Start triangle: " + i);
                return i;
            }
        }

        Debug.Log("No triangle found");
        return int.MaxValue;
    }
    
    // Updates which triangle the ball is in, and returns the world coordinates from the barycentric coordinates
    public Vector3 UpdateTriangleIndex(Vector2 ballPosition)
    {
        Vector3 barycentricCoordinates = new Vector3();
        
        // Finds the three vertices for the current triangle the ball is inside
        Vector3 v0 = Triangles[currentTriangleIndex].Vertices[0];
        Vector3 v1 = Triangles[currentTriangleIndex].Vertices[1];
        Vector3 v2 = Triangles[currentTriangleIndex].Vertices[2];

        barycentricCoordinates = GetBarycentricCoordinates(new Vector2(v0.x, v0.z), new Vector2(v1.x, v1.z),
            new Vector2(v2.x, v2.z), ballPosition);

        // If ball is inside current triangle
        if (barycentricCoordinates is { x: >= 0, y: >= 0, z: >= 0 })
        {
            // Sets height value
            _height = barycentricCoordinates.x * v0.y + barycentricCoordinates.y * v1.y +
                     barycentricCoordinates.z * v2.y;
            
            // Converts and returns barycentric coordinates to world coordinates
            return barycentricCoordinates.x * v0 + barycentricCoordinates.y * v1 + barycentricCoordinates.z * v2;
        }
        
        // Searches through current triangle's neighbouring triangles if ball has moved out of current triangle
        for (int i = 0; i < Triangles[currentTriangleIndex].Neighbours.Length; i++)
        {
            // Changes current triangle index to be one of the neighbouring triangles
            int tempTriangleIndex = Triangles[currentTriangleIndex].Neighbours[i];

            //Debug.Log("Temp triangle index: " + tempTriangleIndex);
            
            if (tempTriangleIndex != -1) // Makes sure to only check triangles that exist
            {
                v0 = Triangles[tempTriangleIndex].Vertices[0];
                v1 = Triangles[tempTriangleIndex].Vertices[1];
                v2 = Triangles[tempTriangleIndex].Vertices[2];
                
                barycentricCoordinates = GetBarycentricCoordinates(new Vector2(v0.x, v0.z), new Vector2(v1.x, v1.z),
                    new Vector2(v2.x, v2.z), ballPosition);
                
                // If ball is inside current triangle
                if (barycentricCoordinates is { x: >= 0, y: >= 0, z: >= 0 })
                {
                    // Sets height value
                    currentTriangleIndex = tempTriangleIndex;
                    _height = barycentricCoordinates.x * v0.y + barycentricCoordinates.y * v1.y +
                             barycentricCoordinates.z * v2.y;
            
                    // Converts and returns barycentric coordinates to world coordinates
                    return barycentricCoordinates.x * v0 + barycentricCoordinates.y * v1 + barycentricCoordinates.z * v2;
                }
            }
        }
        
        // If ball is outside mesh bounds
        Debug.Log("No triangles found!");
        return new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
    }
    
    public int UpdateTriangleIndex(Vector2 ballPosition, int triangleIndex)
    {
        Vector3 barycentricCoordinates = new Vector3();
        float height;
        Vector3 barycentricPosition = new Vector3();

        Triangle tempTriangle = new Triangle();

        // Finds the three vertices for the current triangle the ball is inside
        Vector3 v0 = Triangles[triangleIndex].Vertices[0];
        Vector3 v1 = Triangles[triangleIndex].Vertices[1];
        Vector3 v2 = Triangles[triangleIndex].Vertices[2];

        barycentricCoordinates = GetBarycentricCoordinates(new Vector2(v0.x, v0.z), new Vector2(v1.x, v1.z),
            new Vector2(v2.x, v2.z), ballPosition);

        //Debug.Log("Barycentric Coordinates: " + barycentricCoordinates);

        // If ball is inside current triangle
        if (barycentricCoordinates is { x: >= 0, y: >= 0, z: >= 0 })
        {
            tempTriangle = Triangles[triangleIndex];

            // Converts barycentric coordinates to height value and world coordinates
            barycentricPosition = barycentricCoordinates.x * v0 + barycentricCoordinates.y * v1 +
                                  barycentricCoordinates.z * v2;
            height = barycentricCoordinates.x * v0.y + barycentricCoordinates.y * v1.y +
                     barycentricCoordinates.z * v2.y;

            // Reassigns height and barycentric position value in struct
            tempTriangle.SetBaryAndHeight(barycentricPosition, height);

            Triangles[triangleIndex] = tempTriangle;
            //Debug.Log("Barycentric Position: " + barycentricPosition);

            return triangleIndex;
        }

        // Searches through current triangle's neighbouring triangles if ball has moved out of current triangle
        for (int i = 0; i < Triangles[triangleIndex].Neighbours.Length; i++)
        {
            // Changes current triangle index to be one of the neighbouring triangles
            int tempTriangleIndex = Triangles[triangleIndex].Neighbours[i];

            //Debug.Log("Neighbour number " + i + " index: " + tempTriangleIndex);

            //Debug.Log("Temp triangle index: " + tempTriangleIndex);

            if (tempTriangleIndex != -1) // Makes sure to only check triangles that exist
            {
                v0 = Triangles[tempTriangleIndex].Vertices[0];
                v1 = Triangles[tempTriangleIndex].Vertices[1];
                v2 = Triangles[tempTriangleIndex].Vertices[2];

                barycentricCoordinates = GetBarycentricCoordinates(new Vector2(v0.x, v0.z), new Vector2(v1.x, v1.z),
                    new Vector2(v2.x, v2.z), ballPosition);

                // If ball is inside current triangle
                if (barycentricCoordinates is { x: >= 0, y: >= 0, z: >= 0 })
                {
                    tempTriangle = Triangles[triangleIndex];

                    // Converts barycentric coordinates to height value and world coordinates
                    barycentricPosition = barycentricCoordinates.x * v0 + barycentricCoordinates.y * v1 +
                                          barycentricCoordinates.z * v2;
                    height = barycentricCoordinates.x * v0.y + barycentricCoordinates.y * v1.y +
                             barycentricCoordinates.z * v2.y;

                    // Reassigns height and barycentric position value in struct
                    tempTriangle.SetBaryAndHeight(barycentricPosition, height);

                    Triangles[triangleIndex] = tempTriangle;
                    //Debug.Log("Barycentric Position: " + barycentricPosition);

                    return tempTriangleIndex;
                }
            }
        }

        // If ball is outside mesh bounds
        Debug.Log("No triangles found!");
        return int.MaxValue;
    }
    
    private Vector3 GetBarycentricCoordinates(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 vertex)
    {
        // Finds vectors in triangle
        Vector2 v21 = p2 - p1;
        Vector2 v31 = p3 - p1;
        Vector2 vp = vertex - p1; // Vector between vertex point and first point in triangle
        
        // Calculates area
        float area = v21.x * v31.y - v31.x * v21.y; 
        
        float w = (v21.x * vp.y - vp.x * v21.y) / area;
        float v = (vp.x * v31.y - v31.x * vp.y) / area;
        float u = 1.0f - v - w; // u + v + w = 1
        
        // Returns barycentric coordinates
        return new Vector3(u, v, w);
    }

    private Vector3 CalculateNormalVector(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        // Calculates two vectors along triangle's edge
        Vector3 v1 = p2 - p1;
        Vector3 v2 = p3 - p1;

        // Calculates and normalises the cross product of the two given vectors to get the normal vector
        return Vector3.Cross(v1, v2).normalized;
    }
}
