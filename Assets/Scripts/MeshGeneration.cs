using System;
using System.Collections.Generic;
using UnityEngine;

public class MeshGeneration : MonoBehaviour
{
    // Scripts
    private PointCloudLoader _pointCloudLoader;
    
    // World size
    //[SerializeField] private int worldX;
    //[SerializeField] private int worldZ;
    [SerializeField] private int resolution;
    
    // Mesh
    private Mesh _mesh;
    
    // Vertices and triangles
    private int[] _triangles;
    //private Vector3[] _vertices;
    private List<Vector3> _vertices;

    private void Start()
    {
        _pointCloudLoader = FindObjectOfType<PointCloudLoader>();
        
        GenerateMesh();
        UpdateMesh();
    }

    private void GenerateMesh()
    {
        // Initializes mesh variable
        _mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = _mesh;

        // Assigns size of arrays
        // _triangles = new int[worldX * worldZ * 6];
        // _vertices = new Vector3[(worldX + 1) * (worldZ + 1)];
        
        _triangles = new int[resolution * resolution * 6];
        //_vertices = new Vector3[(resolution + 1) * (resolution + 1)];
        _vertices = new List<Vector3>();

        // Assigns the vertices
        for (int i = 0, z = 0; z < resolution + 1; z++)
        {
            for (int x = 0; x < resolution + 1; x++)
            {
                _vertices.Add(_pointCloudLoader.points[i]);
                i++;
            }
        }
        
        _vertices.Sort();

        for (int i = 0; i < 10; i++)
        {
            Debug.Log("Position: " + _vertices[i]);
        }

        int tris = 0;
        int verts = 0;

        // Creates quads, each having two triangles
        for (int z = 0; z < resolution; z++)
        {
            for (int x = 0; x < resolution; x++)
            {
                _triangles[tris] = verts;
                _triangles[tris + 1] = verts + resolution + 1;
                _triangles[tris + 2] = verts + 1;

                _triangles[tris + 3] = verts + 1;
                _triangles[tris + 4] = verts + resolution + 1;
                _triangles[tris + 5] = verts + resolution + 2;

                verts++;
                tris += 6;
            }

            verts++;
        }
    }

    private void UpdateMesh()
    {
        // Makes sure no other meshes of mesh variable has been created
        _mesh.Clear();

        // Assigns triangles and vertices for mesh
        _mesh.vertices = _vertices.ToArray();
        _mesh.triangles = _triangles;
        
        // Recalculates normals of triangles in mesh
        _mesh.RecalculateNormals();
    }
}
