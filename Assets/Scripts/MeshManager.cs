using System;
using UnityEngine;

public class MeshManager : MonoBehaviour
{
    // Scripts
    //[SerializeField] private PointCloudLoader pointCloudLoader;
    [SerializeField] private GameObject pointCloud;
    private PointCloudLoader _pointCloudScript;
    
    // Materials
    [SerializeField] private Material blue;

    private void Start()
    {
        _pointCloudScript = pointCloud.GetComponent<PointCloudLoader>();   
        GenerateTriangleMesh(_pointCloudScript.vertices, _pointCloudScript.indices);
        
        Debug.Log("Vertex: " + _pointCloudScript.vertices[5]);
        Debug.Log("Index: " + _pointCloudScript.indices[5]);
    }

    // Generates a mesh using vertices and indices from .txt file
    private void GenerateTriangleMesh(Vector3[] vertices, int[] indices)
    {
        gameObject.AddComponent<MeshFilter>();
        gameObject.AddComponent<MeshRenderer>();

        var triangleMesh = new Mesh()
        {
            vertices = vertices,
            triangles = indices
        };

        GetComponent<MeshFilter>().mesh = triangleMesh;
        GetComponent<MeshRenderer>().material = blue;
    }
}
