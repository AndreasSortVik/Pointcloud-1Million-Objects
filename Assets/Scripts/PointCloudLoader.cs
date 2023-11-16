using System;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.SocialPlatforms.GameCenter;

public class PointCloudLoader : MonoBehaviour
{
    [SerializeField] private Mesh mesh;
    [SerializeField] private Material material;

    private Matrix4x4[] _matrices;
    private int _lineCount = 963101;
    private RenderParams _rp;

    private void Start()
    {
        ReadFile();
    }

    private void ReadFile()
    {
        int counter = 0;
        string filePath = Environment.CurrentDirectory + "//Assets//merged.txt";
        
        // Code on how to center all the objects to origa are taken from Linus Nordbakken: https://github.com/Lolinonusos/Skoleunity/blob/main/Assets/Vissim/PunktskyRender.cs
        float xMin = float.MaxValue;
        float xMax = float.MinValue;
        
        float yMin = float.MaxValue;
        float yMax = float.MinValue;
        
        float zMin = float.MaxValue;
        float zMax = float.MinValue;
        
        
        
        if (File.Exists(filePath))
        {
            string[] lines = File.ReadAllLines(filePath); // String array storing all the lines in txt file
            
            Vector3[] vertices = new Vector3[_lineCount]; // Stores all points in Vector3
            _matrices = new Matrix4x4[vertices.Length];

            // Goes through each line in file
            foreach (string line in lines)
            {
                 string[] parts = line.Split(' '); // Stores all individual XYZ-point

                 if (parts.Length == 3) // Makes sure only lines with three points are added into points array
                 {
                     float x = float.Parse(parts[0], CultureInfo.InvariantCulture.NumberFormat); 
                     float y = float.Parse(parts[2], CultureInfo.InvariantCulture.NumberFormat); 
                     float z = float.Parse(parts[1], CultureInfo.InvariantCulture.NumberFormat);
                     
                     if (xMax < x) { xMax = x; }
                     if (xMin > x) { xMin = x; }
                     
                     if (yMax < y) { yMax = y; }
                     if (yMin > y) { yMin = y; }
                     
                     if (zMax < z) { zMax = z; }
                     if (zMin > z) { zMin = z; }

                     vertices[counter] = new Vector3(x, y, z); // Adds point into points array
                     counter++;
                 } 
            }
            
            
            for (int i = 0; i < vertices.Length; i++) {
                
                // Centers points around origo
                vertices[i].x -= 0.5f * (xMin + xMax);
                vertices[i].y -= 0.5f * (yMin + yMax);
                vertices[i].z -= 0.5f * (zMin + zMax);
                
                // Adds positions of vertices to martix
                _matrices[i] = Matrix4x4.TRS(vertices[i], Quaternion.identity, Vector3.one);
            }
        }
        else
        {
            Debug.LogError("File not found: " + filePath);
        }
    }

    private void Update()
    {
        // Renders the instances
        Graphics.DrawMeshInstanced(mesh, 0, material, _matrices);
    }
}
