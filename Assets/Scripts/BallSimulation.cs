using System;
using UnityEngine;

public class BallSimulation : MonoBehaviour
{
    public TriangleSurfaceGeneration triangleSurface;
    
    private Vector3 _barycentricPosition;
    
    private float _radius;
    
    // Speed
    [SerializeField] private Vector3 accelerationVector;
    [SerializeField] private Vector3 velocity;
    private Vector3 _oldVelocity;
    private float _fallSpeed;
    private readonly float _gravity = 9.81f;

    private int _triangleIndex = int.MaxValue;

    [SerializeField] private float dotProduct;

    private void Start()
    {
        //SetPositionAndScale(startPosition, scale);
        
        if (triangleSurface == null)
            Debug.LogError("Surface reference missing");
        
        // Finds the triangle the ball is in
        _triangleIndex = triangleSurface.FindFirstTriangleIndex(new Vector2(transform.position.x, transform.position.z));
        Debug.Log("Start triangle: " + _triangleIndex);
    }

    private void FixedUpdate()
    {
        if (_triangleIndex != int.MaxValue)
            BallMovement(Time.fixedDeltaTime);
        
        //Debug.Log("Current triangle: " + _triangleIndex);
    }

    public void SetPositionAndScale(Vector3 position, float s)
    {
        // Sets start position of the ball
        transform.position = position;
        
        // Sets scale of ball and calculates the radius of the ball
        transform.localScale = new Vector3(s, s, s);
        _radius = s / 2;
    }

    private void BallMovement(float deltaTime)
    {
        if (CheckCollision()) // Ball has collided with mesh
        {
            //Debug.Log("Check collision: true");
            
            // Gets normal vector from the triangle that the ball is currently inside
            //Vector3 normalVector = triangleSurface.Triangles[triangleSurface.currentTriangleIndex].NormalVector;
            Vector3 normalVector = triangleSurface.Triangles[_triangleIndex].NormalVector;

            Vector3 gravityForce = new Vector3(0, -_gravity, 0);
            Vector3 normalForce = -Vector3.Dot(gravityForce, normalVector) * normalVector;
            accelerationVector = gravityForce + normalForce;
            
            // Updates the velocity of the ball using equation 8.14
            velocity = _oldVelocity + accelerationVector * deltaTime;
            
            // Updates the position of the ball using equation 8.15
            Vector3 tempPos = transform.position + velocity * deltaTime;
            
            //Debug.Log("Height: " + triangleSurface.Triangles[_triangleIndex].Height);
            //transform.position = new Vector3(tempPos.x, triangleSurface._height + _radius, tempPos.z);
            transform.position = new Vector3(tempPos.x, triangleSurface.Triangles[_triangleIndex].Height + _radius,
                tempPos.z);
            
            _oldVelocity = velocity;
        }
        else // Ball is in free fall
        {
            //Debug.Log("Check collision: false");
            
            float y = _fallSpeed * deltaTime + 0.5f * -_gravity * deltaTime * deltaTime;
            _fallSpeed = _fallSpeed + -_gravity * deltaTime;
            transform.Translate(0, y, 0);
        }
    }

    // Checks if ball has collided with a triangle in the mesh
    private bool CheckCollision()
    {
        // Using (k = C + ((S - C) . n) * n when |(S - C) . n| <= r) to calculate the collision point
        Vector3 position = transform.position; // C
        _triangleIndex = triangleSurface.UpdateTriangleIndex(new Vector2(position.x, position.z), _triangleIndex);
        
        //Debug.Log("Triangle index: " + _triangleIndex);
        _barycentricPosition = triangleSurface.Triangles[_triangleIndex].BarycentricPosition; // S
        //Vector3 normalVector = triangleSurface.Triangles[triangleSurface.currentTriangleIndex].NormalVector; // n
        Vector3 normalVector = triangleSurface.Triangles[_triangleIndex].NormalVector;

        dotProduct = Vector3.Dot(_barycentricPosition - position, normalVector);
        
        if (MathF.Abs(dotProduct) <= _radius)
            return true;
        
        return false;
    }
}
