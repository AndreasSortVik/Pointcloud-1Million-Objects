using System;
using UnityEngine;

public class BallSimulation : MonoBehaviour
{
    [SerializeField] private GameObject ball;
    [SerializeField] private Vector3 startPosition;
    
    private float _fallSpeed;
    private readonly float _gravity = 9.81f;

    private void Start()
    {
        ball.transform.position = startPosition;
    }

    private void FixedUpdate()
    {
        BallMovement();
    }

    private void BallMovement()
    {
        BallFalling(Time.fixedDeltaTime);
    }

    private void BallFalling(float deltaTime)
    {
        float y = _fallSpeed * deltaTime + 0.5f * -_gravity * deltaTime * deltaTime;
        _fallSpeed = _fallSpeed + -_gravity * deltaTime;
        transform.Translate(0, y, 0);
    }
}
