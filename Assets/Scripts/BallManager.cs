using UnityEngine;
using Random = UnityEngine.Random;

public class BallManager : MonoBehaviour
{
    [SerializeField] private TriangleSurfaceGeneration triangleSurface;
    
    [SerializeField] private GameObject ballPrefab;
    private BallSimulation[] _rain;
    
    [SerializeField] private Vector3 range;
    [SerializeField] private int rainAmount;
    [SerializeField] private float scale;

    private void Start()
    {
        // Initializes array
        _rain = new BallSimulation[rainAmount];

        // Positions gameobjects randomly
        for (int i = 0; i < _rain.Length; i++)
        {
            // Creates a random position to spawn the game object at
            float x = Random.Range(0, range.x);
            float y = Random.Range(0, range.y);
            float z = Random.Range(0, range.z);
            Vector3 position = new Vector3(x, y + 50, z);

            // Instantiates game object at position
            GameObject ball = Instantiate(ballPrefab, position, Quaternion.identity, transform);
            ball.GetComponent<BallSimulation>().triangleSurface = triangleSurface;

            BallSimulation tempBall = ball.GetComponent<BallSimulation>();

            if (tempBall != null)
            {
                _rain[i] = tempBall;
                _rain[i].SetPositionAndScale(position, scale);
            }
            else
            {
                Debug.LogError("BallSimulation component not found!");
            }
        }
    }
}
