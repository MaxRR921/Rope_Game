using System.Collections.Generic;
using UnityEngine;

public class Rope : MonoBehaviour
{
    [SerializeField]
    private List<Point> points = new List<Point>();
    [SerializeField]
    private List<Constraint> constraints = new List<Constraint>();

    [SerializeField]
    private float gravity = 10f;

    [SerializeField]
    private int numPoints = 10;

    [SerializeField]
    private float length;

    [SerializeField]
    private float frequency;

    [SerializeField]
    private Vector3 start_position;

    private List<Vector3> pointPositions = new List<Vector3>();

    [SerializeField]
    private LineRenderer lineRenderer;

    [SerializeField]
    private GameObject obj_pinned_to;

    [SerializeField]
    private float airFriction;


    [SerializeField]
    private List<BoxCollider> boxColliders = new List<BoxCollider>();

    [SerializeField]
    private List<SphereCollider> sphereColliders = new List<SphereCollider>();

    private List<AABB> aABBs = new List<AABB>();

    private List<Sphere> spheres = new List<Sphere>();

    void Start()
    {
        // Initialize LineRenderer
        lineRenderer = gameObject.GetComponent<LineRenderer>() ?? gameObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Unlit/Color"));
        lineRenderer.material.color = Color.red;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;

        foreach (BoxCollider collider in boxColliders)
        {
            aABBs.Add(new AABB(collider));
        }

        foreach(SphereCollider sphereCollider in sphereColliders)
        {
            spheres.Add(new Sphere(sphereCollider));
        }


        // Generate rope points and constraints
        InstantiateSections(numPoints);

        // Set LineRenderer positions
        UpdateLineRenderer();

        Debug.Log("HELLO!!");
    }

    void Update()
    {
        Simulate();
        UpdateLineRenderer();
    }

    [System.Serializable]
    class Point
    {
        private Vector3 m_position;
        private Vector3 m_previous_position;
        private bool m_fix;

        public Point(Vector3 position, Vector3 previous_position, bool fix)
        {
            m_position = position;
            m_previous_position = previous_position;
            m_fix = fix;
        }

        public Vector3 getPosition() => m_position;
        public Vector3 getPreviousPosition() => m_previous_position;
        public bool isFix() => m_fix;
        public void setPosition(Vector3 position) => m_position = position;
        public void setPreviousPosition(Vector3 position) => m_previous_position = position;
    }

    [System.Serializable]
    class Constraint
    {
        public Point m_pointA;
        public Point m_pointB;
        public float m_length;

        public Constraint(Point pointA, Point pointB)
        {
            m_pointA = pointA;
            m_pointB = pointB;
            m_length = Vector3.Distance(pointA.getPosition(), pointB.getPosition());
        }
    }

    private void Simulate()
    {
        // Apply Verlet integration
        foreach (Point p in points)
        {
            if (!p.isFix())
            {
                Vector3 velocity = (p.getPosition() - p.getPreviousPosition()) * airFriction;
                Vector3 positionBeforeUpdate = p.getPosition();
                Vector3 newPosition = positionBeforeUpdate + velocity + Vector3.down * gravity * Time.deltaTime * Time.deltaTime;

                foreach(Sphere sphere_data in spheres)
                {
                    if (IsCollidingWithSphere(newPosition, sphere_data.center, sphere_data.radius))
                    {
                        ResolveSphereCollision(ref newPosition, sphere_data.center, sphere_data.radius);
                    }
                }

                foreach(AABB aABB in aABBs)
                {
                    if(IsCollidingWithAABB(newPosition, aABB.min, aABB.max))
                    {
                        ResolveAABBCollision(ref newPosition, aABB.min, aABB.max);
                    }
                }

                p.setPosition(newPosition);
                p.setPreviousPosition(positionBeforeUpdate);
            }
            else
            {
                p.setPosition(obj_pinned_to.transform.position);
            }
        }

        // Satisfy constraints
        for (int i = 0; i < 5; i++)
        {
            foreach (var c in constraints)
            {
                Vector3 delta = c.m_pointB.getPosition() - c.m_pointA.getPosition();
                float distance = delta.magnitude;
                float difference = (distance - c.m_length) / distance;
                
                if (!c.m_pointA.isFix())
                {
                    c.m_pointA.setPosition(c.m_pointA.getPosition() + delta * 0.5f * difference);
                }

                if (!c.m_pointB.isFix())
                {
                    c.m_pointB.setPosition(c.m_pointB.getPosition() - delta * 0.5f * difference);
                }
            }
        }
    }

    private void InstantiateSections(int numPoints)
    {
        Vector3 distance_y = new Vector3(0, length / frequency, 0);
        Point last_point = null;

        for (int i = 0; i < numPoints; i++)
        {
            Vector3 currentPosition = start_position + (distance_y * i);
            Point newPoint = new Point(currentPosition, currentPosition, i == 0);
            points.Add(newPoint);

            if (i > 0)
            {
                constraints.Add(new Constraint(last_point, newPoint));
            }

            last_point = newPoint;
            pointPositions.Add(newPoint.getPosition());
        }
    }

    private void UpdateLineRenderer()
    {
        pointPositions.Clear();
        foreach (var p in points)
        {
            pointPositions.Add(p.getPosition());
        }
        lineRenderer.positionCount = pointPositions.Count; //linerenderer is faster if i just store them all in array like this, seems dumb but its good.
        lineRenderer.SetPositions(pointPositions.ToArray());
    }



    bool IsCollidingWithSphere(Vector3 pointPosition, Vector3 sphereCenter, float sphereRadius)
    {
        float distance = Vector3.Distance(pointPosition, sphereCenter);
        return distance < sphereRadius;
    }
    bool IsCollidingWithAABB(Vector3 pointPosition, Vector3 aabbMin, Vector3 aabbMax)
    {
        return pointPosition.x >= aabbMin.x && pointPosition.x <= aabbMax.x &&
               pointPosition.y >= aabbMin.y && pointPosition.y <= aabbMax.y &&
               pointPosition.z >= aabbMin.z && pointPosition.z <= aabbMax.z;
    }


    void ResolveSphereCollision(ref Vector3 pointPosition, Vector3 sphereCenter, float sphereRadius)
    {
        Vector3 direction = (pointPosition - sphereCenter).normalized;
        pointPosition = sphereCenter + direction * sphereRadius;
    }

    void ResolveAABBCollision(ref Vector3 pointPosition, Vector3 aabbMin, Vector3 aabbMax)
    {
        // Calculate distances to each face of the AABB
        float distToMinX = Mathf.Abs(pointPosition.x - aabbMin.x);
        float distToMaxX = Mathf.Abs(pointPosition.x - aabbMax.x);
        float distToMinY = Mathf.Abs(pointPosition.y - aabbMin.y);
        float distToMaxY = Mathf.Abs(pointPosition.y - aabbMax.y);
        float distToMinZ = Mathf.Abs(pointPosition.z - aabbMin.z);
        float distToMaxZ = Mathf.Abs(pointPosition.z - aabbMax.z);

        // Find the smallest distance
        float minDist = Mathf.Min(distToMinX, distToMaxX, distToMinY, distToMaxY, distToMinZ, distToMaxZ);

        // Push the point to the closest face
        if (minDist == distToMinX) pointPosition.x = aabbMin.x;
        else if (minDist == distToMaxX) pointPosition.x = aabbMax.x;
        else if (minDist == distToMinY) pointPosition.y = aabbMin.y;
        else if (minDist == distToMaxY) pointPosition.y = aabbMax.y;
        else if (minDist == distToMinZ) pointPosition.z = aabbMin.z;
        else if (minDist == distToMaxZ) pointPosition.z = aabbMax.z;
    }


    class Sphere
    {
        public Vector3 center;
        public float radius;

        public Sphere(SphereCollider c)
        {
            SphereCollider collider = c; 
            if (collider == null)
            {
                Debug.LogError($"GameObject {c.name} does not have a SphereCollider!");
            }

            // Get the world-space center of the sphere
            center = collider.transform.TransformPoint(collider.center);

            // Get the world-space radius (considering scale)
            radius = collider.radius * Mathf.Max(
                collider.transform.lossyScale.x,
                collider.transform.lossyScale.y,
                collider.transform.lossyScale.z
            );
        }
    }

    class AABB
    {
        public Vector3 min;
        public Vector3 max;

        public AABB(BoxCollider collider)
        {
            // Calculate bounds in world space
            Bounds bounds = collider.bounds;
            min = bounds.min;
            max = bounds.max;
        }
    }
}
