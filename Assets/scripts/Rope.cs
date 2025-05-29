using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

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
    private Vector3 start_position;

    private List<Vector3> pointPositions = new List<Vector3>();

    [SerializeField]
    private LineRenderer lineRenderer;

    [SerializeField]
    private GameObject obj_pinned_to;

    [SerializeField]
    private float airFriction;

    [SerializeField]
    private List<Collider> collisionColliders = new List<Collider>();

    [SerializeField]
    private float ropeThickness = 0.1f;

    void Start()
    {
        // Initialize LineRenderer
        // grab or add the LineRenderer
        lineRenderer = lineRenderer ?? gameObject.AddComponent<LineRenderer>();
        lineRenderer.alignment = LineAlignment.TransformZ;
        lineRenderer.generateLightingData = true;    // gives it normals/tangents for proper lighting
        lineRenderer.shadowCastingMode = ShadowCastingMode.On;
        lineRenderer.receiveShadows = true;

        // 1) Create a URP Lit material and set it Opaque
        var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        mat.SetFloat("_Surface", 0);          // 0 = Opaque, 1 = Transparent
        mat.color = Color.blue;               // your rope color

        // 2) Assign it
        lineRenderer.material = mat;

        // 3) Make sure it has real thickness
        lineRenderer.startWidth = lineRenderer.endWidth = 0.1f;

        // 4) Enable shadows
        lineRenderer.shadowCastingMode = ShadowCastingMode.On;
        lineRenderer.receiveShadows = true;
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
        private float m_friction;
        private bool m_is_colliding_AABB;
        private int m_collide_count_AABB; //Counts number of frames the rope has been colliding with something for.
        private bool m_is_colliding;
        private int m_collide_count; //Counts number of frames the rope has been colliding with something for.

        public Point(Vector3 position, Vector3 previous_position, bool fix)
        {
            m_position = position;
            m_previous_position = previous_position;
            m_fix = fix;
            m_friction = 0.0f;
            m_is_colliding_AABB = false;
            m_collide_count_AABB = 0;
            m_is_colliding = false;
            m_collide_count = 0;
        }
        public Vector3 getPosition() => m_position;
        public Vector3 getPreviousPosition() => m_previous_position;
        public bool isFix() => m_fix;
        public void setPosition(Vector3 position) => m_position = position;
        public void setPreviousPosition(Vector3 position) => m_previous_position = position;
        public void setFriction(float friction) => m_friction = friction;
        public float getFriction() => m_friction;
        public bool isColliding () => m_is_colliding;
        public void setIsColliding(bool colliding) => m_is_colliding = colliding; 
        public int getCollideCount() => m_collide_count;
        public void setCollideCount(int collide_count)=> m_collide_count = collide_count;

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
                Vector3 velocity = (p.getPosition() - p.getPreviousPosition()) * airFriction * p.getFriction();
                Vector3 positionBeforeUpdate = p.getPosition();
                Vector3 newPosition = positionBeforeUpdate + velocity + Vector3.down * gravity * Time.deltaTime * Time.deltaTime;


                p.setFriction(1.0f);


                // for every collider in your inspector list:
                foreach (var col in collisionColliders)
                {
                    // 1) find closest point on that collider’s surface
                    Vector3 closest = col.ClosestPoint(newPosition);
                    Vector3 delta = newPosition - closest;
                    float dist = delta.magnitude;

                    // 2) are we overlapping? (tube of radius ropeThickness)
                    bool colliding = dist < ropeThickness;

                    if (colliding)
                    {
                        // first‐frame vs continuing collision
                        if (!p.isColliding())
                        {
                            p.setIsColliding(true);
                            p.setCollideCount(1);
                        }
                        else
                        {
                            p.setCollideCount(p.getCollideCount() + 1);
                        }

                        // exactly the same friction formula you used
                        float newFric = p.getFriction() > 0.4f
                            ? 0.99f - (0.001f * p.getCollideCount())
                            : 0.99f;
                        p.setFriction(newFric);

                        // 3) push the point out to the surface + thickness
                        Vector3 normal = delta / dist;
                        newPosition = closest + normal * ropeThickness;
                    }
                    else
                    {
                        p.setIsColliding(false);
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
        Vector3 distance_y = new Vector3(0, length / numPoints, 0);
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

}
