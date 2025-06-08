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

    // Removed the "has_been_cut" field so that each frame we can cut again.
    // private bool has_been_cut = false;

    [SerializeField]
    private int numPoints = 10;

    [SerializeField]
    private float length;

    [SerializeField]
    private GameObject start_object;

    private Vector3 start_position;

    private List<Vector3> pointPositions = new List<Vector3>();

    [SerializeField]
    private LineRenderer lineRenderer;

    [SerializeField]
    private float airFriction;

    [SerializeField]
    private List<Collider> collisionColliders = new List<Collider>();

    [SerializeField]
    private float ropeThickness = 0.1f;

    [SerializeField]
    private List<Rope> child_ropes = new List<Rope>();

    [SerializeField]
    private List<PinnableObject> pinnableObjects = new List<PinnableObject>();

    // When spawning a child, skip its InstantiateSections call
    private bool skipInstantiate = false;

    void Start()
    {


        
        // Initialize or grab existing LineRenderer
        lineRenderer = lineRenderer ?? gameObject.AddComponent<LineRenderer>();
        lineRenderer.generateLightingData = true;
        lineRenderer.shadowCastingMode = ShadowCastingMode.On;
        lineRenderer.receiveShadows = true;
        start_position = transform.position;

        // Create URP Lit material (Opaque)
        var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        mat.SetFloat("_Surface", 0);  // 0 = Opaque
        mat.color = Color.blue;
        lineRenderer.material = mat;




        // Thickness
        lineRenderer.startWidth = lineRenderer.endWidth = 0.1f;

        // Only instantiate sections if this is not a spawned child
        if (!skipInstantiate)
        {
            InstantiateSections(numPoints);
        }

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
        private int m_pid;
        private Vector3 m_position;
        private Vector3 m_previous_position;
        private bool m_fix;
        private float m_friction;
        private bool m_is_colliding_AABB;
        private int m_collide_count_AABB;
        private bool m_is_colliding;
        private int m_collide_count;
        private PinnableObject m_object_pinned_to;

        public Point(int pid, Vector3 position, Vector3 previous_position, bool fix)
        {
            m_pid = pid;
            m_position = position;
            m_previous_position = previous_position;
            m_fix = fix;
            m_friction = 0.0f;
            m_is_colliding_AABB = false;
            m_collide_count_AABB = 0;
            m_is_colliding = false;
            m_collide_count = 0;
            m_object_pinned_to = null;
        }
        public Vector3 getPosition() => m_position;
        public PinnableObject getObjectPinnedTo => m_object_pinned_to;
        public void setObjectPinnedTo(PinnableObject object_pinned_to) => m_object_pinned_to = object_pinned_to;
        public int getPid() => m_pid;
        public void setPid(int pid) => m_pid = pid;
        public Vector3 getPreviousPosition() => m_previous_position;
        public bool isFix() => m_fix;
        public void Fix(bool fix) => m_fix = fix;
        public void setPosition(Vector3 position) => m_position = position;
        public void setPreviousPosition(Vector3 position) => m_previous_position = position;
        public void setFriction(float friction) => m_friction = friction;
        public float getFriction() => m_friction;
        public bool isColliding() => m_is_colliding;
        public void setIsColliding(bool colliding) => m_is_colliding = colliding;
        public int getCollideCount() => m_collide_count;
        public void setCollideCount(int collide_count) => m_collide_count = collide_count;
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

    [System.Serializable]
    class PinnableObject
    {
        [SerializeField]
        private GameObject m_gameObject;
        [SerializeField]
        private int id;

        public PinnableObject(int pid, GameObject obj)
        {
            m_gameObject = obj;
            id = pid;
        }
        public int getId() => id;
        public GameObject getGameObject => m_gameObject;
    }

    private void Simulate()
    {
        // Use a local flag each frame so we can cut once per frame,
        // but allow future cuts on future frames.
        bool cutOccurred = false;

        // Verlet integration + collision + cutting
        foreach (Point p in points)
        {
            if (!p.isFix())
            {
                Vector3 velocity = (p.getPosition() - p.getPreviousPosition()) * airFriction * p.getFriction();
                Vector3 positionBeforeUpdate = p.getPosition();
                Vector3 newPosition = positionBeforeUpdate + velocity + Vector3.down * gravity * Time.deltaTime * Time.deltaTime;

                p.setFriction(1.0f);

                foreach (Collider col in collisionColliders)
                {

                    if (!col.enabled)
                        continue;   // ← this line prevents disabled cutters from ever testing
                    Vector3 closest = col.ClosestPoint(newPosition);
                    Vector3 delta = newPosition - closest;
                    float dist = delta.magnitude;
                    bool colliding = dist < ropeThickness;

                    if (colliding)
                    {
                        Debug.Log($"ROPE colliding {this.gameObject.name}");
                        if (!p.isColliding())
                        {
                            p.setIsColliding(true);
                            p.setCollideCount(1);
                        }
                        else
                        {
                            p.setCollideCount(p.getCollideCount() + 1);
                        }

                        float newFric = p.getFriction() > 0.4f
                            ? 0.99f - (0.001f * p.getCollideCount())
                            : 0.99f;
                        p.setFriction(newFric);

                        Vector3 normal = delta / dist;
                        newPosition = closest + normal * ropeThickness;
                    }
                    
                    // If this frame hasn't cut yet, allow a cut
                    
                    if (!cutOccurred && colliding && col.CompareTag("Cutter"))
                    {

                        col.enabled = false;
                        Debug.Log($"Cutting rope {this.gameObject.name}");
                        cutOccurred = true;

                        
                        int cutIndex = points.IndexOf(p);
                        DecoupleAt(cutIndex);

                        col.enabled = false;
                        Debug.Log($"Decoupled rope at index {cutIndex}");

                        // Return so we don't process positions/constraints for this frame
                        return;
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
                if (p.getObjectPinnedTo != null)
                {
                    p.setPosition(p.getObjectPinnedTo.getGameObject.transform.position);
                }
            }
        }

        // Constraint satisfaction
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

    private void DecoupleAt(int index)
    {
        if (index < 0 || index >= points.Count - 1) return;

        // Split points
        List<Point> lowerPoints = points.GetRange(index + 1, points.Count - (index + 1));
        points.RemoveRange(index + 1, points.Count - (index + 1));

        // Remove constraints that reference removed points
        constraints.RemoveAll(c => !points.Contains(c.m_pointA) || !points.Contains(c.m_pointB));

        // Update this rope's renderer so it ends at the cut
        UpdateLineRenderer();

        // Spawn a new child rope for the dangling segment
        SpawnChildSegment(lowerPoints);
    }

    private void SpawnChildSegment(List<Point> segmentPoints)
    {
        GameObject child = new GameObject("RopeSegment");
        child.transform.parent = transform.parent;
        child.transform.position = Vector3.zero;

        Rope r = child.AddComponent<Rope>();

        // Prevent the new Rope's Start() from re-instantiating points
        r.skipInstantiate = true;

        // Copy settings
        r.gravity = gravity;
        r.airFriction = airFriction;
        r.ropeThickness = ropeThickness;
        r.collisionColliders = new List<Collider>(collisionColliders);
        r.length = 0f;

        // Provide existing points and rebuild constraints
        r.points = segmentPoints;
        r.numPoints = segmentPoints.Count;
        segmentPoints[0].setPreviousPosition(segmentPoints[0].getPosition());

        r.constraints = new List<Constraint>();
        for (int i = 0; i < segmentPoints.Count - 1; i++)
        {
            r.constraints.Add(new Constraint(segmentPoints[i], segmentPoints[i + 1]));
        }

        // Hook up a new LineRenderer on the child
        LineRenderer lr = child.AddComponent<LineRenderer>();
        lr.material = lineRenderer.material;
        lr.startWidth = lr.endWidth = lineRenderer.startWidth;
        lr.shadowCastingMode = ShadowCastingMode.On;
        lr.receiveShadows = true;
        r.lineRenderer = lr;
    }

    //------------------------------------------------------------------

    private void InstantiateSections(int numPoints)
    {
        Vector3 distance_y = new Vector3(0, length / numPoints, 0);
        Point last_point = null;

        for (int i = 0; i < numPoints; i++)
        {
            Vector3 currentPosition = start_position + (distance_y * i);
            Point newPoint = new Point(i, currentPosition, currentPosition, false);

            foreach (PinnableObject p in pinnableObjects)
            {
                if (p.getId() == newPoint.getPid())
                {
                    newPoint.setObjectPinnedTo(p);
                    newPoint.Fix(true);
                }
            }

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
            Vector3 pos = p.getPosition();
            // Prevent NaN/Infinity from causing invalid AABB or IsFinite errors:
            if (!IsValidVector3(pos))
                pos = Vector3.zero;
            pointPositions.Add(pos);
        }

        lineRenderer.positionCount = pointPositions.Count;
        lineRenderer.SetPositions(pointPositions.ToArray());
    }

    private bool IsValidVector3(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
                 float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }
}
