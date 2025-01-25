using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "AABB", menuName = "scripts/AABB", order = 1)]
public class AABB : ScriptableObject
{
    public GameObject obj;
    public BoxCollider collider;
    public Vector3 min;
    public Vector3 max;
    public float friction;

    public void Initialize()
    {
        collider = obj.GetComponent<BoxCollider>();
        Bounds bounds = collider.bounds;
        min = bounds.min;
        max = bounds.max;

    }

     
}
