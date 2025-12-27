using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PhysicsPingPongMover : MonoBehaviour
{
    [Header("Path")]
    [Tooltip("Local direction of movement. Use (0,0,1) for forward (Z), (1,0,0) for right (X).")]
    public Vector3 localDirection = Vector3.forward;

    [Tooltip("Distance (world units) to travel from the starting point along the localDirection.")]
    public float distance = 5f;

    [Header("Motion")]
    [Tooltip("Speed multiplier (units per second). Lower = slower.")]
    public float speed = 0.5f;

    [Tooltip("If true, movement ignores gravity. If false, gravity still affects the Rigidbody.")]
    public bool ignoreGravity = true;

    [Tooltip("If true, object will rotate to face movement direction.")]
    public bool faceDirection = false;

    private Rigidbody _rb;
    private Vector3 _startPos;
    private Vector3 _endPos;
    private float _timeAccumulator = 0f;

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }

    private void Start()
    {
        // Save starting position
        _startPos = transform.position;

        // Compute world-space direction from localDirection
        Vector3 worldDir = transform.TransformDirection(localDirection.normalized);

        // Compute end position
        _endPos = _startPos + worldDir * Mathf.Abs(distance);

        // Optional: configure Rigidbody according to settings
        _rb.useGravity = !ignoreGravity;
        // Freeze rotations so physics won't tip the object while moving
        _rb.constraints = RigidbodyConstraints.FreezeRotation;
        // Interpolation for smooth visual motion
        _rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    private void FixedUpdate()
    {
        // accumulate time scaled by speed; pingpong expects a value that goes 0->1->0
        _timeAccumulator += Time.fixedDeltaTime * speed;

        float t = Mathf.PingPong(_timeAccumulator, 1f); // 0..1..0
        Vector3 targetPos = Vector3.Lerp(_startPos, _endPos, t);

        // Move the Rigidbody smoothly using MovePosition (physics-friendly)
        _rb.MovePosition(targetPos);

        if (faceDirection)
        {
            // Face the movement direction (only when there's displacement)
            Vector3 moveDir = (_endPos - _startPos).normalized;
            if (moveDir.sqrMagnitude > 0.0001f)
            {
                Quaternion targetRot = Quaternion.LookRotation(moveDir, Vector3.up);
                _rb.MoveRotation(Quaternion.Slerp(_rb.rotation, targetRot, 0.2f));
            }
        }
    }

    // Expose a helper to set endpoints in code if you want to at runtime
    public void SetPath(Vector3 worldStart, Vector3 worldEnd)
    {
        _startPos = worldStart;
        _endPos = worldEnd;
        // reset accumulator so motion starts from startPos
        _timeAccumulator = 0f;
    }
}
