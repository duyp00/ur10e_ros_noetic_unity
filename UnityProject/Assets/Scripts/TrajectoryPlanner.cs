using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    private string m_TopicName = "/collision_object";
    [SerializeField]
    string m_RosServiceName = "ur10e_rg2_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR10e;
    public GameObject UR10e { get => m_UR10e; set => m_UR10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }
    // [SerializeField]
    // GameObject m_Table;
    // [SerializeField]
    // GameObject m_3DPrinter ;

    [SerializeField]
    GameObject m_Table;
    public GameObject Table { get => m_Table; set => m_Table = value; }
    [SerializeField]
    GameObject m_Printer;
    public GameObject Printer { get => m_Printer; set => m_Printer = value; }
    [SerializeField]
    GameObject m_MovingObstacle;
    public GameObject MovingObstacle { get => m_MovingObstacle; set => m_MovingObstacle = value; }
    
    [SerializeField]
    float m_ObstaclePublishRate = 10f; // How often to publish obstacle position (Hz)
    private float m_LastObstaclePublishTime = 0f;


    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(180, 180, 0);
    // TODO: Adjust for better position offset
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.27f;

    // Articulation Bodies for the Robot arm
    ArticulationBody[] m_JointArticulationBodies;
    // Articulation Bodies for the Gripper
    ArticulationBody m_LeftInnerKnuckle;
    ArticulationBody m_RightInnerKnuckle;
    ArticulationBody m_LeftOuterKnuckle;
    ArticulationBody m_RightOuterKnuckle;
    ArticulationBody m_leftInnerFinger;
    ArticulationBody m_rightInnerFinger;

    // Gripper attach point and attached object tracking
    private Transform m_GripperAttachPoint;
    private Rigidbody m_AttachedObject;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        // Register the collision object publisher
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR10e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Identify gripper joints
        string gripperBasePath = "base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/onrobot_rg2_base_link";
        m_LeftInnerKnuckle = m_UR10e.transform.Find(gripperBasePath + "/left_inner_knuckle").GetComponent<ArticulationBody>();
        m_RightInnerKnuckle = m_UR10e.transform.Find(gripperBasePath + "/right_inner_knuckle").GetComponent<ArticulationBody>();
        m_LeftOuterKnuckle = m_UR10e.transform.Find(gripperBasePath + "/left_outer_knuckle").GetComponent<ArticulationBody>();
        m_RightOuterKnuckle = m_UR10e.transform.Find(gripperBasePath + "/right_outer_knuckle").GetComponent<ArticulationBody>();
        m_leftInnerFinger = m_UR10e.transform.Find(gripperBasePath + "/left_outer_knuckle/left_inner_finger").GetComponent<ArticulationBody>();
        m_rightInnerFinger = m_UR10e.transform.Find(gripperBasePath + "/right_outer_knuckle/right_inner_finger").GetComponent<ArticulationBody>();

        if (!m_LeftInnerKnuckle || !m_RightInnerKnuckle || !m_LeftOuterKnuckle || !m_RightOuterKnuckle || !m_leftInnerFinger || !m_rightInnerFinger)
        {
            Debug.LogError("Some gripper articulation bodies are missing. Please check the hierarchy.");
        }

        // Set gripper attach point (use the base of the gripper as attach point)
        m_GripperAttachPoint = m_UR10e.transform.Find(gripperBasePath);
        if (m_GripperAttachPoint == null)
        {
            Debug.LogError("Gripper attach point not found!");
        }
    }

    /// <summary>
    ///     Update is called once per frame - used to publish moving obstacle position
    /// </summary>
    void Update()
    {
        // Continuously publish the moving obstacle position at the specified rate
        if (m_MovingObstacle != null && Time.time - m_LastObstaclePublishTime >= 1f / m_ObstaclePublishRate)
        {
            PublishMovingObstacle();
            m_LastObstaclePublishTime = Time.time;
        }
    }

    /// <summary>
    ///     Close the gripper and attach object
    /// </summary>
    void CloseGripper()
    {
        float closeValue = 35f;
        SetGripperPosition(closeValue);
        
        // Wait a bit for gripper to close, then attach object
        StartCoroutine(AttachObjectAfterDelay(0.5f));
    }

    IEnumerator AttachObjectAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        AttachObjectToGripper(m_Target);
    }

    /// <summary>
    ///     Detach object and open the gripper
    /// </summary>
    void OpenGripper()
    {
        DetachObjectFromGripper();
        
        float openValue = 0f;
        SetGripperPosition(openValue);
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>Ur10eMoveitJoints</returns>
    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        // Publish the table mesh at the start
        PublishTableMesh();
        // Publish the printer mesh at the start
        PublishPrinterMesh();

        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(180, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from ur10e_rg2_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }

    void PublishTableMesh()
    {
        // Get the MeshFilter component from the Table GameObject
        MeshFilter meshFilter = m_Table.GetComponent<MeshFilter>();
        if (meshFilter == null)
        {
            Debug.LogError("MeshFilter component not found on Table GameObject.");
            return;
        }

        // Retrieve the mesh, vertices, and triangles data
        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // Validate mesh data
        if (vertices == null || vertices.Length == 0 || triangles == null || triangles.Length == 0)
        {
            Debug.LogError("Mesh data is invalid or empty.");
            return;
        }

        // Prepare vertices and triangles for ROS messages
        List<PointMsg> points = new List<PointMsg>();
        foreach (Vector3 vertex in vertices)
        {
            // Convert vertex to FLU orientation and add to points list
            var fluVertex = m_Table.transform.TransformPoint(vertex).To<FLU>();
            points.Add(new PointMsg(fluVertex.x, fluVertex.y, fluVertex.z));
        }

        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            triangleMsgs.Add(new MeshTriangleMsg
            {
                vertex_indices = new uint[]
                {
                    (uint)triangles[i],
                    (uint)triangles[i + 2], // Swap these two indices
                    (uint)triangles[i + 1]
                }
            });
        }


        // Create MeshMsg
        MeshMsg meshMsg = new MeshMsg
        {
            vertices = points.ToArray(),
            triangles = triangleMsgs.ToArray()
        };

        // Get the table's position and rotation in FLU
        PoseMsg tablePose = new PoseMsg
        {
            // Set the position of the object to the origin (0, 0, 0) in the base_link frame
            position = new PointMsg(0.0, 0.0, 0.0),

            // Set the orientation of the object to no rotation (identity quaternion) in the base_link frame
            orientation = new QuaternionMsg(0.0, 0.0, 0.0, 1.0)

        };

        // Create CollisionObjectMsg
        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_link"
            },
            id = "table",
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new PoseMsg[] { tablePose },
            meshes = new MeshMsg[] { meshMsg }
        };

        Debug.Log($"Unity Position: {m_Table.transform.position}");
        Debug.Log($"Unity Roatation: {m_Table.transform.rotation}");
        Debug.Log($"Position sent to ROS: {tablePose.position.x},{tablePose.position.y},{tablePose.position.z}");
        Debug.Log($"Rotation sent to ROS: {tablePose.orientation.x},{tablePose.orientation.y},{tablePose.orientation.z},{tablePose.orientation.w}");

        // Publish the CollisionObjectMsg
        m_Ros.Publish("/collision_object", collisionObject);
    }

    void PublishPrinterMesh()
    {
        // Get the MeshFilter component from the 3D Printer GameObject
        MeshFilter meshFilter = m_Printer.GetComponent<MeshFilter>();
        if (meshFilter == null)
        {
            Debug.LogError("MeshFilter component not found on 3D Printer GameObject.");
            return;
        }

        // Retrieve the mesh, vertices, and triangles data
        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // Validate mesh data
        if (vertices == null || vertices.Length == 0 || triangles == null || triangles.Length == 0)
        {
            Debug.LogError("Mesh data is invalid or empty.");
            return;
        }
        // Prepare vertices and triangles for ROS messages
        List<PointMsg> points = new List<PointMsg>();
        foreach (Vector3 vertex in vertices)
        {
            // Convert vertex to FLU orientation and add to points list
            var fluVertex = m_Printer.transform.TransformPoint(vertex).To<FLU>();
            points.Add(new PointMsg(fluVertex.x, fluVertex.y, fluVertex.z));
        }

        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            triangleMsgs.Add(new MeshTriangleMsg
            {
                vertex_indices = new uint[]
                {
                    (uint)triangles[i],
                    (uint)triangles[i + 2], // Swap these two indices
                    (uint)triangles[i + 1]
                }
            });
        }

        // Create MeshMsg
        MeshMsg meshMsg = new MeshMsg
        {
            vertices = points.ToArray(),
            triangles = triangleMsgs.ToArray()
        };
        
        PoseMsg printerPose = new PoseMsg
        {
            // Set the position of the object to the origin (0, 0, 0) in the base_link frame
            position = new PointMsg(0.0, 0.0, 0.0),

            // Set the orientation of the object to no rotation (identity quaternion) in the base_link frame
            orientation = new QuaternionMsg(0.0, 0.0, 0.0, 1.0)
        };

        // Create CollisionObjectMsg
        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_link"
            },
            id = "3d_printer",
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new PoseMsg[] { printerPose },
            meshes = new MeshMsg[] { meshMsg }
        };

        Debug.Log($"Unity Position: {m_Printer.transform.position}");
        Debug.Log($"Unity Roatation: {m_Printer.transform.rotation}");
        Debug.Log($"Position sent to ROS: {printerPose.position.x},{printerPose.position.y},{printerPose.position.z}");
        Debug.Log($"Rotation sent to ROS: {printerPose.orientation.x},{printerPose.orientation.y},{printerPose.orientation.z},{printerPose.orientation.w}");

        // Publish the CollisionObjectMsg
        m_Ros.Publish("/collision_object", collisionObject);
    }

    /// <summary>
    ///     Publish the moving obstacle's current position to ROS as a collision object.
    ///     This method is called continuously to update the obstacle position in the planning scene.
    /// </summary>
    void PublishMovingObstacle()
    {
        if (m_MovingObstacle == null)
        {
            return;
        }

        // Get the obstacle's current position and scale
        Vector3 position = m_MovingObstacle.transform.position;
        Vector3 scale = m_MovingObstacle.transform.lossyScale;
        Quaternion rotation = m_MovingObstacle.transform.rotation;

        // Convert position to ROS FLU coordinate system
        var fluPosition = position.To<FLU>();
        var fluRotation = rotation.To<FLU>();

        // Create a box primitive for the obstacle (using the object's scale as dimensions)
        var solidPrimitive = new SolidPrimitiveMsg
        {
            type = SolidPrimitiveMsg.BOX,
            dimensions = new double[] { scale.x, scale.z, scale.y } // FLU: x, y, z -> Unity: x, z, y
        };

        // Create pose for the obstacle
        PoseMsg obstaclePose = new PoseMsg
        {
            position = new PointMsg(fluPosition.x, fluPosition.y, fluPosition.z),
            orientation = new QuaternionMsg(fluRotation.x, fluRotation.y, fluRotation.z, fluRotation.w)
        };

        // Create CollisionObjectMsg with MOVE operation to update existing object
        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "base_link"
            },
            id = "moving_obstacle",
            operation = CollisionObjectMsg.ADD, // ADD will update if exists
            primitives = new SolidPrimitiveMsg[] { solidPrimitive },
            primitive_poses = new PoseMsg[] { obstaclePose }
        };

        // Publish the CollisionObjectMsg
        m_Ros.Publish("/collision_object", collisionObject);
    }

    void SetGripperPosition(float position)
    {
        // Increase grip force by adjusting forceLimit, stiffness, and damping
        float gripForceLimit = 300f;
        float gripStiffness = 6000f;
        float gripDamping = 200f;

        ArticulationDrive drive = m_LeftInnerKnuckle.xDrive;
        drive.target = -position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_LeftInnerKnuckle.xDrive = drive;

        drive = m_RightInnerKnuckle.xDrive;
        drive.target = -position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_RightInnerKnuckle.xDrive = drive;

        drive = m_LeftOuterKnuckle.xDrive;
        drive.target = position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_LeftOuterKnuckle.xDrive = drive;

        drive = m_RightOuterKnuckle.xDrive;
        drive.target = -position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_RightOuterKnuckle.xDrive = drive;

        drive = m_leftInnerFinger.xDrive;
        drive.target = position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_leftInnerFinger.xDrive = drive;

        drive = m_rightInnerFinger.xDrive;
        drive.target = position;
        drive.forceLimit = gripForceLimit;
        drive.stiffness = gripStiffness;
        drive.damping = gripDamping;
        m_rightInnerFinger.xDrive = drive;
    }

    /// <summary>
    ///     Attach object to gripper using parent-child relationship (standard simulation approach)
    /// </summary>
    void AttachObjectToGripper(GameObject obj)
    {
        if (obj == null || m_GripperAttachPoint == null)
        {
            Debug.LogWarning("Cannot attach object: object or gripper attach point is null");
            return;
        }

        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.useGravity = false;
            rb.isKinematic = true;
            obj.transform.SetParent(m_GripperAttachPoint, true);
            m_AttachedObject = rb;
            Debug.Log($"Object '{obj.name}' attached to gripper");
        }
        else
        {
            Debug.LogWarning($"Object '{obj.name}' does not have a Rigidbody component");
        }
    }

    /// <summary>
    ///     Detach object from gripper and restore physics
    /// </summary>
    void DetachObjectFromGripper()
    {
        if (m_AttachedObject != null)
        {
            m_AttachedObject.transform.SetParent(null, true);
            m_AttachedObject.isKinematic = false;
            m_AttachedObject.useGravity = true;
            Debug.Log($"Object '{m_AttachedObject.name}' detached from gripper");
            m_AttachedObject = null;
        }
    }

}
