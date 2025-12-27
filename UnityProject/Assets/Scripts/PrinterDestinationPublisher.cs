using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PrinterDestinationPublisher : MonoBehaviour
{
    [SerializeField]
    private string m_TopicName = "/collision_object";

    [SerializeField]
    private GameObject m_Printer;

    // ROS Connector
    private ROSConnection m_Ros;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);
    }

    public void Publish()
    {
        // Get the mesh information
        MeshFilter meshFilter = m_Printer.GetComponent<MeshFilter>();

        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // Create shape_msgs/Mesh from the Unity mesh
        List<MeshMsg> meshMsgs = new List<MeshMsg>();
        MeshMsg meshMsg = new MeshMsg();

        List<PointMsg> points = new List<PointMsg>();
        foreach (var vertex in vertices)
        {
            Vector3 worldVertex = m_Printer.transform.TransformPoint(vertex);
            points.Add(new PointMsg(worldVertex.x, worldVertex.y, worldVertex.z));
        }

        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            MeshTriangleMsg triangleMsg = new MeshTriangleMsg
            {
                vertex_indices = new uint[] { (uint)triangles[i], (uint)triangles[i + 1], (uint)triangles[i + 2] }
            };
            triangleMsgs.Add(triangleMsg);
        }

        meshMsg.vertices = points.ToArray();
        meshMsg.triangles = triangleMsgs.ToArray();
        meshMsgs.Add(meshMsg);

        // Convert the orientation from Unity to ROS and account for the 90-degree difference
        Vector3 unityPosition = m_Printer.transform.position;
        Quaternion unityRotation = m_Printer.transform.rotation;

        // Adjust the rotation by 90 degrees around the x-axis
        Quaternion adjustment = Quaternion.Euler(90, 0, 0);
        Quaternion rosQuaternion = unityRotation * adjustment;

        var printerPose = new PoseMsg
        {
            position = unityPosition.To<FLU>(),
            orientation = new QuaternionMsg(
                rosQuaternion.x,
                rosQuaternion.y,
                rosQuaternion.z,
                rosQuaternion.w
            )
        };

        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "world"
            },
            id = "printer",
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new List<PoseMsg> { printerPose }.ToArray(),
            meshes = meshMsgs.ToArray()
        };

        // Publish the collision object
        m_Ros.Publish(m_TopicName, collisionObject);
    }
}
