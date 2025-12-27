using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SimplePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "simple_topic";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    // Method to publish data when button is clicked
    public void Publish()
    {
        StringMsg message = new StringMsg("Hello from Unity!");
        ros.Publish(topicName, message);
    }
}

