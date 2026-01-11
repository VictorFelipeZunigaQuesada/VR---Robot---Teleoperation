using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointStateSubscriber : MonoBehaviour
{
    [Header("Robot Reference")]
    public GameObject robotRoot;

    private ROSConnection ros;
    private Dictionary<string, ArticulationBody> articulationBodies;

    // Guardamos la posición del dedo izquierdo para calcular mimic
    private float leftFingerPosition = 0f;

    // Mapeo nombres ROS → Unity
    private Dictionary<string, string> jointNameMapping = new Dictionary<string, string>()
    {
        {"joint1", "link1"},
        {"joint2", "link2"},
        {"joint3", "link3"},
        {"joint4", "link4"},
        {"joint5", "link5"},
        {"joint6", "link6"},
        {"left_finger_joint", "left_finger"},
        {"right_finger_joint", "right_finger"}
    };

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);
        InitializeArticulationBodies();
    }

    void InitializeArticulationBodies()
    {
        articulationBodies = new Dictionary<string, ArticulationBody>();

        ArticulationBody[] bodies = robotRoot.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody body in bodies)
        {
            if (body.jointType != ArticulationJointType.FixedJoint)
            {
                articulationBodies[body.name] = body;
            }
        }
    }

    void UpdateJointStates(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string rosJointName = jointState.name[i];
            double position = jointState.position[i];

            // Convertir nombre ROS → Unity
            string unityJointName = rosJointName;
            if (jointNameMapping.ContainsKey(rosJointName))
                unityJointName = jointNameMapping[rosJointName];

            if (!articulationBodies.ContainsKey(unityJointName))
                continue;

            ArticulationBody body = articulationBodies[unityJointName];
            ArticulationDrive drive = body.xDrive;

            // ============================
            //     ★ GRIPPER LOGIC ★
            // ============================
            if (unityJointName == "left_finger")
            {
                // Guardamos la posición real del dedo izquierdo
                leftFingerPosition = (float)position;

                // Aplicar directamente
                drive.target = leftFingerPosition;
            }
            else if (unityJointName == "right_finger")
            {
                // ★ MIMIC EXACTO COMO EL URDF ★
                // multiplier = -1
                drive.target = -leftFingerPosition;
            }
            else
            {
                // ============================
                //  JOINTS ROTATIVOS DEL BRAZO
                // ============================
                drive.target = (float)(position * Mathf.Rad2Deg);
            }

            body.xDrive = drive;
        }
    }
}
