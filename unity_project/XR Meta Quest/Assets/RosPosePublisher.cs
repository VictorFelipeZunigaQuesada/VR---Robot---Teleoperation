using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosPosePublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "target_pose";
    
    private Pose lastConfirmedPose;
    private bool shouldPublishContinuous = false;
    private int messageCount = 0; // Contador para ver el flujo en consola

    void Start()
    {
        // Inicializa la conexión con el endpoint de ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    // Esta función se activa cuando presionas el botón "ACEPTAR"
    public void PublishToRos(Pose relativePose)
    {
        lastConfirmedPose = relativePose;
        shouldPublishContinuous = true;
        Debug.Log("<color=green>🚀 BOTÓN PRESIONADO: Iniciando flujo continuo a ROS.</color>");
    }

    void Update()
    {
        // Una vez activado, envía la pose en cada frame
        if (shouldPublishContinuous)
        {
            SendPoseMessage(lastConfirmedPose);
        }
    }

    private void SendPoseMessage(Pose pose)
    {
        // 1. SANEAMIENTO (Mantenemos tu lógica original de 90 grados)
        Vector3 euler = pose.rotation.eulerAngles;
        float cleanX = Mathf.Round(euler.x / 90f) * 90f;
        float cleanZ = Mathf.Round(euler.z / 90f) * 90f;
        float cleanY = euler.y; // Rotación libre en el eje vertical de Unity

        Quaternion cleanRotation = Quaternion.Euler(cleanX, cleanY, cleanZ);

        // 2. CONSTRUCCIÓN DEL MENSAJE (Ejes puros de Unity)
        PoseMsg msg = new PoseMsg
        {
            position = new PointMsg(pose.position.x, pose.position.y, pose.position.z),
            orientation = new QuaternionMsg {
                x = cleanRotation.x,
                y = cleanRotation.y,
                z = cleanRotation.z,
                w = cleanRotation.w
            }
        };

        // 3. PUBLICACIÓN A ROS
        ros.Publish(topicName, msg);

        // 4. LOG CONTINUO EN CONSOLA
        messageCount++;
        Debug.Log($"[MSG #{messageCount}] Enviando a ROS -> Pos: {msg.position.x:F2}, {msg.position.y:F2}, {msg.position.z:F2}");
    }
}