    using UnityEngine;
    using Unity.Robotics.ROSTCPConnector;
    using RosMessageTypes.Geometry;

    public class DetectedObjectSubscriber : MonoBehaviour
    {
        [Header("Real object (ROS-driven)")]
        public Transform detectedObject;
        public MeshRenderer detectedRenderer;

        [Header("Ghost object (user-driven)")]
        public Transform targetObjectGhost;
        public MeshRenderer ghostRenderer;

        private bool receivedFirstPose = false;

        void Start()
        {
            if (detectedRenderer != null)
                detectedRenderer.enabled = false;

            if (ghostRenderer != null)
                ghostRenderer.enabled = false;

            ROSConnection.GetOrCreateInstance()
                .Subscribe<PoseStampedMsg>(
                    "/detected_object_pose",
                    PoseCallback
                );
        }

        void PoseCallback(PoseStampedMsg msg)
        {
            Vector3 unityPos = new Vector3(
                -(float)msg.pose.position.x,
            (float)msg.pose.position.y,
                (float)msg.pose.position.z
            );

            Quaternion unityRot = new Quaternion(
                (float)msg.pose.orientation.x,
            -(float)msg.pose.orientation.y,
                -(float)msg.pose.orientation.z,
            (float)msg.pose.orientation.w
            );

            // 🔹 Objeto real (siempre sigue ROS)
            detectedObject.localPosition = unityPos;
            detectedObject.localRotation = unityRot;

            if (!receivedFirstPose)
            {
                receivedFirstPose = true;

                // 🔹 Mostrar objeto real
                if (detectedRenderer != null)
                    detectedRenderer.enabled = true;

                // 🔹 Inicializar el ghost EXACTAMENTE en la misma pose
                if (targetObjectGhost != null)
                {
                    targetObjectGhost.localPosition = unityPos;
                    targetObjectGhost.localRotation = unityRot;
                }

                if (ghostRenderer != null)
                    ghostRenderer.enabled = true;
            }
        }
    }
