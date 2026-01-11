using UnityEngine;

public class CameraOpticalFrameTF : MonoBehaviour
{
    void Start()
    {
        // TF ROS: camera_link -> camera_optical_frame
        transform.localPosition = Vector3.zero;
        transform.localRotation = Quaternion.Euler(180f, 0f, 0f);
    }
}
