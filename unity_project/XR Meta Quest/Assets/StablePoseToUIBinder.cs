using UnityEngine;

public class StablePoseToUIBinder : MonoBehaviour
{
    [Header("References")]
    public StablePoseDetector detector;
    public PoseConfirmationUI ui;

    void OnEnable()
    {
        if (detector != null)
            detector.OnStablePoseDetected += HandleStablePose;
    }

    void OnDisable()
    {
        if (detector != null)
            detector.OnStablePoseDetected -= HandleStablePose;
    }

    void HandleStablePose(Pose pose)
    {
        Debug.Log("🟢 Stable pose event received -> calling UI");

        if (ui != null)
            ui.ShowConfirmation(pose);
        else
            Debug.LogError("❌ PoseConfirmationUI reference missing");
    }
}
