using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;

public class PoseConfirmationUI : MonoBehaviour
{
    [Header("UI Root (parent UI object)")]
    public GameObject uiRoot;   // GameObject "PosePanel" (el que tiene CanvasGroup)

    [Header("References")]
    public StablePoseDetector detector; // Referencia al script del objeto físico

    [Header("Debug")]
    public bool logDebug = true;

    [Header("Reference Frame")]
    public Transform robotBase; // Arrastra aquí el objeto UF_ROBOT

    private CanvasGroup canvasGroup;

    // Pose candidata
    private Pose pendingPose;
    private bool hasPendingPose = false;

    // Evento futuro (ROS / MoveIt)
    public UnityEvent<Pose> OnPoseConfirmed;

    void Start()
    {
        if (uiRoot == null)
        {
            Debug.LogError("❌ UI Root no asignado en PoseConfirmationUI");
            return;
        }

        canvasGroup = uiRoot.GetComponent<CanvasGroup>();

        if (canvasGroup == null)
        {
            Debug.LogError("❌ UI Root no tiene CanvasGroup");
            return;
        }

        // Si no asignaste el detector por Inspector, intentamos buscarlo
        if (detector == null)
        {
            detector = FindObjectOfType<StablePoseDetector>();
        }

        HidePanel();
    }

    /// <summary>
    /// Llamado cuando se detecta una pose estable
    /// </summary>
    public void ShowConfirmation(Pose stablePose)
    {
        pendingPose = stablePose;
        hasPendingPose = true;

        // Mostrar panel
        canvasGroup.alpha = 1f;
        canvasGroup.interactable = true;
        canvasGroup.blocksRaycasts = true;

        if (logDebug)
            Debug.Log("🟢 Pose estable recibida → mostrando panel");
    }

    /// <summary>
    /// Botón SÍ
    /// </summary>
    public void OnConfirmYes()
    {
        if (!hasPendingPose || robotBase == null) return;

        // 1. Aplicamos tu corrección de 180° en Z (la que hizo que el cubo calzara)
        Quaternion correction = Quaternion.Euler(0, 0, 180);
        Quaternion worldRotation = pendingPose.rotation * correction;

        // 2. CONVERSIÓN A ESPACIO LOCAL DEL ROBOT
        // InverseTransformPoint convierte la posición del mundo a la "realidad" del robot
        Vector3 relativePosition = robotBase.InverseTransformPoint(pendingPose.position);
        
        // Calculamos la rotación relativa restando la rotación del robot
        Quaternion relativeRotation = Quaternion.Inverse(robotBase.rotation) * worldRotation;

        Pose relativePose = new Pose(relativePosition, relativeRotation);

        if (logDebug)
        {
            Vector3 localEuler = relativePose.rotation.eulerAngles;
            Debug.Log("<color=cyan>========================================</color>");
            Debug.Log("<color=cyan><b>[DATOS RELATIVOS AL ROBOT (UF_ROBOT)]</b></color>");
            Debug.Log($"<b>Posición Local:</b> {relativePosition.x:F4}, {relativePosition.y:F4}, {relativePosition.z:F4}");
            Debug.Log($"<b>Rotación Euler Local:</b> {localEuler.x:F2}, {localEuler.y:F2}, {localEuler.z:F2}");
            Debug.Log("<color=white>Nota: Estos datos ya consideran que el robot está rotado 180° en Y</color>");
            Debug.Log("<color=cyan>========================================</color>");
        }

        OnPoseConfirmed?.Invoke(relativePose);
        if (detector != null) detector.ResetDetection();
        HidePanel();
    }

    /// <summary>
    /// Botón NO
    /// </summary>
    public void OnConfirmNo()
    {
        if (logDebug)
            Debug.Log("❌ Pose RECHAZADA por el usuario");

        // Reiniciar el detector para que pueda detectar la siguiente estabilidad
        if (detector != null) detector.ResetDetection();
        
        HidePanel();
    }

    private void HidePanel()
    {
        hasPendingPose = false;

        canvasGroup.alpha = 0f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;

        // Limpia el estado de los botones para que no se vean pálidos/seleccionados
        if (EventSystem.current != null)
        {
            EventSystem.current.SetSelectedGameObject(null);
        }

        if (logDebug)
            Debug.Log("CleanUp: Panel ocultado y sistema de UI reseteado.");
    }
}