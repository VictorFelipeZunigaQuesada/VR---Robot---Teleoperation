using UnityEngine;
using System;

[RequireComponent(typeof(Rigidbody))]
public class StablePoseDetector : MonoBehaviour
{
    public Rigidbody rb;

    public event Action<Pose> OnStablePoseDetected;

    const float LIN_VEL_EPS = 0.01f;   // m/s
    const float ANG_VEL_EPS = 0.5f;    // deg/s
    const float STABLE_TIME = 0.5f;    // segundos continuos

    float stableTimer = 0f;
    bool poseCaptured = false;

    // 🔑 control de interacción
    bool userHasInteracted = false;
    bool wasKinematic = false;

    void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        // Estado inicial
        wasKinematic = rb.isKinematic;
    }

    void FixedUpdate()
    {
        // 🔎 Detectar transición: agarrado → soltado
        if (wasKinematic && !rb.isKinematic)
        {
            userHasInteracted = true;
            poseCaptured = false;
            stableTimer = 0f;

            Debug.Log("🟡 Interacción válida detectada (grab → release)");
        }

        wasKinematic = rb.isKinematic;

        // ❌ No evaluar si el usuario nunca interactuó
        if (!userHasInteracted)
            return;

        // ❌ No evaluar mientras esté agarrado
        if (rb.isKinematic)
        {
            stableTimer = 0f;
            return;
        }

        // ❌ Evitar múltiples capturas
        if (poseCaptured)
            return;

        // Evaluación de estabilidad
        if (IsNearlyStill())
        {
            stableTimer += Time.fixedDeltaTime;

            if (stableTimer >= STABLE_TIME)
            {
                CaptureStablePose();
            }
        }
        else
        {
            stableTimer = 0f;
        }
    }

    bool IsNearlyStill()
    {
        return rb.velocity.magnitude < LIN_VEL_EPS &&
               rb.angularVelocity.magnitude * Mathf.Rad2Deg < ANG_VEL_EPS;
    }

    void CaptureStablePose()
    {
        poseCaptured = true;
        rb.Sleep();

        // Log simplificado para no estorbar
        Debug.Log("<color=yellow>--- DETECTOR: Pose estable detectada ---</color>");

        Pose stablePose = new Pose(transform.position, transform.rotation);
        OnStablePoseDetected?.Invoke(stablePose);
    }


    // 🔔 Para producción (Meta XR real)
    public void NotifyUserGrab()
    {
        userHasInteracted = true;
        poseCaptured = false;
        stableTimer = 0f;
        rb.WakeUp();

        Debug.Log("🟢 NotifyUserGrab()");
    }

    public void ResetDetection()
    {
        poseCaptured = false;   // Permite capturar una nueva pose
        stableTimer = 0f;       // Reinicia el contador de tiempo
        
        // IMPORTANTE:
        // Mantenemos userHasInteracted en FALSE.
        // Solo pasará a TRUE en el FixedUpdate cuando detecte 
        // que el objeto pasó de ser agarrado (Kinematic) a soltado.
        userHasInteracted = false; 
        
        if (rb != null) rb.WakeUp();
        
        Debug.Log("🔄 Detector listo: Esperando a que el usuario mueva el objeto de nuevo.");
    }
}
