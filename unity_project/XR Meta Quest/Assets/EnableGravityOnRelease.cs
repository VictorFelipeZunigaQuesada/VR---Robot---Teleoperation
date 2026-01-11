using UnityEngine;
using Oculus.Interaction;

[RequireComponent(typeof(Rigidbody))]
public class EnableGravityOnRelease : MonoBehaviour
{
    private Rigidbody rb;
    private Grabbable grabbable;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        grabbable = GetComponent<Grabbable>();

        // Estado inicial: flota
        rb.useGravity = false;
        rb.isKinematic = false;
    }

    void OnEnable()
    {
        if (grabbable != null)
        {
            grabbable.WhenPointerEventRaised += OnPointerEvent;
        }
    }

    void OnDisable()
    {
        if (grabbable != null)
        {
            grabbable.WhenPointerEventRaised -= OnPointerEvent;
        }
    }

    private void OnPointerEvent(PointerEvent evt)
    {
        // Cuando se suelta
        if (evt.Type == PointerEventType.Unselect)
        {
            rb.useGravity = true;
            rb.isKinematic = false;
        }
    }
}
