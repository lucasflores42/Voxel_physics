using UnityEngine;
using System.Collections.Generic;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Transform))]
public class PlayerRBController : MonoBehaviour
{
    public SimulationManager simManager; // optional; will Find if null
    public List<Particle> particles; // optional direct reference
    public List<RigidBodyData> rigidbodies; // optional direct reference
    public int playerRbId = 1;
    public float moveSpeed = 5f;
    public float runSpeed = 8f;
    public float lookSensitivity = 2f;
    public Transform cameraTransform;
    public bool thirdPerson = true;
    public Vector3 thirdPersonOffset = new Vector3(0f, 1.6f, -3f);
    public float cameraFollowSpeed = 10f;
    public float cameraLookAtHeight = 1.2f;

    RigidBodyData playerRb;
    float yaw;
    float pitch;

    void Start()
    {
        if (simManager == null)
            simManager = UnityEngine.Object.FindFirstObjectByType<SimulationManager>();

        if (cameraTransform == null)
        {
            var cam = Camera.main;
            if (cam != null) cameraTransform = cam.transform;
        }

        // if camera is parented to player in scene, detach so we control it explicitly
        if (cameraTransform != null && cameraTransform.parent == transform)
            cameraTransform.parent = null;

        Cursor.lockState = CursorLockMode.Locked;
    }

    void Update()
    {
        // ensure we have particle/rigidbody references
        if (rigidbodies == null && simManager != null)
            rigidbodies = new List<RigidBodyData>(simManager.RigidBodies as IEnumerable<RigidBodyData> ?? new RigidBodyData[0]);

        if (particles == null && simManager != null)
            particles = new List<Particle>(simManager.Particles as IEnumerable<Particle> ?? new Particle[0]);

        if (playerRb == null && rigidbodies != null)
        {
            foreach (var rb in rigidbodies) if (rb.id == playerRbId) { playerRb = rb; break; }
        }

        // look (new Input System)
        Vector2 mouse;
        if (Mouse.current != null)
        {
            var d = Mouse.current.delta.ReadValue();
            mouse = new Vector2(d.x, d.y);
        }
        else
        {
            mouse = Vector2.zero;
        }
        yaw += mouse.x * lookSensitivity;
        pitch -= mouse.y * lookSensitivity;
        pitch = Mathf.Clamp(pitch, -80f, 80f);

        transform.localRotation = Quaternion.Euler(0f, yaw, 0f);
        if (cameraTransform != null) cameraTransform.localRotation = Quaternion.Euler(pitch, 0f, 0f);

        // movement (WASD or arrow keys) using new Input System
        float forward = 0f;
        float strafe = 0f;
        if (Keyboard.current != null)
        {
            if (Keyboard.current.wKey.isPressed || Keyboard.current.upArrowKey.isPressed) forward += 1f;
            if (Keyboard.current.sKey.isPressed || Keyboard.current.downArrowKey.isPressed) forward -= 1f;
            if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed) strafe += 1f;
            if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed) strafe -= 1f;
        }
        Vector3 dir = (transform.forward * forward + transform.right * strafe);
        if (dir.sqrMagnitude > 1f) dir.Normalize();

        bool run = Keyboard.current != null && Keyboard.current.leftShiftKey.isPressed;
        float speed = run ? runSpeed : moveSpeed;

        if (playerRb != null && playerRb.physics == 1)
        {
            float vy = playerRb.velocity.y;
            Vector3 hor = new Vector3(dir.x * speed, 0f, dir.z * speed);
            playerRb.velocity = hor + new Vector3(0f, vy, 0f);

            float angVelY = mouse.x * lookSensitivity * Mathf.Deg2Rad;
            playerRb.angularVelocity = new Vector3(0f, angVelY, 0f);
        }
        else
        {
            // fallback: move transform if RB not available
            transform.position += (transform.forward * forward + transform.right * strafe) * speed * Time.deltaTime;
        }

        // third-person camera follow
        if (cameraTransform != null && thirdPerson)
        {
            if (playerRb != null)
            {
                Vector3 desired = playerRb.centerOfMass + Quaternion.Euler(0f, yaw, 0f) * thirdPersonOffset;
                cameraTransform.position = Vector3.Lerp(cameraTransform.position, desired, cameraFollowSpeed * Time.deltaTime);
                Vector3 lookTarget = playerRb.centerOfMass + Vector3.up * cameraLookAtHeight;
                cameraTransform.LookAt(lookTarget);
            }
            else
            {
                // keep a default local offset if no RB exists
                cameraTransform.position = Vector3.Lerp(cameraTransform.position, transform.position + thirdPersonOffset, cameraFollowSpeed * Time.deltaTime);
                cameraTransform.LookAt(transform.position + Vector3.up * cameraLookAtHeight);
            }
        }
    }
}