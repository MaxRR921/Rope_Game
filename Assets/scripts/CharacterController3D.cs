using UnityEngine;
using UnityEngine.InputSystem;

public class CharacterController3D : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float jumpForce = 7f;

    private Rigidbody rb;
    private PlayerControls playerInput;
    private InputAction moveAction;
    private InputAction jumpAction;

    private Vector2 moveInput;
    private bool jumpQueued;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        playerInput = new PlayerControls(); // auto-generated class from your input asset
    }

    void OnEnable()
    {
        moveAction = playerInput.Player.Move;
        jumpAction = playerInput.Player.Jump;

        moveAction.Enable();
        jumpAction.Enable();

        jumpAction.performed += OnJump;
    }

    void OnDisable()
    {
        jumpAction.performed -= OnJump;

        moveAction.Disable();
        jumpAction.Disable();
    }

    void FixedUpdate()
    {
        moveInput = moveAction.ReadValue<Vector2>();
        Vector3 direction = new Vector3(moveInput.x, 0, moveInput.y);
        rb.MovePosition(rb.position + direction * moveSpeed * Time.fixedDeltaTime);

        if (jumpQueued)
        {
            rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
            jumpQueued = false;
        }
    }

    void OnJump(InputAction.CallbackContext context)
    {
        if (IsGrounded())
        {
            jumpQueued = true;
        }
    }

    private bool IsGrounded()
    {
        return Physics.Raycast(transform.position, Vector3.down, 1.1f);
    }
}
