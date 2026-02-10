using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class MovJugador : MonoBehaviour
{
    [Header("Movimiento")]
    public float velocidad = 5f;
    public float velAngular = 200f;
    public float gravedad = -9.81f;

    private CharacterController controller;
    private float velocidadVertical = 0f;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        
        // Si no tiene CharacterController, añadirlo
        if (controller == null)
        {
            controller = gameObject.AddComponent<CharacterController>();
            controller.radius = 0.5f;
            controller.height = 2f;
            controller.center = new Vector3(0, 1f, 0);
        }
    }

    void Update()
    {
        // Leer inputs
        float moveForward = 0f;
        float rotateInput = 0f;

        if (Input.GetKey(KeyCode.W)) moveForward = 1f;
        else if (Input.GetKey(KeyCode.S)) moveForward = -1f;

        if (Input.GetKey(KeyCode.A)) rotateInput = -1f;
        else if (Input.GetKey(KeyCode.D)) rotateInput = 1f;

        // ROTACIÓN - Totalmente independiente de colisiones
        if (rotateInput != 0f)
        {
            transform.Rotate(0, rotateInput * velAngular * Time.deltaTime, 0);
        }

        // MOVIMIENTO - CharacterController maneja colisiones sin afectar rotación
        Vector3 movimiento = transform.forward * moveForward * velocidad;

        // Aplicar gravedad
        if (controller.isGrounded)
        {
            velocidadVertical = -2f; // Pequeño valor para mantenerlo pegado al suelo
        }
        else
        {
            velocidadVertical += gravedad * Time.deltaTime;
        }

        movimiento.y = velocidadVertical;

        // Mover el personaje (respeta colisiones pero no afecta rotación)
        controller.Move(movimiento * Time.deltaTime);
    }
}