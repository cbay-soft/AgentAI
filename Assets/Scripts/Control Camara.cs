using UnityEngine;

public class ControlCamara : MonoBehaviour
{
    [Header("Referencias")]
    public Transform objetivo;

    [Header("Configuración de Cámara")]
    public float distanciaCamara = 8f;
    public float alturaCamara = 8f;
    public float suavizado = 3f;
    public float suavizadoRotacion = 60f;

    void LateUpdate()
    {
        if (objetivo == null) return;

        // Calcular posición deseada DETRÁS del jugador (rotación relativa)
        Vector3 offsetRelativo = new Vector3(0f, alturaCamara, -distanciaCamara);
        
        // Rotar el offset según la rotación del jugador
        Vector3 posicionDeseada = objetivo.position + objetivo.rotation * offsetRelativo;

        // Suavizar movimiento de la cámara
        transform.position = Vector3.Lerp(transform.position, posicionDeseada, suavizado * Time.deltaTime);

        // Apuntar hacia el objetivo con suavizado
        Quaternion rotacionDeseada = Quaternion.LookRotation(objetivo.position - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, rotacionDeseada, suavizadoRotacion * Time.deltaTime);
    }
}
