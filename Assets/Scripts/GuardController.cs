using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Controlador de IA para un guardia que patrulla, detecta y persigue al jugador
/// Implementa separación entre lógica de decisión y sistema de movimiento
/// </summary>
public class GuardController : MonoBehaviour
{
    #region Estados y Enumeraciones

    /// <summary>
    /// Estados posibles del agente inteligente
    /// </summary>
    public enum GuardState
    {
        Patrol, // Patrullando entre waypoints
        Chase, // Persiguiendo al jugador
        Alert, // En estado de alerta tras perder de vista al jugador
    }

    #endregion
    #region Variables de Inspector

    [Header("Referencias")]
    [Tooltip("Array de puntos de patrulla")]
    public Transform[] patrolWaypoints;

    [Tooltip("Transform del jugador a detectar")]
    public Transform player;

    [Header("Parámetros de Movimiento")]
    [Tooltip("Velocidad durante la patrulla")]
    public float patrolSpeed = 2f;

    [Tooltip("Velocidad durante la persecución")]
    public float chaseSpeed = 5f;

    [Header("Parámetros de Detección")]
    [Tooltip("Rango máximo de visión en metros")]
    public float viewRange = 10f;

    [Tooltip("Ángulo del campo de visión en grados")]
    [Range(0, 180)]
    public float viewAngle = 90f;

    [Tooltip("Capa del jugador para detección")]
    public LayerMask playerLayer;

    [Tooltip("Capa de obstáculos que bloquean la visión")]
    public LayerMask obstacleLayer;

    [Header("Parámetros de Comportamiento")]
    [Tooltip("Tiempo en estado de alerta antes de volver a patrullar")]
    public float alertDuration = 3f;

    [Tooltip("Distancia mínima al waypoint para considerarlo alcanzado")]
    public float waypointReachDistance = 1f;

    #endregion
    #region Variables Privadas

    private NavMeshAgent navAgent;
    private GuardState currentState;
    private int currentWaypointIndex = 0;
    private float alertTimer = 0f;
    private Vector3 lastKnownPlayerPosition;

    #endregion
    #region Métodos de Unity

    void Start()
    {
        // Obtener referencia al NavMeshAgent (capa de movimiento)
        navAgent = GetComponent<NavMeshAgent>();

        // Validar configuración
        if (patrolWaypoints.Length == 0)
        {
            Debug.LogError("¡No hay waypoints asignados!");
            enabled = false;
            return;
        }

        if (player == null)
        {
            Debug.LogError("¡No hay jugador asignado!");
            enabled = false;
            return;
        }

        // Inicializar estado
        SetState(GuardState.Patrol);
    }

    void Update()
    {
        // Lógica de decisión: ejecutar comportamiento según estado actual
        switch (currentState)
        {
            case GuardState.Patrol:
                PatrolBehavior();
                break;

            case GuardState.Chase:
                ChaseBehavior();
                break;

            case GuardState.Alert:
                AlertBehavior();
                break;
        }
    }

    #endregion
    #region Métodos de Comportamiento (Lógica de Decisión)

    /// <summary>
    /// Comportamiento de patrulla: moverse entre waypoints y detectar al jugador
    /// </summary>
    private void PatrolBehavior()
    {
        // Comprobar detección del jugador
        if (CanSeePlayer())
        {
            SetState(GuardState.Chase);
            return;
        }

        // Navegar hacia el waypoint actual
        if (!navAgent.pathPending && navAgent.remainingDistance <= waypointReachDistance)
        {
            // Waypoint alcanzado, ir al siguiente
            currentWaypointIndex = (currentWaypointIndex + 1) % patrolWaypoints.Length;
            SetDestination(patrolWaypoints[currentWaypointIndex].position);
        }
    }

    /// <summary>
    /// Comportamiento de persecución: seguir al jugador mientras sea visible
    /// </summary>
    private void ChaseBehavior()
    {
        if (CanSeePlayer())
        {
            // Jugador visible: actualizar destino a su posición actual
            lastKnownPlayerPosition = player.position;
            SetDestination(player.position);
        }
        else
        {
            // Jugador perdido de vista: entrar en estado de alerta
            SetState(GuardState.Alert);
        }
    }

    /// <summary>
    /// Comportamiento de alerta: buscar al jugador en última posición conocida
    /// </summary>
    private void AlertBehavior()
    {
        // Verificar si el jugador vuelve a ser visible
        if (CanSeePlayer())
        {
            SetState(GuardState.Chase);
            return;
        }

        // Ir a la última posición conocida
        if (!navAgent.pathPending && navAgent.remainingDistance <= waypointReachDistance)
        {
            // Incrementar temporizador de alerta
            alertTimer += Time.deltaTime;

            if (alertTimer >= alertDuration)
            {
                // Tiempo agotado: volver a patrullar
                SetState(GuardState.Patrol);
            }
        }
    }

    #endregion
    #region Métodos de Percepción (Sensores)

    /// <summary>
    /// Detecta si el jugador está dentro del campo de visión
    /// Aplica: detección por distancia + ángulo + verificación de obstáculos
    /// </summary>
    /// <returns>True si el jugador es visible</returns>
    private bool CanSeePlayer()
    {
        // 1. Verificar distancia (optimización: más rápido que raycast)
        float distanceToPlayer = Vector3.Distance(transform.position, player.position);
        if (distanceToPlayer > viewRange)
            return false;

        // 2. Calcular dirección hacia el jugador
        Vector3 directionToPlayer = (player.position - transform.position).normalized;

        // 3. Verificar ángulo de visión
        // Producto punto entre forward del guardia y dirección al jugador
        float angle = Vector3.Angle(transform.forward, directionToPlayer);
        if (angle > viewAngle / 2f)
            return false;

        // 4. Raycast para verificar obstáculos (línea de visión despejada)
        // Este es el paso más costoso, por eso se deja al final
        if (
            Physics.Raycast(
                transform.position + Vector3.up,
                directionToPlayer,
                distanceToPlayer,
                obstacleLayer
            )
        )
        {
            return false; // Hay un obstáculo bloqueando la visión
        }

        return true; // ¡Jugador detectado!
    }

    #endregion
    #region Métodos de Acción (Sistema de Movimiento)

    /// <summary>
    /// Establece un destino para el NavMeshAgent
    /// Esta es la interfaz entre la lógica de decisión y el sistema de movimiento
    /// </summary>
    /// <param name="destination">Posición objetivo</param>
    private void SetDestination(Vector3 destination)
    {
        if (navAgent.isOnNavMesh)
        {
            navAgent.SetDestination(destination);
        }
    }

    #endregion
    #region Métodos de Gestión de Estados

    /// <summary>
    /// Cambia el estado actual y ejecuta lógica de transición
    /// </summary>
    /// <param name="newState">Nuevo estado a establecer</param>
    private void SetState(GuardState newState)
    {
        // Lógica de salida del estado anterior
        switch (currentState)
        {
            case GuardState.Alert:
                alertTimer = 0f;
                break;
        }

        // Cambiar estado
        currentState = newState;

        // Lógica de entrada al nuevo estado
        switch (newState)
        {
            case GuardState.Patrol:
                navAgent.speed = patrolSpeed;
                SetDestination(patrolWaypoints[currentWaypointIndex].position);
                Debug.Log("Estado: PATRULLA");
                break;

            case GuardState.Chase:
                navAgent.speed = chaseSpeed;
                Debug.Log("Estado: PERSECUCIÓN");
                break;

            case GuardState.Alert:
                navAgent.speed = chaseSpeed;
                SetDestination(lastKnownPlayerPosition);
                Debug.Log("Estado: ALERTA");
                break;
        }
    }

    #endregion
    #region Visualización de Debug

    /// <summary>
    /// Dibuja gizmos para visualizar el campo de visión en el editor
    /// </summary>
    private void OnDrawGizmosSelected()
    {
        // Color según estado
        Gizmos.color =
            currentState == GuardState.Chase ? Color.red
            : currentState == GuardState.Alert ? Color.yellow
            : Color.green;

        // Dibujar rango de visión
        Gizmos.DrawWireSphere(transform.position, viewRange);

        // Dibujar líneas del campo de visión
        Vector3 leftBoundary =
            Quaternion.Euler(0, -viewAngle / 2f, 0) * transform.forward * viewRange;
        Vector3 rightBoundary =
            Quaternion.Euler(0, viewAngle / 2f, 0) * transform.forward * viewRange;

        Gizmos.DrawLine(transform.position, transform.position + leftBoundary);
        Gizmos.DrawLine(transform.position, transform.position + rightBoundary);

        // Si detecta al jugador, dibujar línea hacia él
        if (Application.isPlaying && CanSeePlayer())
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, player.position);
        }
    }

    #endregion
}
