using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Sistema de IA para guardia de seguridad con FSM (Finite State Machine)
/// Desarrollado para Unity 6 con optimizaciones de rendimiento
/// @author: Senior Unity Developer
/// @version: 1.0
/// </summary>
[RequireComponent(typeof(NavMeshAgent))]
public class GuardAI : MonoBehaviour
{
    #region Enumeración de Estados

    /// <summary>
    /// Estados posibles del agente (Finite State Machine)
    /// </summary>
    public enum AIState
    {
        Patrol, // Patrullando ruta establecida
        Chase, // Persiguiendo al jugador
        Alert, // Alerta en última posición conocida
    }

    #endregion
    #region Variables Serializadas (Inspector)

    [Header("â•â•â• REFERENCIAS â•â•â•")]
    [SerializeField, Tooltip("Puntos de patrulla en orden secuencial")]
    private Transform[] waypointArray;

    [SerializeField, Tooltip("Transform del jugador objetivo")]
    private Transform playerTarget;

    [Header("â•â•â• VELOCIDADES â•â•â•")]
    [SerializeField, Range(1f, 5f), Tooltip("Velocidad durante patrulla (m/s)")]
    private float patrolSpeed = 2.5f;

    [SerializeField, Range(3f, 10f), Tooltip("Velocidad durante persecución (m/s)")]
    private float chaseSpeed = 6f;

    [Header("â•â•â• CAMPO DE VISIÓN (FOV) â•â•â•")]
    [SerializeField, Range(5f, 30f), Tooltip("Distancia máxima de detección")]
    private float detectionRange = 15f;

    [SerializeField, Range(30f, 180f), Tooltip("Ángulo del cono de visión")]
    private float fieldOfViewAngle = 110f;

    [SerializeField, Tooltip("Altura del origen del raycast (ojos del guardia)")]
    private float eyeHeight = 1.6f;

    [SerializeField, Tooltip("Capas que representan al jugador")]
    private LayerMask playerLayerMask = 1 << 6; // Layer 6: Player

    [SerializeField, Tooltip("Capas que bloquean la visión (paredes, obstáculos)")]
    private LayerMask obstacleLayerMask = 1 << 0; // Layer 0: Default

    [Header("â•â•â• COMPORTAMIENTO â•â•â•")]
    [SerializeField, Range(0.5f, 2f), Tooltip("Distancia para considerar waypoint alcanzado")]
    private float waypointReachThreshold = 1f; // Fixed newline in Tooltip

    [SerializeField, Range(2f, 10f), Tooltip("Duración del estado de alerta(segundos)")]
    private float alertStateDuration = 4f; // Fixed newline in Tooltip

    [SerializeField, Range(0.1f, 1f), Tooltip("Intervalo entre chequeos de visión (optimización)")]
    private float visionCheckInterval = 0.2f; // Fixed newline in Tooltip
    #endregion
    #region Variables Privadas

    // Referencias en caché (optimización)
    private NavMeshAgent navAgent;
    private Transform cachedTransform;

    // Control de estados
    private AIState currentState = AIState.Patrol;

    // Control de patrulla
    private int currentWaypointIndex = 0;

    // Control de alerta
    private Vector3 lastKnownPlayerPos;
    private float alertTimer = 0f;

    // OptimizaciÃ³n de visiÃ³n
    private float visionCheckTimer = 0f;
    private bool playerInSight = false;

    // PosiciÃ³n de los "ojos" del guardia
    private Vector3 EyePosition => cachedTransform.position + Vector3.up * eyeHeight;

    #endregion
    #region MÃ©todos de Unity Lifecycle

    private void Awake()
    {
        // Cachear componentes para optimizaciÃ³n
        navAgent = GetComponent<NavMeshAgent>();
        cachedTransform = transform;

        // ValidaciÃ³n de configuraciÃ³n
        ValidateConfiguration();
    }

    private void Start()
    {
        // Inicializar mÃ¡quina de estados
        TransitionToState(AIState.Patrol);
    }

    private void Update()
    {
        // Ejecutar lÃ³gica del estado actual (Finite State Machine)
        ExecuteCurrentState();

        // Actualizar detecciÃ³n de jugador con intervalo optimizado
        UpdateVisionCheck();
    }

    #endregion
    #region MÃ¡quina de Estados Finitos (FSM)

    /// <summary>
    /// NÃºcleo de la FSM: ejecuta la lÃ³gica del estado actual
    /// </summary>
    private void ExecuteCurrentState()
    {
        switch (currentState)
        {
            case AIState.Patrol:
                ExecutePatrolState();
                break;

            case AIState.Chase:
                ExecuteChaseState();
                break;

            case AIState.Alert:
                ExecuteAlertState();
                break;
        }
    }

    /// <summary>
    /// ESTADO PATRULLA: Movimiento entre waypoints + detecciÃ³n pasiva
    /// </summary>
    private void ExecutePatrolState()
    {
        // TransiciÃ³n a Chase si detecta jugador
        if (playerInSight)
        {
            TransitionToState(AIState.Chase);
            return;
        }

        // LÃ³gica de patrulla cÃ­clica
        if (HasReachedDestination())
        {
            AdvanceToNextWaypoint();
        }
    }

    /// <summary>
    /// ESTADO PERSECUCIÃ“N: Seguir al jugador activamente
    /// </summary>
    private void ExecuteChaseState()
    {
        if (playerInSight)
        {
            // Actualizar destino a posiciÃ³n actual del jugador
            lastKnownPlayerPos = playerTarget.position;
            navAgent.SetDestination(lastKnownPlayerPos);
        }
        else
        {
            // Jugador perdido: transiciÃ³n a alerta
            TransitionToState(AIState.Alert);
        }
    }

    /// <summary>
    /// ESTADO ALERTA: Investigar Ãºltima posiciÃ³n conocida
    /// </summary>
    private void ExecuteAlertState()
    {
        // Si vuelve a ver al jugador, retomar persecuciÃ³n
        if (playerInSight)
        {
            TransitionToState(AIState.Chase);
            return;
        }

        // Esperar en la Ãºltima posiciÃ³n conocida
        if (HasReachedDestination())
        {
            alertTimer += Time.deltaTime;

            // Timeout: volver a patrullar
            if (alertTimer >= alertStateDuration)
            {
                TransitionToState(AIState.Patrol);
            }
        }
    }

    /// <summary>
    /// Gestiona la transiciÃ³n entre estados con lÃ³gica de entrada/salida
    /// </summary>
    private void TransitionToState(AIState newState)
    {
        // === LÃ³gica de SALIDA del estado actual ===
        switch (currentState)
        {
            case AIState.Alert:
                alertTimer = 0f;
                break;
        }

        // === Cambio de estado ===
        AIState previousState = currentState;
        currentState = newState;

        // === LÃ³gica de ENTRADA al nuevo estado ===
        switch (newState)
        {
            case AIState.Patrol:
                navAgent.speed = patrolSpeed;
                navAgent.isStopped = false;
                navAgent.ResetPath();
                navAgent.SetDestination(waypointArray[currentWaypointIndex].position);
                Debug.Log($"[GuardAI] {previousState} â†’ PATROL", this);
                break;

            case AIState.Chase:
                navAgent.speed = chaseSpeed;
                lastKnownPlayerPos = playerTarget.position;
                navAgent.isStopped = false;
                navAgent.ResetPath();
                navAgent.SetDestination(lastKnownPlayerPos);
                Debug.Log($"[GuardAI] {previousState} â†’ CHASE", this);
                break;

            case AIState.Alert:
                navAgent.speed = chaseSpeed; // Mantener velocidad alta
                navAgent.isStopped = false;
                navAgent.ResetPath();
                navAgent.SetDestination(lastKnownPlayerPos);
                alertTimer = 0f;
                Debug.Log($"[GuardAI] {previousState} â†’ ALERT", this);
                break;
        }
    }

    #endregion
    #region Sistema de DetecciÃ³n (Sensor)

    /// <summary>
    /// Actualiza la detecciÃ³n del jugador con intervalo optimizado
    /// Evita raycast cada frame (mejora de rendimiento)
    /// </summary>
    private void UpdateVisionCheck()
    {
        visionCheckTimer += Time.deltaTime;

        if (visionCheckTimer >= visionCheckInterval)
        {
            visionCheckTimer = 0f;
            playerInSight = CanSeePlayer();
        }
    }

    /// <summary>
    /// Sistema de detecciÃ³n de jugador con FOV (Field of View)
    /// Implementa: Distancia + Ãngulo + LÃ­nea de visiÃ³n despejada
    /// </summary>
    /// <returns>True si el jugador es visible</returns>
    private bool CanSeePlayer()
    {
        if (playerTarget == null)
            return false;

        Vector3 eyePos = EyePosition;
        Vector3 playerPos = playerTarget.position + Vector3.up; // Centro del jugador

        // CHECK DE DISTANCIA (mÃ¡s econÃ³mico, se evalÃºa primero)
        float sqrDistance = (playerPos - eyePos).sqrMagnitude;
        if (sqrDistance > detectionRange * detectionRange)
            return false;

        // 2ï¸âƒ£CHECK DE ÃNGULO (FOV cÃ³nico)
        Vector3 directionToPlayer = (playerPos - eyePos).normalized;
        float angleToPlayer = Vector3.Angle(cachedTransform.forward, directionToPlayer);

        if (angleToPlayer > fieldOfViewAngle * 0.5f)
            return false;

        // 3ï¸âƒ£RAYCAST PARA OBSTÃCULOS (mÃ¡s costoso, se evalÃºa Ãºltimo)
        float actualDistance = Mathf.Sqrt(sqrDistance);

        if (Physics.Raycast(eyePos, directionToPlayer, actualDistance, obstacleLayerMask))
        {
            return false; // Hay un obstÃ¡culo bloqueando la visiÃ³n
        }

        // JUGADOR VISIBLE
        return true;
    }

    #endregion
    #region Utilidades de NavegaciÃ³n

    /// <summary>
    /// Verifica si el agente ha llegado al destino actual
    /// </summary>
    private bool HasReachedDestination()
    {
        // Verificar que el path estÃ© calculado y el agente tenga un destino vÃ¡lido
        if (navAgent.pathPending)
            return false;
        if (!navAgent.hasPath || navAgent.pathStatus == NavMeshPathStatus.PathInvalid)
            return true;

        float threshold = navAgent.stoppingDistance + waypointReachThreshold;
        return navAgent.remainingDistance <= threshold;
    }

    /// <summary>
    /// Avanza al siguiente waypoint en la secuencia de patrulla (cÃ­clico)
    /// </summary>
    private void AdvanceToNextWaypoint()
    {
        currentWaypointIndex = (currentWaypointIndex + 1) % waypointArray.Length;
        navAgent.SetDestination(waypointArray[currentWaypointIndex].position);
    }

    #endregion
    #region ValidaciÃ³n y Debug

    /// <summary>
    /// Valida la configuraciÃ³n del componente al iniciar
    /// </summary>
    private void ValidateConfiguration()
    {
        if (waypointArray == null || waypointArray.Length == 0)
        {
            Debug.LogError(
                "[GuardAI] ¡No hay waypoints asignados! Deshabilitando componente.",
                this
            );
            enabled = false;
            return;
        }

        if (playerTarget == null)
        {
            Debug.LogWarning("[GuardAI] No hay jugador asignado. Detección deshabilitada.", this);
        }

        if (navAgent == null)
        {
            Debug.LogError("[GuardAI] ¡NavMeshAgent no encontrado! Añadiendo componente...", this);
            navAgent = gameObject.AddComponent<NavMeshAgent>();
        }
    }

    /// <summary>
    /// VisualizaciÃ³n del campo de visiÃ³n en la Scene View (Editor)
    /// Color verde = Patrulla | Rojo = PersecuciÃ³n | Amarillo = Alerta
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
        {
            cachedTransform = transform;
        }

        Vector3 eyePos = EyePosition;

        // COLOR SEGÃšN ESTADO
        Color gizmoColor = currentState switch
        {
            AIState.Patrol => Color.green,
            AIState.Chase => Color.red,
            AIState.Alert => Color.yellow,
            _ => Color.white,
        };

        Gizmos.color = gizmoColor;

        // ESFERA DE RANGO DE DETECCIÃ“N
        Gizmos.DrawWireSphere(eyePos, detectionRange);

        // CONO DE VISIÃ“N (FOV)
        Vector3 forward = cachedTransform.forward;
        Vector3 leftBoundary = Quaternion.Euler(0, -fieldOfViewAngle * 0.5f, 0) * forward;
        Vector3 rightBoundary = Quaternion.Euler(0, fieldOfViewAngle * 0.5f, 0) * forward;

        Gizmos.DrawLine(eyePos, eyePos + leftBoundary * detectionRange);
        Gizmos.DrawLine(eyePos, eyePos + rightBoundary * detectionRange);

        // ARCO DEL FOV (visualizaciÃ³n mejorada)
        DrawFieldOfViewArc(eyePos, forward, fieldOfViewAngle, detectionRange, gizmoColor);

        // LÃNEA HACIA EL JUGADOR SI ESTÃ EN SIGHT
        if (Application.isPlaying && playerInSight && playerTarget != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(eyePos, playerTarget.position + Vector3.up);
        }

        // WAYPOINTS DE PATRULLA
        if (waypointArray != null && waypointArray.Length > 0)
        {
            Gizmos.color = Color.cyan;
            for (int i = 0; i < waypointArray.Length; i++)
            {
                if (waypointArray[i] == null)
                    continue;

                Vector3 wpPos = waypointArray[i].position;
                Gizmos.DrawWireSphere(wpPos, 0.5f);

                // LÃ­nea hacia el siguiente waypoint
                int nextIndex = (i + 1) % waypointArray.Length;
                if (waypointArray[nextIndex] != null)
                {
                    Gizmos.DrawLine(wpPos, waypointArray[nextIndex].position);
                }
            }
        }
    }

    /// <summary>
    /// Dibuja un arco para visualizar mejor el FOV
    /// </summary>
    private void DrawFieldOfViewArc(
        Vector3 origin,
        Vector3 forward,
        float angle,
        float radius,
        Color color
    )
    {
        int segments = 20;
        float angleStep = angle / segments;
        Vector3 previousPoint = Quaternion.Euler(0, -angle * 0.5f, 0) * forward * radius;

        Gizmos.color = new Color(color.r, color.g, color.b, 0.3f);

        for (int i = 1; i <= segments; i++)
        {
            float currentAngle = -angle * 0.5f + angleStep * i;
            Vector3 currentPoint = Quaternion.Euler(0, currentAngle, 0) * forward * radius;
            Gizmos.DrawLine(origin + previousPoint, origin + currentPoint);
            previousPoint = currentPoint;
        }
    }

    #endregion
}
