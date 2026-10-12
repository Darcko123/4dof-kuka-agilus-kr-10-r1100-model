/**
 * @file robotics.c
 * @brief Librería de cinemática directa para el brazo robótico 4DOF tipo
 * KUKA Agilus KR10 R1100.
 *
 * @details Provee funciones para inicializar el modelo cinemático
 * (parámetros Denavit-Hartenberg), calcular la transformación homogénea
 * base -> efector final, extraer la posición y orientación del efector y
 * mantener el estado articular/cartesiano del brazo.
 *
 * @author Daniel Ruiz
 * @date October 5, 2026
 * @version 1.0.0
 */

#include "robotics.h"

// ============================================================================
// CONSTANTES PÚBLICAS
// ============================================================================

/**
 * @brief Tabla DH por defecto.
 *
 * @details
 * | i | a [mm] | alpha [rad] | d [mm]  | theta                |
 * |---|--------|-------------|---------|----------------------|
 * | 1 | 0      | +pi/2       | H0 + L1 | q1                   |
 * | 2 | L2     | 0           | 0       | q2                   |
 * | 3 | L3     | 0           | 0       | q3                   |
 * | 4 | L4     | 0           | 0       | q4                   |
 *
 * Con q = 0 el brazo queda extendido en horizontal sobre el eje X0. El
 * hombro parte de su tope horizontal y q2 > 0 lo levanta (q2 = 90° vertical).
 * Para incluir la pinza, usar a4 = L4 + LGRIP.
 */
const Robotics_DH_Param_t Robotics_DefaultDHTable[ROBOTICS_NUM_JOINTS] = {
    /*  a             alpha              d                            offset  dir     min                         max                       */
    { 0.0f,         ROBOTICS_HALF_PI,  ROBOTICS_H0 + ROBOTICS_L1,   0.0f,  1.0f,  ROBOTICS_JOINT_MIN_DEFAULT, ROBOTICS_JOINT_MAX_DEFAULT },
    { ROBOTICS_L2,  0.0f,              0.0f,                        0.0f,  1.0f,  ROBOTICS_JOINT_MIN_DEFAULT, ROBOTICS_JOINT_MAX_DEFAULT },
    { ROBOTICS_L3,  0.0f,              0.0f,                        0.0f,  1.0f,  ROBOTICS_JOINT_MIN_DEFAULT, ROBOTICS_JOINT_MAX_DEFAULT },
    { ROBOTICS_L4,  0.0f,              0.0f,                        0.0f,  1.0f,  ROBOTICS_JOINT_MIN_DEFAULT, ROBOTICS_JOINT_MAX_DEFAULT }
};

// ============================================================================
// VARIABLES PRIVADAS
// ============================================================================

static Robotics_DH_Param_t Robotics_DH[ROBOTICS_NUM_JOINTS];        /**< Tabla DH activa */
static float               Robotics_SinAlpha[ROBOTICS_NUM_JOINTS];  /**< sin(alpha) precalculado por eslabón */
static float               Robotics_CosAlpha[ROBOTICS_NUM_JOINTS];  /**< cos(alpha) precalculado por eslabón */
static uint8_t             Robotics_Initialized = 0;                /**< Bandera para verificar si el módulo está inicializado */

// ============================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS
// ============================================================================

static void robotics_SetIdentity(Robotics_Transform_t* T);
static void robotics_ApplyDH(Robotics_Transform_t* T, float a, float sa, float ca, float d, float theta);

// ============================================================================
// FUNCIONES PRIVADAS
// ============================================================================

/**
 * @brief Carga la transformación identidad.
 *
 * @param T Transformación a inicializar.
 */
static void robotics_SetIdentity(Robotics_Transform_t* T)
{
    T->R[0][0] = 1.0f;  T->R[0][1] = 0.0f;  T->R[0][2] = 0.0f;  T->p[0] = 0.0f;
    T->R[1][0] = 0.0f;  T->R[1][1] = 1.0f;  T->R[1][2] = 0.0f;  T->p[1] = 0.0f;
    T->R[2][0] = 0.0f;  T->R[2][1] = 0.0f;  T->R[2][2] = 1.0f;  T->p[2] = 0.0f;
}

/**
 * @brief Post-multiplica en sitio una transformación por la matriz DH de un
 * eslabón: T = T * A(a, alpha, d, theta).
 *
 * @details La matriz DH estándar es:
 *
 *     [ ct   -st*ca    st*sa   a*ct ]
 *     [ st    ct*ca   -ct*sa   a*st ]
 * A = [ 0       sa       ca      d  ]
 *     [ 0        0        0      1  ]
 *
 * Aprovechando su estructura, cada fila de T se actualiza como:
 *   c0' = ct*c0 + st*c1
 *   u   = ct*c1 - st*c0
 *   c1' = ca*u  + sa*c2
 *   c2' = ca*c2 - sa*u
 *   p'  = p + a*c0' + d*c2
 *
 * Esto requiere 30 multiplicaciones por eslabón frente a las 64 de una
 * multiplicación 4x4 genérica, y no necesita buffer temporal.
 *
 * @param T Transformación acumulada (entrada y salida).
 * @param a Longitud del eslabón [mm].
 * @param sa sin(alpha).
 * @param ca cos(alpha).
 * @param d Offset del eslabón [mm].
 * @param theta Ángulo DH de la articulación [rad].
 */
static void robotics_ApplyDH(Robotics_Transform_t* T, float a, float sa, float ca, float d, float theta)
{
    const float ct = cosf(theta);
    const float st = sinf(theta);

    for(uint32_t r = 0; r < 3U; r++)
    {
        const float c0 = T->R[r][0];
        const float c1 = T->R[r][1];
        const float c2 = T->R[r][2];
        const float x  = ct * c0 + st * c1;
        const float u  = ct * c1 - st * c0;

        T->R[r][0] = x;
        T->R[r][1] = ca * u  + sa * c2;
        T->R[r][2] = ca * c2 - sa * u;
        T->p[r]   += a * x + d * c2;
    }
}

// ============================================================================
// FUNCIONES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa el modelo cinemático del robot.
 *
 * @param dhTable Tabla de ROBOTICS_NUM_JOINTS parámetros DH, o NULL para
 * usar Robotics_DefaultDHTable.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_Init(const Robotics_DH_Param_t* dhTable)
{
    if(dhTable == NULL)
    {
        dhTable = Robotics_DefaultDHTable;
    }

    // Validar la tabla completa antes de modificar el estado del módulo
    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        if(dhTable[i].direction != 1.0f && dhTable[i].direction != -1.0f)
        {
            return ROBOTICS_INVALID_PARAM;
        }

        if(!(dhTable[i].minAngle < dhTable[i].maxAngle))
        {
            return ROBOTICS_INVALID_PARAM;
        }
    }

    // Copiar la tabla y precalcular la trigonometría constante de cada eslabón
    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        Robotics_DH[i]       = dhTable[i];
        Robotics_SinAlpha[i] = sinf(dhTable[i].alpha);
        Robotics_CosAlpha[i] = cosf(dhTable[i].alpha);
    }

    // Marcar como inicializado exitosamente
    Robotics_Initialized = 1U;

    return ROBOTICS_OK;
}

/**
 * @brief Desinicializa el modelo cinemático.
 *
 * @return Robotics_Status_t Estado de la operación (OK).
 */
Robotics_Status_t Robotics_DeInit(void)
{
    Robotics_Initialized = 0U;      // Marca el módulo como no inicializado

    return ROBOTICS_OK;
}

/**
 * @brief Calcula la transformación DH estándar de un eslabón.
 *
 * @param a Longitud del eslabón [mm].
 * @param alpha Torsión del eslabón [rad].
 * @param d Offset del eslabón [mm].
 * @param theta Ángulo de la articulación [rad].
 * @param[out] T Transformación resultante.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_DH_Transform(float a, float alpha, float d, float theta, Robotics_Transform_t* T)
{
    if(T == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    robotics_SetIdentity(T);
    robotics_ApplyDH(T, a, sinf(alpha), cosf(alpha), d, theta);

    return ROBOTICS_OK;
}

/**
 * @brief Compone dos transformaciones homogéneas: Out = A * B.
 *
 * @details
 *   R_out = R_A * R_B
 *   p_out = R_A * p_B + p_A
 *
 * @param A Primera transformación.
 * @param B Segunda transformación.
 * @param[out] Out Transformación resultante (puede coincidir con A o B).
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_TransformMultiply(const Robotics_Transform_t* A, const Robotics_Transform_t* B, Robotics_Transform_t* Out)
{
    Robotics_Transform_t temp;  // Buffer temporal para permitir aliasing con A o B

    if(A == NULL || B == NULL || Out == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    for(uint32_t i = 0; i < 3U; i++)
    {
        for(uint32_t j = 0; j < 3U; j++)
        {
            temp.R[i][j] = A->R[i][0] * B->R[0][j]
                         + A->R[i][1] * B->R[1][j]
                         + A->R[i][2] * B->R[2][j];
        }

        temp.p[i] = A->R[i][0] * B->p[0]
                  + A->R[i][1] * B->p[1]
                  + A->R[i][2] * B->p[2]
                  + A->p[i];
    }

    *Out = temp;

    return ROBOTICS_OK;
}

/**
 * @brief Verifica que los ángulos articulares estén dentro de sus límites.
 *
 * @param q Arreglo de ROBOTICS_NUM_JOINTS ángulos articulares [rad].
 *
 * @return Robotics_Status_t OK, OUT_OF_RANGE, NOT_INITIALIZED o INVALID_PARAM.
 */
Robotics_Status_t Robotics_CheckJointLimits(const float q[ROBOTICS_NUM_JOINTS])
{
    if(Robotics_Initialized != 1U)
    {
        return ROBOTICS_NOT_INITIALIZED;
    }

    if(q == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        // Comparación negada para que un NaN también se considere fuera de rango
        if(!(q[i] >= Robotics_DH[i].minAngle && q[i] <= Robotics_DH[i].maxAngle))
        {
            return ROBOTICS_OUT_OF_RANGE;
        }
    }

    return ROBOTICS_OK;
}

/**
 * @brief Calcula la cinemática directa del robot: T = A1 * A2 * A3 * A4.
 *
 * @details Partiendo de la identidad, se post-multiplica en sitio cada
 * eslabón de la cadena cinemática:
 * - A1: base (0) -> hombro (1)
 * - A2: hombro (1) -> codo (2)
 * - A3: codo (2) -> muñeca (3)
 * - A4: muñeca (3) -> efector final (4)
 *
 * El ángulo DH de cada articulación es theta = direction * q + offset.
 *
 * @param q Arreglo de ROBOTICS_NUM_JOINTS ángulos articulares [rad].
 * @param[out] T Transformación base -> efector final.
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM).
 */
Robotics_Status_t Robotics_ForwardKinematics(const float q[ROBOTICS_NUM_JOINTS], Robotics_Transform_t* T)
{
    if(Robotics_Initialized != 1U)
    {
        return ROBOTICS_NOT_INITIALIZED;
    }

    if(q == NULL || T == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    robotics_SetIdentity(T);

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        const float theta = Robotics_DH[i].direction * q[i] + Robotics_DH[i].offset;

        robotics_ApplyDH(T, Robotics_DH[i].a, Robotics_SinAlpha[i], Robotics_CosAlpha[i], Robotics_DH[i].d, theta);
    }

    return ROBOTICS_OK;
}

/**
 * @brief Extrae la posición y orientación del efector final.
 *
 * @details
 * - Posición: columna de traslación p.
 * - Yaw: el eje Z4 (eje común de los joints 2-4) es horizontal y
 *   perpendicular al plano del brazo. Con alpha1 = +pi/2 se cumple
 *   Z4 = (sin(yaw), -cos(yaw), 0), por lo que yaw = atan2(r02, -r12).
 *   Esta forma sigue siendo válida con la pinza apuntando en vertical,
 *   donde los ángulos de Euler ZYX caen en gimbal lock.
 * - Pitch: ángulo del vector de aproximación X4 respecto a la horizontal
 *   del plano del brazo, h = (cos(yaw), sin(yaw), 0):
 *   pitch = atan2(r20, X4 · h), con X4 · h = r00*(-r12) + r10*r02.
 *   Cubre el rango completo (-pi, pi].
 *
 * @param T Transformación base -> efector final.
 * @param[out] pose Pose del efector final.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_GetPose(const Robotics_Transform_t* T, Robotics_Pose_t* pose)
{
    if(T == NULL || pose == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    // Posición
    pose->x = T->p[0];
    pose->y = T->p[1];
    pose->z = T->p[2];

    // Orientación
    pose->yaw   = atan2f(T->R[0][2], -T->R[1][2]);
    pose->pitch = atan2f(T->R[2][0], T->R[1][0] * T->R[0][2] - T->R[0][0] * T->R[1][2]);

    return ROBOTICS_OK;
}

/**
 * @brief Inicializa la estructura de estado del brazo.
 *
 * @param arm Puntero a la estructura Robotics_Arm_t.
 * @param q0 Ángulos articulares iniciales [rad], o NULL para la posición cero.
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM, OUT_OF_RANGE).
 */
Robotics_Status_t Robotics_InitArm(Robotics_Arm_t* arm, const float q0[ROBOTICS_NUM_JOINTS])
{
    static const float qZero[ROBOTICS_NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};

    if(arm == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    if(q0 == NULL)
    {
        q0 = qZero;
    }

    // Inicializar estructura en un estado conocido
    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        arm->q[i] = 0.0f;
    }
    robotics_SetIdentity(&arm->T);
    arm->pose.x     = 0.0f;
    arm->pose.y     = 0.0f;
    arm->pose.z     = 0.0f;
    arm->pose.pitch = 0.0f;
    arm->pose.yaw   = 0.0f;
    arm->lastError  = ROBOTICS_OK;

    // Establecer los ángulos iniciales
    return Robotics_SetJointAngles(arm, q0);
}

/**
 * @brief Actualiza los ángulos articulares del brazo y recalcula su pose.
 *
 * @param arm Puntero a la estructura Robotics_Arm_t.
 * @param q Nuevos ángulos articulares [rad].
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM, OUT_OF_RANGE).
 */
Robotics_Status_t Robotics_SetJointAngles(Robotics_Arm_t* arm, const float q[ROBOTICS_NUM_JOINTS])
{
    Robotics_Status_t status;

    if(arm == NULL)
    {
        return ROBOTICS_INVALID_PARAM;
    }

    // Validar inicialización, puntero y límites articulares
    status = Robotics_CheckJointLimits(q);
    arm->lastError = status;
    if(status != ROBOTICS_OK)
    {
        return status;
    }

    // Calcular la cinemática directa (no puede fallar tras las validaciones previas)
    (void)Robotics_ForwardKinematics(q, &arm->T);
    (void)Robotics_GetPose(&arm->T, &arm->pose);

    for(uint32_t i = 0; i < ROBOTICS_NUM_JOINTS; i++)
    {
        arm->q[i] = q[i];
    }

    return ROBOTICS_OK;
}
