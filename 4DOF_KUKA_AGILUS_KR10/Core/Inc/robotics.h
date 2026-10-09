/**
 * @file robotics.h
 * @brief Librería de cinemática directa para el brazo robótico 4DOF tipo
 * KUKA Agilus KR10 R1100.
 *
 * @details Provee funciones para inicializar el modelo cinemático
 * (parámetros Denavit-Hartenberg), calcular la transformación homogénea
 * base -> efector final, extraer la posición y orientación del efector y
 * mantener el estado articular/cartesiano del brazo.
 *
 * Configuración cinemática (DH estándar):
 * - Joint 1: giro de la base (yaw) alrededor del eje vertical Z0.
 * - Joint 2: hombro (pitch).
 * - Joint 3: codo (pitch).
 * - Joint 4: muñeca (pitch).
 *
 * El marco base {0} se ubica en el suelo, bajo el eje del joint 1, con Z0
 * vertical hacia arriba. Todas las longitudes están en milímetros y todos
 * los ángulos en radianes.
 *
 * La librería no depende de la HAL, por lo que es portable a cualquier
 * plataforma con FPU de precisión simple (todas las operaciones usan float).
 *
 * @author Daniel Ruiz
 * @date October 5, 2026
 * @version 1.0.0
 */

#ifndef ROBOTICS_H
#define ROBOTICS_H

// ============================================================================
// INCLUDES
// ============================================================================

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// MACROS Y CONSTANTES
// ============================================================================

/**
 * @brief Número de articulaciones del brazo.
 */
#define ROBOTICS_NUM_JOINTS     4U

/**
 * @brief Constante PI en precisión simple.
 *
 * @note Se evita M_PI (double) para no forzar aritmética de doble precisión
 * por software en el Cortex-M4F, cuya FPU solo soporta float.
 */
#define ROBOTICS_PI             3.14159265358979f
#define ROBOTICS_HALF_PI        (ROBOTICS_PI * 0.5f)

/**
 * @brief Conversión entre grados y radianes (evaluada en tiempo de compilación
 * cuando el argumento es constante).
 */
#define ROBOTICS_DEG2RAD(deg)   ((deg) * (ROBOTICS_PI / 180.0f))
#define ROBOTICS_RAD2DEG(rad)   ((rad) * (180.0f / ROBOTICS_PI))

/**
 * @brief Parámetros físicos del robot (en milímetros).
 */
#define ROBOTICS_H0             35.0f   /**< Altura de la base desde el suelo */
#define ROBOTICS_L1             30.0f   /**< Offset vertical del joint1 al plano del joint2 */
#define ROBOTICS_L2             67.0f   /**< Longitud del brazo proximal (hombro) */
#define ROBOTICS_L3             61.0f   /**< Longitud del brazo distal (codo) */
#define ROBOTICS_L4             29.0f   /**< Longitud del antebrazo (muñeca) */
#define ROBOTICS_LGRIP          45.0f   /**< Longitud de la pinza (no incluida en la tabla DH por defecto) */

/**
 * @brief Límites articulares por defecto (rango de 180° del servo centrado en 0).
 */
#define ROBOTICS_JOINT_MIN_DEFAULT  (-ROBOTICS_HALF_PI)
#define ROBOTICS_JOINT_MAX_DEFAULT  ( ROBOTICS_HALF_PI)

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

/**
 * @brief Enumeración para estados de retorno de la librería.
 */
typedef enum {
    ROBOTICS_OK = 0,                /**< Operación exitosa */
    ROBOTICS_ERROR = 1,             /**< Error en la operación */
    ROBOTICS_NOT_INITIALIZED = 2,   /**< Módulo no inicializado */
    ROBOTICS_INVALID_PARAM = 3,     /**< Parámetro inválido */
    ROBOTICS_OUT_OF_RANGE = 4       /**< Ángulo articular fuera de límites */
} Robotics_Status_t;

/**
 * @brief Parámetros Denavit-Hartenberg (convención estándar) de un eslabón.
 *
 * @details El ángulo DH efectivo de la articulación se calcula como:
 *   theta = direction * q + offset
 * lo que permite compensar el sentido de montaje del servo y la posición
 * de cero mecánico sin modificar el resto del modelo.
 */
typedef struct {
    float a;            /**< Longitud del eslabón, a lo largo de X(i) [mm] */
    float alpha;        /**< Torsión del eslabón, alrededor de X(i) [rad] */
    float d;            /**< Offset del eslabón, a lo largo de Z(i-1) [mm] */
    float offset;       /**< Offset angular sumado a theta [rad] */
    float direction;    /**< Sentido de giro de la articulación (+1.0f o -1.0f) */
    float minAngle;     /**< Límite inferior de q [rad] */
    float maxAngle;     /**< Límite superior de q [rad] */
} Robotics_DH_Param_t;

/**
 * @brief Transformación homogénea compacta.
 *
 * @details Representa la matriz 4x4
 *     [ R  p ]
 *     [ 0  1 ]
 * almacenando solo la rotación R (3x3) y la traslación p (3x1). La última
 * fila es constante, por lo que omitirla ahorra memoria (48 vs 64 bytes) y
 * operaciones en cada composición.
 */
typedef struct {
    float R[3][3];      /**< Matriz de rotación; la columna j es el eje j del marco en coordenadas base */
    float p[3];         /**< Vector de posición del origen del marco [mm] */
} Robotics_Transform_t;

/**
 * @brief Pose del efector final expresada en el marco base.
 *
 * @details Al tener solo 4 grados de libertad (yaw + 3 pitch paralelos), la
 * orientación alcanzable del efector queda descrita por dos ángulos; el roll
 * no es controlable. La orientación completa sigue disponible en
 * Robotics_Transform_t::R.
 */
typedef struct {
    float x;            /**< Coordenada X del efector final [mm] */
    float y;            /**< Coordenada Y del efector final [mm] */
    float z;            /**< Coordenada Z del efector final, desde el suelo [mm] */
    float pitch;        /**< Ángulo del vector de aproximación sobre la horizontal; 0 = pinza nivelada, >0 hacia arriba [rad] */
    float yaw;          /**< Rumbo del plano del brazo alrededor de Z0 [rad] */
} Robotics_Pose_t;

/**
 * @brief Estado del brazo robótico.
 */
typedef struct {
    float q[ROBOTICS_NUM_JOINTS];   /**< Ángulos articulares actuales [rad] */
    Robotics_Transform_t T;         /**< Transformación base -> efector final para q */
    Robotics_Pose_t pose;           /**< Pose del efector final para q */
    Robotics_Status_t lastError;    /**< Estado de la última operación realizada sobre el brazo */
} Robotics_Arm_t;

/**
 * @brief Tabla DH por defecto construida a partir de los parámetros físicos.
 *
 * @details Puede copiarse y modificarse para pasarla a Robotics_Init().
 */
extern const Robotics_DH_Param_t Robotics_DefaultDHTable[ROBOTICS_NUM_JOINTS];

// ============================================================================
// FUNCIONES INLINE
// ============================================================================

/**
 * @brief Limita un ángulo a un rango específico.
 *
 * @param angle Ángulo a limitar.
 * @param min Límite mínimo.
 * @param max Límite máximo.
 *
 * @return float Ángulo limitado al rango [min, max].
 */
static inline float Robotics_ConstrainAngle(float angle, float min, float max)
{
    if(angle < min) return min;
    if(angle > max) return max;
    return angle;
}

// ============================================================================
// PROTOTIPOS DE FUNCIONES PÚBLICAS
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el modelo cinemático del robot.
 *
 * @details Copia la tabla DH y precalcula sin(alpha) y cos(alpha) de cada
 * eslabón, ya que son constantes durante la ejecución.
 *
 * @param dhTable Tabla de ROBOTICS_NUM_JOINTS parámetros DH, o NULL para
 * usar Robotics_DefaultDHTable.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_Init(const Robotics_DH_Param_t* dhTable);

/**
 * @brief Desinicializa el modelo cinemático.
 *
 * @return Robotics_Status_t Estado de la operación (OK).
 */
Robotics_Status_t Robotics_DeInit(void);

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
Robotics_Status_t Robotics_DH_Transform(float a, float alpha, float d, float theta, Robotics_Transform_t* T);

/**
 * @brief Compone dos transformaciones homogéneas: Out = A * B.
 *
 * @note Out puede coincidir con A o con B.
 *
 * @param A Primera transformación.
 * @param B Segunda transformación.
 * @param[out] Out Transformación resultante.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_TransformMultiply(const Robotics_Transform_t* A, const Robotics_Transform_t* B, Robotics_Transform_t* Out);

/**
 * @brief Verifica que los ángulos articulares estén dentro de sus límites.
 *
 * @param q Arreglo de ROBOTICS_NUM_JOINTS ángulos articulares [rad].
 *
 * @return Robotics_Status_t OK si todos están en rango, OUT_OF_RANGE si
 * alguno lo excede (o es NaN), NOT_INITIALIZED o INVALID_PARAM.
 */
Robotics_Status_t Robotics_CheckJointLimits(const float q[ROBOTICS_NUM_JOINTS]);

/**
 * @brief Calcula la cinemática directa del robot: T = A1 * A2 * A3 * A4.
 *
 * @note No verifica límites articulares; usar Robotics_CheckJointLimits()
 * o Robotics_SetJointAngles() si se requiere.
 *
 * @param q Arreglo de ROBOTICS_NUM_JOINTS ángulos articulares [rad]
 * (base, hombro, codo, muñeca).
 * @param[out] T Transformación base -> efector final.
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM).
 */
Robotics_Status_t Robotics_ForwardKinematics(const float q[ROBOTICS_NUM_JOINTS], Robotics_Transform_t* T);

/**
 * @brief Extrae la posición y orientación del efector final.
 *
 * @note Asume la estructura de la tabla por defecto: alpha1 = +pi/2 y
 * joints 2-4 paralelos (alpha2 = alpha3 = alpha4 = 0).
 *
 * @param T Transformación base -> efector final.
 * @param[out] pose Pose del efector final.
 *
 * @return Robotics_Status_t Estado de la operación (OK, INVALID_PARAM).
 */
Robotics_Status_t Robotics_GetPose(const Robotics_Transform_t* T, Robotics_Pose_t* pose);

/**
 * @brief Inicializa la estructura de estado del brazo.
 *
 * @param arm Puntero a la estructura Robotics_Arm_t.
 * @param q0 Ángulos articulares iniciales [rad], o NULL para la posición cero.
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM, OUT_OF_RANGE).
 */
Robotics_Status_t Robotics_InitArm(Robotics_Arm_t* arm, const float q0[ROBOTICS_NUM_JOINTS]);

/**
 * @brief Actualiza los ángulos articulares del brazo y recalcula su pose.
 *
 * @details Si algún ángulo está fuera de límites, el estado del brazo no se
 * modifica (salvo arm->lastError) y se retorna ROBOTICS_OUT_OF_RANGE.
 *
 * @param arm Puntero a la estructura Robotics_Arm_t.
 * @param q Nuevos ángulos articulares [rad].
 *
 * @return Robotics_Status_t Estado de la operación (OK, NOT_INITIALIZED,
 * INVALID_PARAM, OUT_OF_RANGE).
 */
Robotics_Status_t Robotics_SetJointAngles(Robotics_Arm_t* arm, const float q[ROBOTICS_NUM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_H */
