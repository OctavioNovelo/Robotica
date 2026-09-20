#pragma once

// =============================================================
//  UMouse — settings.h
//  UN SOLO LUGAR para calibrar todo.
//  Cambia PROFILE para alternar entre prueba y competencia.
// =============================================================

// ── PERFIL ───────────────────────────────────────────────────
#define PROFILE  1

// ── MODO DE NAVEGACIÓN ───────────────────────────────────────
// 1 = SOLO ENCODERS — el BNO085 NI SIQUIERA se inicializa (no se le
//     toca el pin de reset, no se llama gyroInit() en absoluto). Úsalo
//     para aislar por completo si el BNO es la causa de un reinicio o
//     comportamiento raro del sistema.
// 2 = SOLO BNO — los giros y la corrección de rumbo en avance SIEMPRE
//     usan el gyro, SIN caer a encoders aunque el BNO falle o quede
//     "zombie" (para poder VER el problema del BNO sin que el
//     fallback lo tape/oculte).
// 3 = AMBOS — BNO con fallback automático a encoders si no está vivo
//     (gyroIsAlive()). Este es el modo de competencia (el que se
//     estaba usando hasta ahora).
#define NAV_MODE  2

#define NAV_USE_GYRO             (NAV_MODE == 2 || NAV_MODE == 3)
#define NAV_USE_ENCODER_FALLBACK (NAV_MODE == 3)

// ── GEOMETRÍA ────────────────────────────────────────────────
#define WHEEL_DIAM_MM    33.5f   
#define TRACK_WIDTH_MM   90.0f  

// ── ENCODER ──────────────────────────────────────────────────
#define ENC_PPR_MOTOR    7      
#define ENC_GEAR_RATIO   30     
#define ENC_EDGES        2      

// Derivados — no tocar
#define ENC_TICKS_PER_REV  ((float)(ENC_PPR_MOTOR * ENC_GEAR_RATIO * ENC_EDGES))
#define WHEEL_CIRC_MM      ((float)(3.14159265f * WHEEL_DIAM_MM))
#define TICKS_PER_MM       ((float)(ENC_TICKS_PER_REV / WHEEL_CIRC_MM))

// ── LABERINTO ────────────────────────────────────────────────
#if PROFILE == 1
  #define CELL_MM   155.0f   //160
#else
  #define CELL_MM   190.0f   
#endif

#if PROFILE == 1
  #define PRESHIFT_MM   10.5f
#else
  #define PRESHIFT_MM   17.5f
#endif

// ── COMPENSACIÓN DE FRENADO (overshoot por inercia) ───────────
// Por más rápido que frenes (motorBrakeAll), el robot NO se detiene
// en el instante en que el encoder marca la meta: sigue avanzando
// unos milímetros por inercia mientras se aplica el freno+coast. Esto
// hace que el robot SIEMPRE llegue un poco más lejos de lo calculado,
// y se nota MUCHO más en movimientos cortos (como el preshift antes
// de girar, que debe dejarte a la mitad de la celda) que en un avance
// completo — por eso "se pasa" en el preshift aunque el avance largo
// se vea razonable.
//
// CÓMO CALIBRAR: con el robot detenido, márcale el punto de arranque
// en el piso con cinta. Hazlo correr varios preshifts/avances (puedes
// usar PAGE_MOTOR o una prueba simple) y mide en mm cuánto pasa del
// punto donde debería quedar quieto.
//   - Si se sigue pasando de largo → SUBE STOP_OVERSHOOT_MM.
//   - Si se empieza a quedar corto → BAJA STOP_OVERSHOOT_MM (mínimo 0).
// Afecta TODOS los avances/preshifts por igual (se resta antes de
// convertir a ticks), así que un solo número corrige el patrón.
#define STOP_OVERSHOOT_MM   6.0f

#define CELL_FWD_MM    ((float)(CELL_MM - PRESHIFT_MM - STOP_OVERSHOOT_MM))
#define ENC_CELL_FWD   ((int)(CELL_FWD_MM * TICKS_PER_MM))
#define ENC_PRESHIFT   ((int)((PRESHIFT_MM - STOP_OVERSHOOT_MM) * TICKS_PER_MM))

#define TURN90_ARC_MM  ((float)(3.14159265f * TRACK_WIDTH_MM / 4.0f))
#define ENC_TURN90_CALC ((int)(TURN90_ARC_MM * TICKS_PER_MM))

#define ENC_TURN90  ENC_TURN90_CALC

#if PROFILE == 1
  #define INITIAL_ADVANCE_MM  10.5f
#else
  #define INITIAL_ADVANCE_MM  65.0f
#endif
// También se le resta STOP_OVERSHOOT_MM: es un avance corto (como el
// preshift), así que el overshoot por inercia se nota igual de fuerte aquí.
#define ENC_INITIAL_ADVANCE  ((int)((INITIAL_ADVANCE_MM - STOP_OVERSHOOT_MM) * TICKS_PER_MM))

// ── BATERÍA / DIVISOR RESISTIVO ──────────────────────────────
#define VBAT_R_TOP_OHM     3272.0f
#define VBAT_R_BOTTOM_OHM  987.0f
#define VBAT_DIV_RATIO     ((float)((VBAT_R_TOP_OHM + VBAT_R_BOTTOM_OHM) / VBAT_R_BOTTOM_OHM))

// ── MOTORES ──────────────────────────────────────────────────
#define V_MOTOR_LIMIT   6.0f   
#define MOTOR_KICK_MIN  50     

#define MOTOR_ALLOW_OVERDRIVE      false
#define MOTOR_HARD_DUTY_FRACTION   0.50f
#define MOTOR_SAFE_FALLBACK_MAX    64

#define FWD_PWM         50
#define FWD_VOLTS       3.1f

// TURN_VOLTS define el TECHO de PWM del giro por gyro (turnPwm, pasado como
// maxPwm a _turnGyro). ANTES estaba en 3.1f (igual que FWD_VOLTS) — con la
// mayoría de baterías eso da un turnPwm tan bajo que pwmForVolts() lo
// termina forzando hasta MOTOR_KICK_MIN (su propio piso interno). Cuando el
// techo (turnPwm) y el piso (antes MOTOR_KICK_MIN, ver TURN_KICK_MIN abajo)
// quedan iguales, _turnGyro deja de ser "proporcional": el motor gira
// SIEMPRE al mismo PWM fijo sin importar qué tan cerca esté del objetivo,
// frena de golpe al cruzar la tolerancia, se pasa por inercia, y rebota
// (ping-pong) tratando de corregir — el techo y el piso deben quedar
// separados por un margen real para que exista una zona donde el término
// proporcional (kp*errYaw) pueda ir bajando la potencia según se acerca.
//   - Sube TURN_VOLTS si con tu batería actual turnPwm (revisa por Serial
//     o PAGE_MOTOR) sigue saliendo igual a TURN_KICK_MIN/MOTOR_KICK_MIN.
//   - No lo subas tanto que el giro entre demasiado rápido y no le dé
//     tiempo al lazo de control (GYRO_CTRL_INTERVAL_MS) a reaccionar.
#define TURN_VOLTS      4.2f

// Piso de PWM PROPIO para el giro por gyro (antes _turnGyro usaba
// MOTOR_KICK_MIN, pensado para el ARRANQUE del avance recto desde el
// reposo — no para desacelerar un giro que ya viene con inercia angular).
// Debe quedar CLARAMENTE por debajo de turnPwm (ver TURN_VOLTS arriba) para
// dejarle espacio real al control proporcional cerca del objetivo.
//   - Si el giro tiembla/se queda pegado sin llegar a cerrar los últimos
//     grados (se frena antes de tiempo), SUBE TURN_KICK_MIN.
//   - Si el giro sigue rebotando (overshoot) al llegar, BAJA TURN_KICK_MIN
//     (le da más margen a la desaceleración proporcional).
#define TURN_KICK_MIN   28

// ── GIROS INDEPENDIENTES IZQUIERDA / DERECHA (CON ENCODERS) ───
#define TURN_VOLTS_L    TURN_VOLTS
#define TURN_VOLTS_R    TURN_VOLTS

// ENC_TURN90_CALC es el cálculo geométrico "de libro" de cuántos ticks
// corresponden a un giro de 90°, según TRACK_WIDTH_MM. En la práctica
// casi nunca da exacto (deslizamiento de llantas, fricción distinta
// entre lados, el propio overshoot por inercia al frenar), así que
// cada lado tiene su AJUSTE en ticks para afinarlo por separado. Esto
// SOLO afecta el giro por encoder (fallback sin gyro, _turnEncoder) —
// con gyro activo (NAV_MODE 2 o 3) el que manda es GYRO_TURN90_DEG_L/R,
// más abajo.
//   - Si el giro IZQUIERDO se queda CORTO (no llega a 90°) → BAJA
//     ENC_TURN90_TRIM_L (menos ticks restados = gira más).
//   - Si el giro IZQUIERDO se PASA (más de 90°) → SUBE ENC_TURN90_TRIM_L.
//   - Mismo criterio para ENC_TURN90_TRIM_R con el giro DERECHO.
// Calíbralos con NAV_MODE 1 (solo encoders) para no mezclar con el
// comportamiento del gyro mientras ajustas.
#define ENC_TURN90_TRIM_L   76
#define ENC_TURN90_TRIM_R   73

#define ENC_TURN90_L    (ENC_TURN90_CALC - ENC_TURN90_TRIM_L)
#define ENC_TURN90_R    (ENC_TURN90_CALC - ENC_TURN90_TRIM_R)

// TURN_TRIM_L/R: multiplican el PWM de la rueda "de apoyo" (la que NO
// marca la distancia del giro) durante un giro por encoder (fallback),
// para compensar que un motor gire más rápido que el otro al mismo PWM
// y el giro salga "curveado" en vez de un pivote limpio en el sitio:
//   - TURN_TRIM_L actúa en ff_turnLeft90 (fallback): mueve la rueda
//     DERECHA hacia adelante mientras la izquierda va en reversa. Si
//     el giro izquierdo se siente "curveado"/corto, SUBE TURN_TRIM_L;
//     si se pasa de más, BÁJALO.
//   - TURN_TRIM_R actúa en ff_turnRight90 (fallback): mueve la rueda
//     IZQUIERDA hacia atrás. Mismo criterio, ajustando TURN_TRIM_R.
#define TURN_TRIM_L     1.00f
#define TURN_TRIM_R     1.00f

#define ENC_SIGN_L   (+1)
#define ENC_SIGN_R   (-1)

// ENC_TRIM_R: multiplica el PWM base (y los ticks esperados) de la
// rueda DERECHA para igualarla a la izquierda durante el avance recto
// clásico (_driveEncoder, cuando NO se usa gyro). Es la variable más
// "directa" para el desvío en línea recta: si el robot avanza torcido
// con el mismo PWM en ambas ruedas, es porque un motor entrega más
// fuerza que el otro a igual señal, y esto lo compensa.
//   - Si el robot se desvía hacia la DERECHA en línea recta → SUBE
//     ENC_TRIM_R poco a poco (1.00 -> 1.03 -> 1.06 ...) para darle
//     más fuerza a la rueda derecha.
//   - Si se desvía hacia la IZQUIERDA → BAJA ENC_TRIM_R (1.00 -> 0.97
//     -> 0.94 ...) para quitarle fuerza a la derecha.
// Calibra esto PRIMERO (con NAV_MODE 1, solo encoders, sin gyro de por
// medio) antes de tocar KP_ENC/KI_ENC: si el robot ya viene torcido de
// base, el control PI tiene que estar corrigiendo todo el tiempo para
// compensarlo, y esa lucha constante es una causa típica de oscilación.
#define ENC_TRIM_R   1.00f

// ── CONTROL AVANZADO BNO + IR (AVANCE) ───────────────────────
// Ganancia P para mantener el rumbo (yaw) durante el avance recto con
// gyro. Si el robot zigzaguea/serpentea SOLO cuando usa gyro (NeoPixel
// naranja/púrpura — ver sensors.h) para avanzar, BAJA KP_GYRO_FWD. Si
// se desvía de forma lenta y no corrige lo suficientemente rápido,
// SÚBELA. El resultado final está limitado por ENC_CORR_MAX_FRAC
// (arriba), así que si ya subiste eso y sigue sin corregir fuerte,
// puede que el tope de ENC_CORR_MAX_FRAC sea lo que lo está frenando.
#define KP_GYRO_FWD   10.0f
// Ganancia I para el mismo control de rumbo. Un P puro (como era antes)
// NUNCA cancela del todo una asimetría física constante entre motores
// (la misma que corriges con ENC_TRIM_R en modo encoder) — en equilibrio
// queda un error de yaw pequeño pero CONSTANTE, que es justo lo que se
// ve como "se desvía hacia un lado sostenidamente" aunque el control ya
// esté "corrigiendo". La integral acumula ese error residual y termina
// de cancelarlo.
//
// OJO CON LA ESCALA: esto NO es como KP_GYRO_FWD (que anda en 12-13).
// La integral se ACUMULA cada 10ms (100 veces por segundo), así que un
// valor grande se satura casi instantáneo contra KI_GYRO_FWD_MAX y se
// queda empujando a fondo hacia un solo lado de forma constante — se ve
// como que el robot queda "pegado" contra una pared sin corregir, y
// hacia qué lado satura primero depende de ruido mínimo del sensor al
// arrancar, así que además sale distinto en cada corrida (a veces
// derecha, a veces izquierda). Si ves ESE síntoma (pegado a una pared,
// sin patrón repetible), es casi seguro que KI_GYRO_FWD está demasiado
// alto — bájalo, no lo subas.
//   - Si el robot SIGUE desviándose siempre hacia el MISMO lado en
//     avances largos (no zigzaguea, se va derecho pero curvo) → SUBE
//     KI_GYRO_FWD de a poco: 0.0 -> 0.02 -> 0.04 -> 0.06 ... (pasos
//     chicos, NO como KP_GYRO_FWD).
//   - Si empieza a oscilar de forma lenta y rítmica (tarda 1-2s en
//     notarse, distinto al temblor inmediato de KP alto), o si queda
//     empujando fijo hacia un lado sin corregir → te pasaste, BAJA
//     KI_GYRO_FWD.
#define KI_GYRO_FWD       0.03f
#define KI_GYRO_FWD_MAX   2.5f   // antiwindup, igual criterio que KI_ENC_MAX
// Ganancia P para centrado lateral usando los IR laterales (mantener
// al robot a la misma distancia de ambas paredes). 0.0f = desactivado.
// Si el robot rebota entre las paredes (toca una, se aleja, toca la
// otra), BAJA KP_CENTER. Si se arrastra muy pegado a una pared sin
// corregir hacia el centro, SÚBELA.
#define KP_CENTER     0.01f
#define IR_BRAKE_THR  80      // Umbral IR frontal para empezar a frenar gradualmente
#define PWM_MIN_BRAKE 45       // PWM mínimo absoluto al frenar

// ── GIROS INDEPENDIENTES IZQUIERDA / DERECHA (POR GYRO) ──────
// Grados OBJETIVO reales para cada giro cuando se usa el BNO085
// (NAV_MODE 2 o 3). En teoría deberían ser 90.0 en ambos, pero si el
// robot sistemáticamente se pasa o se queda corto en un giro
// PARTICULAR (offset del montaje del sensor, forma de frenar, etc.),
// ajusta el número directamente — es el equivalente, para el gyro, de
// ENC_TURN90_TRIM_L/R de arriba:
//   - Gira DE MENOS (se queda corto de 90°) → SUBE el valor (ej. 92.0).
//   - Gira DE MÁS (se pasa de 90°) → BAJA el valor (ej. 87.0).
// Ajusta L y R por separado — no tienen que terminar en el mismo número.
#define GYRO_TURN90_DEG_L   90.0f
#define GYRO_TURN90_DEG_R   90.0f
// Qué tan cerca del objetivo (en grados) se considera "ya llegué" y
// corta el giro. Bajarlo da más precisión pero puede hacer que el
// giro tiemble/oscile buscando el punto exacto; subirlo lo hace más
// rápido de dar por terminado, a costa de precisión.
#define GYRO_TURN_TOLERANCE_DEG   3.0f
// Ganancia proporcional del giro por gyro (qué tan fuerte empuja el
// motor según el error de yaw restante). Si el giro se PASA de 90° y
// "rebota" corrigiendo hacia atrás (overshoot con oscilación al final
// del giro), BAJA la KP de ese lado. Si el giro llega lento o se
// queda pegado sin terminar de cerrar los últimos grados, SÚBELA.
// Nota: ya vienen distintos entre sí (1.0 vs 1.5) — probablemente
// porque un lado ya mostraba overshoot y el otro no; sigue ajustando
// cada uno por separado según lo que veas en TU robot.
#define GYRO_TURN_KP_L   55.0f
#define GYRO_TURN_KP_R   55.0f

// Cada cuánto pide el firmware un reporte nuevo de orientación al BNO085.
// Bajar este valor (ej. 5000) pide datos más seguido; el chip puede o no
// alcanzar a entregarlos a esa tasa. Es la fuente del dato de yaw.
#define BNO_REPORT_INTERVAL_US   10000UL

// Cada cuánto el LAZO DE CONTROL de giro (_turnGyro en motion.h) relee el
// yaw y recalcula el PWM de los motores. Debe ser >= a lo que tarda el BNO
// en entregar un dato nuevo (ver BNO_REPORT_INTERVAL_US) para no actuar
// sobre un valor repetido, pero si se pone muy alto el giro se vuelve
// menos reactivo (puede pasarse del objetivo). Punto de partida para
// experimentar si el giro se pasa de 90° o se queda corto.
#define GYRO_CTRL_INTERVAL_MS   10

// ── VIGILANCIA DEL BNO085 (detectar que quedó "zombie") ──────
// "Zombie" = g_gyroReady queda en true (el init respondió OK) pero el
// sensor deja de entregar datos nuevos, por ejemplo tras un reinicio en
// caliente. Si no llega NINGÚN evento nuevo en más de este tiempo:
//  - gyroIsAlive() empieza a devolver false → floodfill usa encoders.
//  - _turnGyro() corta el giro en vez de seguir con un yaw congelado.
//  - gyroWatchdog() marca g_gyroReady=false y reintenta un reset físico.
#define GYRO_ALIVE_TIMEOUT_MS     200

// Umbral de "vivo" APARTE, solo para la corrección de rumbo durante el
// AVANCE recto (_navUseGyroHeading en motion.h). Los giros siguen usando
// GYRO_ALIVE_TIMEOUT_MS de arriba sin cambios.
//
// Por qué existe por separado: durante el avance los motores están a
// full de forma continua (ruido eléctrico/magnético constante cerca del
// BNO), a diferencia del instante quieto antes de un giro. Si el LED del
// BNO se "congela" específicamente durante el avance (y no en giros ni
// en reposo), es señal de que 200ms es demasiado ajustado para ese
// contexto y el firmware se está cayendo al fallback de encoder a media
// marcha sin aviso.
//
// Cómo calibrar SIN monitor serial: sube el firmware, corre varios
// avances rectos y mira si el robot serpentea/pierde rumbo (indicaría
// que sigue cayendo al fallback) o si vuelve a corregir bien con el BNO.
// Sube este valor en pasos de 50-100ms si sospechas que sigue cayendo al
// fallback; bájalo hacia GYRO_ALIVE_TIMEOUT_MS si quieres detectar un
// BNO realmente muerto más rápido durante el avance.
//
// IMPORTANTE: este número no tiene efecto por sí solo si es mayor que
// GYRO_STALE_TIMEOUT_MS (ver más abajo) — ese otro valor apaga
// g_gyroReady antes de que este llegue a importar. Si subes este,
// sube también GYRO_STALE_TIMEOUT_MS a un número igual o mayor.
#define GYRO_ALIVE_TIMEOUT_DRIVE_MS   150

// Cada cuánto reintenta gyroWatchdog() un reset+reinit completo del BNO
// mientras siga sin responder (ya sea recién detectado "zombie" o que
// nunca arrancó bien desde el inicio).
#define GYRO_RECOVER_COOLDOWN_MS  2000

// Duración de cada parpadeo "latido" de los 3 LEDs debug cada vez que
// llega un dato nuevo del BNO085. Solo aplica cuando el robot NO está
// haciendo una acción normal (ver g_actionInProgress en motion.h) —
// durante una acción, los LEDs se quedan estáticos con su uso normal.
#define BNO_HEARTBEAT_MS          15

// Cuánto espera gyroInit() a que llegue la PRIMERA muestra real de yaw
// antes de declarar el BNO listo. begin_I2C()/enableReport() a veces
// reportan éxito (sobre todo tras un reinicio en caliente del ESP32 sin
// power-cycle del BNO) sin que el chip llegue a entregar ningún dato —
// sin esta espera, g_gyroReady quedaría en true de forma falsa.
#define GYRO_INIT_WAIT_MS       500

// Si durante el uso (no en el init) dejan de llegar muestras nuevas de
// yaw por más de este tiempo, se asume que el BNO se cayó a media
// operación y se marca g_gyroReady=false — así los giros vuelven a usar
// el respaldo por encoder en vez de girar a ciegas con un yaw congelado.
//
// DEBE ser >= GYRO_ALIVE_TIMEOUT_DRIVE_MS (arriba), o g_gyroReady se
// apaga antes de que ese margen llegue a importar.
#define GYRO_STALE_TIMEOUT_MS   150

// ── ARRANQUE SUAVE DEL AVANCE (evita "serpenteo" justo al arrancar) ──
// Después de un giro las dos ruedas venían girando en sentidos
// OPUESTOS (una adelante, otra atrás) y se frenan casi de golpe. Si el
// avance recto arranca a PWM completo desde el primer ciclo, es común
// que un motor "muerda"/tome tracción un instante antes que el otro
// (juego del engranaje al invertir sentido, fricción distinta entre
// ambos, etc.) — el PI ve ese desbalance inicial y corrige fuerte, lo
// que se ve como un serpenteo justo al arrancar que luego se endereza
// solo. Esto es DISTINTO al desvío constante de ENC_TRIM_R: si el
// ángulo del giro ya te da casi exacto y el problema es solo al
// arrancar a avanzar, es esto lo que hay que tocar, no ENC_TRIM_R.
//
//   - FWD_RAMP_TICKS: durante cuántos ticks de encoder el PWM sube
//     gradual desde FWD_RAMP_START_PWM hasta el PWM objetivo, en vez
//     de saltar de golpe. SUBE este valor si sigue serpenteando al
//     arrancar (dale más "rampa"); BÁJALO (o ponlo en 0 para
//     desactivar) si el avance se siente perezoso/lento para arrancar.
//   - FWD_RAMP_START_PWM: PWM inicial de la rampa. No debe quedar
//     abajo de MOTOR_KICK_MIN o el motor ni se mueve.
#define FWD_RAMP_START_PWM   (MOTOR_KICK_MIN + 10)
#define FWD_RAMP_TICKS       250

// ── CONTROL DIFERENCIAL (avance recto por encoder, PI) ────────
// Antes de tocar KP_ENC/KI_ENC, calibra ENC_TRIM_R (arriba) con
// NAV_MODE 1. Si el robot ya va derecho de base, este PI solo tiene
// que corregir ruido/deslizamientos pequeños y es mucho más fácil que
// no oscile.
//
// CÓMO CALIBRAR (en orden):
//  1. Pon KI_ENC en 0.0f temporalmente. Sube KP_ENC de a poco (ej.
//     0.15 -> 0.20 -> 0.25...) hasta que el robot enderece rápido un
//     empujón lateral SIN empezar a zigzaguear (fishtail) de un lado
//     a otro. Si ya zigzaguea, baja KP_ENC un escalón y déjalo ahí.
//  2. Vuelve a subir KI_ENC de a poco desde 0.0f. Sirve para quitar el
//     error residual que KP solo no corrige (ej. un desvío lento y
//     constante), pero es la ganancia MÁS propensa a causar oscilación
//     sostenida si se pasa (el error se "acumula" y sobre-corrige con
//     retraso). Si el robot empieza a serpentear de forma rítmica y
//     constante (no solo al arrancar), BAJA KI_ENC — normalmente basta
//     con un valor pequeño (0.02–0.08).
#define KP_ENC      0.35f // ++ = corrige más fuerte y rápido un desvío repentino; -- = corrige más suave (menos brusco, pero más lento)
#define KI_ENC      0.12f // ++ = elimina más rápido un desvío lento/constante; -- = más estable, pero puede dejar un desvío residual pequeño
#define KI_ENC_MAX  5.0f  // Antiwindup: tope del acumulado de KI_ENC, evita que un error grande deje "cargado" el integrador y luego sobre-corrija de golpe

// Qué tan fuerte puede la corrección (del PI de arriba, o de
// KP_GYRO_FWD) "tironear" una rueda respecto a la otra durante el
// avance recto, como fracción del PWM base actual. El límite es
// SIMÉTRICO a propósito: una corrección con más fuerza hacia un lado
// que hacia el otro es una causa directa de oscilación asimétrica
// (corrige fuerte para un lado, débil para el otro, nunca queda
// centrado). Es la misma idea que ENC_TRIM_R pero para la RESPUESTA
// del control, no para el punto de partida.
//   - Subir ENC_CORR_MAX_FRAC (ej. 0.40 -> 0.55) = correcciones más
//     agresivas, reacciona más rápido a un desvío grande, pero si se
//     pasa empieza a zigzaguear.
//   - Bajar ENC_CORR_MAX_FRAC (ej. 0.40 -> 0.25) = correcciones más
//     suaves y estables, pero tarda más en enderezar un desvío grande.
#define ENC_CORR_MAX_FRAC  0.40f

// ── SENSORES IR ─────────────────────────────────────────────
#define IR_TARGET_L   235
#define IR_TARGET_R   265
#define IR_TARGET_C   265

#define IR_WALL_THR_L  70    
#define IR_WALL_THR_R  80    
#define IR_WALL_THR_FL 80    
#define IR_WALL_THR_FR 80    

#define IR_OPEN_THR   20  
#define IR_CLOSE_THR  600

#define IR_USE_ABS_DIFF   true

// ── LEDs DEBUG ───────────────────────────────────────────────
#define LED_BLINK_MS   150

// =============================================================
//  TABLA DE DEPENDENCIAS — qué revisar si cambias algo
// =============================================================
//  Si cambias...                  También revisa / recalibra...
// -----------------------------------------------------------
//  KP_GYRO_FWD / KP_CENTER        Afectan el AVANCE RECTO (con BNO e IR activos).
//                                 Si zigzaguea corrigiendo rumbo, baja KP_GYRO_FWD.
//                                 Si rebota contra los laterales, baja KP_CENTER.
//
//  IR_BRAKE_THR / PWM_MIN_BRAKE   Determinan cuándo y cuánto frena el robot
//                                 al ver una pared frontal en el floodfill.
//                                 (Solo se aplican en ff_moveForwardOneCell).
//
//  WHEEL_DIAM_MM                  TICKS_PER_MM se recalcula solo, pero
//                                 ENC_TURN90_CALC, ENC_TRIM_R y los
//                                 TURN_TRIM_L/R quedan desactualizados.
//
//  TRACK_WIDTH_MM                 ENC_TURN90_CALC se recalcula solo, reconfirmar.
//
//  CELL_MM                        CELL_FWD_MM y ENC_CELL_FWD se recalculan solos.
//
//  PRESHIFT_MM                    Cambia cuánto avanza el robot antes de cada giro.
//
//  ENC_TRIM_R                     Afecta el AVANCE RECTO clásico (cuando BNO no se usa).
//
//  KP_ENC / KI_ENC                Afectan AVANCE RECTO clásico (_driveEncoder base).
//
//  GYRO_ALIVE_TIMEOUT_DRIVE_MS    Solo afecta el AVANCE (_navUseGyroHeading).
//                                 Si el robot serpentea/pierde rumbo en avances
//                                 largos con NAV_MODE==3, subir este valor.
//                                 No afecta los giros (esos usan GYRO_ALIVE_TIMEOUT_MS).
//
//  STOP_OVERSHOOT_MM              Afecta CELL_FWD_MM, ENC_PRESHIFT y
//                                 ENC_INITIAL_ADVANCE (todo avance por encoder
//                                 se acorta por igual). Si "se pasa" al llegar
//                                 a un punto, subir; si se queda corto, bajar.
//
//  ENC_TRIM_R                     Ver arriba (avance recto clásico, sin gyro).
//                                 Calibrar ANTES que KP_ENC/KI_ENC.
//
//  ENC_CORR_MAX_FRAC              Límite de la corrección en avance recto
//                                 (encoder o gyro). Simétrico a propósito —
//                                 ver comentario arriba si el robot corrige
//                                 más fuerte hacia un lado que el otro.
//
//  ENC_TURN90_TRIM_L/R            Afecta SOLO el giro por encoder (fallback).
//                                 Ver comentario arriba para saber si subir o
//                                 bajar según si el giro se pasa o se queda corto.
//
//  TURN_TRIM_L/R                  Afecta SOLO el giro por encoder (fallback).
//                                 Corrige que el giro salga "curveado".
//
//  GYRO_TURN90_DEG_L/R            Afecta SOLO el giro por gyro (NAV_MODE 2/3).
//                                 Equivalente a ENC_TURN90_TRIM_L/R pero para gyro.
//
//  ORDEN SUGERIDO DE CALIBRACIÓN:
//   1. NAV_MODE=1 (solo encoders). Calibrar ENC_TRIM_R (avance recto).
//   2. Con NAV_MODE=1, calibrar ENC_TURN90_TRIM_L/R y TURN_TRIM_L/R (giros).
//   3. Con ENC_TRIM_R ya razonable, afinar KP_ENC, luego KI_ENC, luego
//      ENC_CORR_MAX_FRAC si sigue oscilando o corrigiendo muy débil/fuerte.
//   4. Calibrar STOP_OVERSHOOT_MM (se nota más en preshift/avance inicial).
//   5. NAV_MODE=2 (solo gyro). Calibrar GYRO_TURN90_DEG_L/R, GYRO_TURN_KP_L/R,
//      KP_GYRO_FWD.
//   6. Activar KP_CENTER (IR) si hay paredes en ambos lados.
//   7. NAV_MODE=3 (competencia): ambos con fallback automático.
// =============================================================