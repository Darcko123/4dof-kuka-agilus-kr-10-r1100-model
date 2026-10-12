#!/usr/bin/env python3
"""
Simulador del brazo 4DOF tipo KUKA Agilus KR10 R1100.

La cinemática NO se reimplementa: se compila el robotics.c del firmware como
librería compartida y se llama vía ctypes, de modo que lo que se ve aquí es
exactamente lo que calcula el STM32. Los servos se simulan replicando la
lógica de PCA9685_SetSmoothAngle()/PCA9685_UpdateSmoothServo() y la
conversión ángulo -> PWM de PCA9685_SetServoAngle().

Uso:
    .venv/bin/python sim_kuka.py                      # GUI interactiva
    .venv/bin/python sim_kuka.py --q 0 30 -45 10      # FK en consola (grados)
    .venv/bin/python sim_kuka.py --gripper ...        # incluye la pinza (a4 = L4 + LGRIP)
    KUKA_FW_DIR=/ruta/al/proyecto .venv/bin/python sim_kuka.py
"""

import argparse
import ctypes as C
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

FW_DIR = Path(os.environ.get(
    "KUKA_FW_DIR",
    Path.home() / "Documentos/GitHub/4dof-kuka-agilus-kr-10-r1100-model/4DOF_KUKA_AGILUS_KR10"))
BUILD_DIR = Path(__file__).resolve().parent / "build"

NJ = 4
UPDATE_INTERVAL_MS = 20     # Intervalo de Servo_Smooth_t (50 Hz, igual que el PWM)
MOVE_DURATION_MS = 1500     # Duración de cada movimiento (SERVO_MOVE_DURATION en main.c)
HOLD_MS = 1000              # Pausa entre poses (DELAY_TEST_SEQUENCE en main.c es 5000)
SERVO_CHANNELS = (0, 1, 2, 3)
SERVO_ZERO_DEG = 0.0        # Ángulo del servo para q = 0 (horizontal), igual que main.c

# Secuencia demo en ángulos articulares q [grados] (base, hombro, codo, muñeca).
# Rango 0°-180° con q = 0 horizontal; q2 > 0 levanta el brazo (q2 = 90° vertical).
DEMO_SEQUENCE = [
    (0, 0, 0, 0),
    (45, 45, 60, 45),
    (135, 40, 60, 30),
    (135, 40, 60, 30),      # Pose repetida: delta ~0, el servo se mueve directo (sin stepSize == 0)
    (90, 70, 80, 30),
    (90, 90, 0, 0),
    (0, 0, 0, 0),
]

STATUS = {0: "OK", 1: "ERROR", 2: "NOT_INITIALIZED", 3: "INVALID_PARAM", 4: "OUT_OF_RANGE"}

# ============================================================================
# LIBRERÍA robotics.c VÍA CTYPES
# ============================================================================


class DHParam(C.Structure):
    _fields_ = [(n, C.c_float) for n in
                ("a", "alpha", "d", "offset", "direction", "minAngle", "maxAngle")]


class Transform(C.Structure):
    _fields_ = [("R", (C.c_float * 3) * 3), ("p", C.c_float * 3)]


class Pose(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("x", "y", "z", "pitch", "yaw")]


QArray = C.c_float * NJ


def build_library():
    """Compila robotics.c como .so si no existe o si el fuente es más nuevo."""
    src = FW_DIR / "Core/Src/robotics.c"
    inc = FW_DIR / "Core/Inc"
    out = BUILD_DIR / "librobotics.so"
    if not src.exists():
        sys.exit(f"No se encontró {src}. Define KUKA_FW_DIR con la ruta del proyecto STM32.")
    if not out.exists() or out.stat().st_mtime < max(src.stat().st_mtime, (inc / "robotics.h").stat().st_mtime):
        BUILD_DIR.mkdir(exist_ok=True)
        subprocess.run(["gcc", "-O2", "-Wall", "-shared", "-fPIC", f"-I{inc}",
                        "-o", str(out), str(src), "-lm"], check=True)
    return out


class Robot:
    """Envoltura sobre robotics.c."""

    def __init__(self, gripper=False):
        self.lib = C.CDLL(str(build_library()))
        lib = self.lib
        lib.Robotics_Init.argtypes = [C.POINTER(DHParam)]
        lib.Robotics_DH_Transform.argtypes = [C.c_float] * 4 + [C.POINTER(Transform)]
        lib.Robotics_TransformMultiply.argtypes = [C.POINTER(Transform)] * 3
        lib.Robotics_CheckJointLimits.argtypes = [QArray]
        lib.Robotics_ForwardKinematics.argtypes = [QArray, C.POINTER(Transform)]
        lib.Robotics_GetPose.argtypes = [C.POINTER(Transform), C.POINTER(Pose)]

        default = (DHParam * NJ).in_dll(lib, "Robotics_DefaultDHTable")
        self.dh = (DHParam * NJ)(*default)                  # Copia modificable
        self.set_gripper(gripper)

    def set_gripper(self, enabled, lgrip=45.0):             # ROBOTICS_LGRIP
        default = (DHParam * NJ).in_dll(self.lib, "Robotics_DefaultDHTable")
        self.dh[3].a = default[3].a + (lgrip if enabled else 0.0)
        status = self.lib.Robotics_Init(self.dh)
        assert status == 0, STATUS[status]

    def limits_deg(self):
        return [(math.degrees(p.minAngle), math.degrees(p.maxAngle)) for p in self.dh]

    def solve(self, q_deg):
        """Devuelve (puntos de la cadena, T final, pose, estado de límites)."""
        q = QArray(*(math.radians(v) for v in q_deg))
        status = self.lib.Robotics_CheckJointLimits(q)

        # Cadena eslabón a eslabón (para dibujar), con las funciones públicas de la librería
        T = Transform()
        for i in range(3):
            T.R[i][i] = 1.0
        A = Transform()
        points = [(0.0, 0.0, 0.0)]
        for i, p in enumerate(self.dh):
            theta = p.direction * q[i] + p.offset
            self.lib.Robotics_DH_Transform(p.a, p.alpha, p.d, theta, C.byref(A))
            self.lib.Robotics_TransformMultiply(C.byref(T), C.byref(A), C.byref(T))
            points.append(tuple(T.p))

        # FK optimizada (la que corre en el STM32); debe coincidir con la cadena
        Tfk = Transform()
        self.lib.Robotics_ForwardKinematics(q, C.byref(Tfk))
        if not np.allclose(points[-1], tuple(Tfk.p), atol=1e-3):
            print(f"[AVISO] FK optimizada {tuple(Tfk.p)} != cadena {points[-1]}", file=sys.stderr)

        pose = Pose()
        self.lib.Robotics_GetPose(C.byref(Tfk), C.byref(pose))
        R = np.array([[Tfk.R[i][j] for j in range(3)] for i in range(3)])
        return np.array(points), R, pose, STATUS[status]


# ============================================================================
# MODELO DE SERVOS (réplica de PCA9685.c)
# ============================================================================


def q_to_servo(q_deg):
    """Ángulo articular [0, 180] -> ángulo de servo [0, 180] (q = 0 horizontal)."""
    return q_deg + SERVO_ZERO_DEG


def servo_to_pwm(angle):
    """Réplica de PCA9685_SetServoAngle(): ticks de 12 bits y ancho de pulso a 50 Hz."""
    angle = min(max(angle, 0.0), 180.0)
    ticks = int(np.float32(angle * (511.9 - 102.4) / 180.0 + 102.4))
    return ticks, ticks * 20000.0 / 4096.0


class SmoothServo:
    """Réplica de Servo_Smooth_t + PCA9685_SetSmoothAngle()/UpdateSmoothServo()."""

    def __init__(self, channel, angle, interval_ms=UPDATE_INTERVAL_MS):
        self.channel = channel
        self.current = np.float32(angle)
        self.target = np.float32(angle)
        self.step = np.float32(0.0)
        self.interval = interval_ms
        self.moving = False

    def set_smooth_angle(self, target, duration_ms):
        target = np.float32(min(max(target, 0.0), 180.0))
        self.target = target
        steps = duration_ms // self.interval
        delta = np.float32(target - self.current)
        if steps > 0 and abs(delta) > 0.01:                 # Igual que el firmware: delta ~0 mueve directo
            self.step = np.float32(delta / np.float32(steps))
            self.moving = True
        else:
            self.current = target
            self.moving = False

    def update(self):
        """Se llama cada `interval` ms. True si terminó."""
        if not self.moving:
            return True
        new = np.float32(self.current + self.step)
        if (self.step > 0 and new >= self.target) or (self.step < 0 and new <= self.target):
            new = self.target
            self.moving = False
        self.current = new
        return not self.moving


# ============================================================================
# MODO CONSOLA
# ============================================================================


def print_report(robot, q_deg):
    pts, R, pose, status = robot.solve(q_deg)
    print(f"q [deg]       = {list(q_deg)}   límites: {status}")
    for i, p in enumerate(pts):
        print(f"  marco {i}: x={p[0]:8.2f}  y={p[1]:8.2f}  z={p[2]:8.2f}  mm")
    print(f"Pose efector  : x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} mm  "
          f"pitch={math.degrees(pose.pitch):.2f}°  yaw={math.degrees(pose.yaw):.2f}°")
    print("R =\n" + np.array2string(R, precision=4, suppress_small=True))
    for ch, q in zip(SERVO_CHANNELS, q_deg):
        s = q_to_servo(q)
        ticks, us = servo_to_pwm(s)
        print(f"  servo ch{ch}: {s:6.1f}°  -> PWM off={ticks:3d}  ({us:6.1f} µs)")
    if pts[:, 2].min() < 0:
        print("¡ATENCIÓN! Algún punto de la cadena queda bajo el suelo (z < 0).")


# ============================================================================
# GUI
# ============================================================================


def run_gui(robot):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, CheckButtons, Slider

    names = ("q1 base", "q2 hombro", "q3 codo", "q4 muñeca")
    servos = [SmoothServo(ch, SERVO_ZERO_DEG) for ch in SERVO_CHANNELS]

    fig = plt.figure("KUKA Agilus KR10 4DOF — simulador", figsize=(13, 8))
    ax = fig.add_axes([0.0, 0.22, 0.62, 0.76], projection="3d")
    info = fig.text(0.63, 0.95, "", family="monospace", fontsize=9, va="top")

    reach = sum(p.a for p in robot.dh[1:]) + 45.0
    ztop = robot.dh[0].d

    # Suelo y pedestal fijos
    g = np.linspace(-reach, reach, 2)
    gx, gy = np.meshgrid(g, g)
    ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.08, color="gray")
    th = np.linspace(0, 2 * np.pi, 40)
    for z in (0.0, 35.0):                                   # ROBOTICS_H0
        ax.plot(20 * np.cos(th), 20 * np.sin(th), z, color="0.5", lw=1)

    (links,) = ax.plot([], [], [], "-", lw=6, color="#f28c00", solid_capstyle="round")
    (joints,) = ax.plot([], [], [], "o", ms=9, color="#333333")
    (trace,) = ax.plot([], [], [], "-", lw=1, color="#1f77b4", alpha=0.7)
    frame_lines = [ax.plot([], [], [], lw=2, color=c)[0] for c in ("r", "g", "b")]
    trail = []

    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, ztop + reach)
    ax.set_box_aspect((2 * reach, 2 * reach, ztop + reach))
    ax.set_xlabel("X0 [mm]")
    ax.set_ylabel("Y0 [mm]")
    ax.set_zlabel("Z0 [mm]")

    sliders = []
    for i, ((lo, hi), name) in enumerate(zip(robot.limits_deg(), names)):
        sax = fig.add_axes([0.08, 0.16 - i * 0.04, 0.5, 0.025])
        sliders.append(Slider(sax, name, lo, hi, valinit=0.0, valfmt="%6.1f°"))

    state = {"demo": None, "syncing": False, "warn": ""}

    def draw(q_deg):
        pts, R, pose, status = robot.solve(q_deg)
        links.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
        joints.set_data_3d(pts[:-1, 0], pts[:-1, 1], pts[:-1, 2])
        ee = pts[-1]
        L = 20.0
        for k, line in enumerate(frame_lines):
            tip = ee + L * R[:, k]
            line.set_data_3d([ee[0], tip[0]], [ee[1], tip[1]], [ee[2], tip[2]])
        if state["demo"] is not None:
            trail.append(ee)
            t = np.array(trail)
            trace.set_data_3d(t[:, 0], t[:, 1], t[:, 2])

        lines = ["ARTICULACIONES / SERVOS",
                 " joint   q[°]   servo[°]  PWM   pulso"]
        for i, q in enumerate(q_deg):
            s = q_to_servo(q)
            ticks, us = servo_to_pwm(s)
            lines.append(f"  {i + 1}   {q:7.1f}   {s:6.1f}   {ticks:4d}  {us:6.0f}µs")
        lines += ["", f"Límites: {status}", "",
                  "EFECTOR FINAL (Robotics_GetPose)",
                  f"  x = {pose.x:8.2f} mm",
                  f"  y = {pose.y:8.2f} mm",
                  f"  z = {pose.z:8.2f} mm",
                  f"  pitch = {math.degrees(pose.pitch):7.2f}°",
                  f"  yaw   = {math.degrees(pose.yaw):7.2f}°",
                  "", "ORIGEN DE CADA MARCO [mm]"]
        for i, p in enumerate(pts):
            lines.append(f"  {i}: ({p[0]:7.1f}, {p[1]:7.1f}, {p[2]:7.1f})")
        if pts[:, 2].min() < 0:
            lines += ["", "¡COLISIÓN CON EL SUELO (z < 0)!"]
        if state["warn"]:
            lines += ["", state["warn"]]
        info.set_text("\n".join(lines))
        info.set_color("red" if pts[:, 2].min() < 0 else "black")
        fig.canvas.draw_idle()

    def on_slider(_):
        if not state["syncing"]:
            draw([s.val for s in sliders])

    for s in sliders:
        s.on_changed(on_slider)

    def set_sliders(q_deg):
        state["syncing"] = True
        for s, v in zip(sliders, q_deg):
            s.set_val(v)
        state["syncing"] = False
        draw(q_deg)

    # ---- Secuencia demo: un tick del timer = un updateInterval del firmware ----
    timer = fig.canvas.new_timer(interval=UPDATE_INTERVAL_MS)

    def start_segment(d):
        target = DEMO_SEQUENCE[d["idx"]]
        for sv, q in zip(servos, target):
            sv.set_smooth_angle(q_to_servo(q), MOVE_DURATION_MS)
        d["elapsed"] = 0
        d["hold"] = 0

    def tick():
        d = state["demo"]
        if d is None:
            return
        if d["hold"] > 0:
            d["hold"] -= UPDATE_INTERVAL_MS
            if d["hold"] <= 0:
                d["idx"] += 1
                if d["idx"] >= len(DEMO_SEQUENCE):
                    state["demo"] = None
                    timer.stop()
                    return
                start_segment(d)
            return
        done = [sv.update() for sv in servos]
        d["elapsed"] += UPDATE_INTERVAL_MS
        set_sliders([sv.current - SERVO_ZERO_DEG for sv in servos])
        if all(done):
            d["hold"] = HOLD_MS
        elif d["elapsed"] > MOVE_DURATION_MS + 10 * UPDATE_INTERVAL_MS:
            stuck = [i + 1 for i, sv in enumerate(servos) if sv.moving]
            state["warn"] = (f"⚠ Pose {d['idx']}: joint(s) {stuck} con stepSize=0 nunca\n"
                             f"  terminan (UpdateSmoothServo devuelve false\n"
                             f"  para siempre). Se fuerza el avance.")
            for sv in servos:
                sv.moving = False
            d["hold"] = HOLD_MS

    timer.add_callback(tick)

    def on_demo(_):
        for sv, s in zip(servos, sliders):
            sv.__init__(sv.channel, q_to_servo(s.val))
        trail.clear()
        state["warn"] = ""
        state["demo"] = {"idx": 0}
        start_segment(state["demo"])
        timer.start()

    def on_home(_):
        state["demo"] = None
        timer.stop()
        trail.clear()
        trace.set_data_3d([], [], [])
        state["warn"] = ""
        set_sliders([0.0] * NJ)

    def on_gripper(_):
        robot.set_gripper(chk.get_status()[0])
        draw([s.val for s in sliders])

    b_demo = Button(fig.add_axes([0.66, 0.11, 0.14, 0.05]), "Secuencia demo")
    b_home = Button(fig.add_axes([0.82, 0.11, 0.14, 0.05]), "Home (q = 0)")
    chk = CheckButtons(fig.add_axes([0.66, 0.03, 0.30, 0.06]),
                       ["Incluir pinza (a4 = L4 + LGRIP)"], [robot.dh[3].a > 29.0])
    b_demo.on_clicked(on_demo)
    b_home.on_clicked(on_home)
    chk.on_clicked(on_gripper)

    draw([0.0] * NJ)
    plt.show()


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--q", nargs=NJ, type=float, metavar=("Q1", "Q2", "Q3", "Q4"),
                    help="ángulos articulares en grados; imprime la FK y sale")
    ap.add_argument("--gripper", action="store_true", help="incluir la pinza en el último eslabón")
    args = ap.parse_args()

    robot = Robot(gripper=args.gripper)
    if args.q:
        print_report(robot, args.q)
    else:
        run_gui(robot)
