# ===================================================
# Calculo de matrices de transformacion homogenea LOCALES
# para el UR10 (6 matrices entre pares de joints) y
# composicion con la pose real de un sistema de referencia
# extraída directamente de la estacion,  para obtener 
# la pose final de la brida del cobot UR10,  
# ===================================================
import numpy as np
from robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME
from robodk.robomath import Mat, pose_2_quaternion

# ---------------------------------------------------
# Conexion con el RoboDK
# ---------------------------------------------------
RDK = Robolink()

robot = RDK.ItemUserPick('Seleccione el robot UR10', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No se selecciono ningun robot valido en RoboDK')

# ---------------------------------------------------
# Sistema de referencia:"UR10 Base", para evitar
# errores de redondeo/transcripcion o de jerarquia.
# Primero se intenta por nombre exacto 'UR10 Base'; si no
# existe (nombre distinto, mayusculas, tildes, etc.), se deja
# elegir interactivamente de la lista de frames de la estacion.
# ---------------------------------------------------
frame_ref = RDK.Item('UR10 Base', ITEM_TYPE_FRAME)

if not frame_ref.Valid():
    print("Aviso: no se encontro un Frame llamado exactamente 'UR10 Base'.")
    print("Selecciona manualmente el sistema de referencia a usar...")
    frame_ref = RDK.ItemUserPick('Seleccione el sistema de referencia', ITEM_TYPE_FRAME)

if not frame_ref.Valid():
    raise Exception(
        "No se selecciono ningun Frame valido. Verifica que exista un sistema "
        "de referencia (Frame) en el arbol de la estacion de RoboDK."
    )

print(f"Usando sistema de referencia: '{frame_ref.Name()}'\n")

# Pose de la base del robot respecto a "UR10 Base", leida en vivo
T_ref_base = np.array(robot.Parent().PoseWrt(frame_ref).Rows())

# ---------------------------------------------------
# NOTA: no se define aqui una pose "T_target" (ground truth) de
# comparacion, ya que actualmente el UR10 no tiene un TCP/herramienta
# conectado en la estacion. Sin una herramienta definida, robot.Pose()/
# SolveFK()/PoseWrt() devuelven la pose de la BRIDA (flange), que es
# exactamente hasta donde llega esta tabla DH (el Link 6 termina en
# la brida, no en una punta de herramienta). Por eso la verificacion
# se hace contra SolveFK (Base->Brida) y PoseWrt (Referencia->Brida),
# sin depender de un valor objetivo fijo.
# ---------------------------------------------------
# ---------------------------------------------------
# Parametros DH del UR10, en MILIMETROS (alpha en radianes).
# Valores de precision completa tomados del panel "Nominal
# parameters" de RoboDK (Robot Parameters > Unlock advanced
# options), NO de urcontrol.conf truncado a 3 decimales.

DH_base = [
    {'a':    0.0,  'd': 127.300, 'alpha': np.deg2rad(90.0)},   # Link 1
    {'a': -612.7,  'd':   0.000, 'alpha': 0.0},                 # Link 2
    {'a': -572.3,  'd':   0.000, 'alpha': 0.0},                 # Link 3
    {'a':    0.0,  'd': 163.941, 'alpha': np.deg2rad(90.0)},   # Link 4
    {'a':    0.0,  'd': 115.700, 'alpha': np.deg2rad(-90.0)},  # Link 5 (alpha=-90, no +90)
    {'a':    0.0,  'd':  92.200, 'alpha': 0.0},                 # Link 6 (hasta el TCP)
]


def build_DH_matrix(a, d, alpha, theta):
    """Construye la matriz de transformacion homogenea DH (4x4)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def print_matrix(M, label, var_name):
    """
    Imprime una matriz 4x4 en formato tipo Julia (listo para copiar).

    label: descripcion textual de la transformacion (p. ej. entre que
           joints se ubica), se conserva como encabezado informativo.
    var_name: nombre de variable asignado a la matriz (p. ej. "T_1",
              ..., "T_6"), usado en la linea de asignacion "var_name = [...]".
    """
    print(f"{label}:")
    print(f"{var_name} = [")
    for i in range(4):
        row = " ".join(f"{M[i, j]: .6f}" for j in range(4))
        if i < 3:
            row += ";"
        print(row)
    print("]\n")


def print_quaternion_translation(M, label, var_name, t_var_name):
    """
    Extrae de la matriz homogenea M (4x4):
      - la submatriz de rotacion -> cuaternion [qw, qx, qy, qz],
        usando el comando robomath.pose_2_quaternion().
      - el vector de traslacion [x, y, z] (mm), en la linea siguiente.

    Este par (cuaternion de rotacion + vector de traslacion) es la
    representacion intermedia lista para construir el CUATERNION DUAL:
        q_dual = q_r + eps * (0.5 * t_quat * q_r)
    donde t_quat = [0, x, y, z] (cuaternion puro de traslacion).

    var_name:   nombre de variable a usar en la linea del cuaternion,
                ej. "h_1", "h_2", ..., "H_base_tcp".
    t_var_name: nombre de variable a usar en la linea del vector de
                traslacion, ej. "t_1", "t_2", ..., "t_6".
    """
    pose_mat = Mat(M.tolist())
    qw, qx, qy, qz = pose_2_quaternion(pose_mat)
    tx, ty, tz = M[0, 3], M[1, 3], M[2, 3]

    print(f"{label}:")
    print(f"{var_name} = quat( {qw: .6f}, {qx: .6f}, {qy: .6f}, {qz: .6f} )   # [qw, qx, qy, qz]")
    print(f"{t_var_name}   = [ {tx: .6f}, {ty: .6f}, {tz: .6f} ]   # [x, y, z] (mm)")
    print("")


def main():
    print("=== Generando 6 matrices locales DH del UR10 (RoboDK) ===\n")

    # -----------------------------------------------
    # 1) Leer angulos actuales de las articulaciones
    #    robot.Joints() -> Mat columna en GRADOS
    # -----------------------------------------------
    joints_deg = robot.Joints().list()
    if joints_deg is None or len(joints_deg) < 6:
        raise Exception("No se pudieron leer las 6 articulaciones del robot")

    thetas = [np.deg2rad(j) for j in joints_deg]

    labels = [
        "between joint '/joint{0}' and joint '/joint{1}'",
        "between joint '/joint{1}' and joint '/joint{2}'",
        "between joint '/joint{2}' and joint '/joint{3}'",
        "between joint '/joint{3}' and joint '/joint{4}'",
        "between joint '/joint{4}' and joint '/joint{5}'",
        "between joint '/joint{5}' and object '/connection'",
    ]

    # -----------------------------------------------
    # 2) Construir las 6 matrices locales (DH), imprimirlas y
    #    extraer su cuaternion de rotacion + vector de traslacion
    #    Nomenclatura: T_1..T_6 (matrices) y t_1..t_6 (traslaciones)
    # -----------------------------------------------
    T_links = []
    for i in range(6):
        dh = DH_base[i]
        T = build_DH_matrix(dh['a'], dh['d'], dh['alpha'], thetas[i])
        T_links.append(T)

        T_name = f"T_{i + 1}"
        h_name = f"h_{i + 1}"
        t_name = f"t_{i + 1}"

        print_matrix(T, labels[i], T_name)
        print_quaternion_translation(
            T, labels[i] + " [cuaternion + traslacion]", h_name, t_name
        )

    # -----------------------------------------------
    # 3) Composicion Base -> TCP (producto de las 6 matrices DH)
    # -----------------------------------------------
    T_base_TCP = np.eye(4)
    for T in T_links:
        T_base_TCP = T_base_TCP @ T

    # ---------------------------------------------------------
    # 3.1) Verificacion AISLADA: DH (Base->TCP) vs SolveFK nativo
    #      de RoboDK. Esto valida SOLO los parametros DH, sin
    #      mezclar el frame de referencia externo.
    # ---------------------------------------------------------
    T_base_TCP_solvefk = np.array(robot.SolveFK(robot.Joints()).Rows())
    error_base = np.max(np.abs(T_base_TCP - T_base_TCP_solvefk))

    print("=== Pose Base -> TCP (composicion DH pura) ===")
    print_matrix(T_base_TCP, "T_base_TCP (DH)", "T_base_tcp")
    print_quaternion_translation(
        T_base_TCP, "T_base_TCP (DH) [cuaternion + traslacion]",
        "H_base_tcp", "t_base_tcp"
    )
    print("=== Pose Base -> TCP (RoboDK SolveFK, referencia) ===")
    print_matrix(T_base_TCP_solvefk, "T_base_TCP (SolveFK)", "T_base_tcp_solvefk")
    print(f"Error maximo DH vs SolveFK (Base->TCP): {error_base:.6f} mm\n")

    # -----------------------------------------------
    # 4) Composicion DIRECTA de las 7 matrices:
    #    T_ref_TCP = T_ref_base * T_1 * T_2 * T_3 * T_4 * T_5 * T_6
    #    (T_ref_base es la pose de la base del robot respecto
    #     al sistema de referencia elegido)
    # -----------------------------------------------
    T_ref_TCP = T_ref_base @ T_base_TCP

    print(f"=== Pose de la BRIDA respecto a '{frame_ref.Name()}' (composicion DH) ===")
    print_matrix(T_ref_TCP, "T_ref_TCP", "T_ref_tcp")
    print_quaternion_translation(
        T_ref_TCP,
        f"T_ref_TCP respecto a '{frame_ref.Name()}' [cuaternion + traslacion]",
        "H_ref_tcp", "t_ref_tcp"
    )

    # ---------------------------------------------------------
    # 5) Verificacion usando el comando nativo de RoboDK disenado
    #    exactamente para esto: pose de un item respecto a otro.
    # ---------------------------------------------------------
    T_ref_TCP_robodk = np.array(robot.PoseWrt(frame_ref).Rows())

    print(f"=== Pose de la BRIDA respecto a '{frame_ref.Name()}' (nativo RoboDK: PoseWrt) ===")
    print_matrix(T_ref_TCP_robodk, "T_ref_TCP (RoboDK PoseWrt)", "T_ref_tcp_robodk")

    error_ref = np.max(np.abs(T_ref_TCP - T_ref_TCP_robodk))
    print(f"Error maximo (composicion DH  vs  PoseWrt RoboDK): {error_ref:.6f} mm\n")

    print("=== Si 'Error DH vs SolveFK (Base->TCP)' y el error de arriba son ===")
    print("=== < 1 mm, los parametros DH y la lectura del frame son correctos.===")
    print("=== Nota: como no hay TCP/herramienta conectada, estas poses llegan===")
    print("=== hasta la BRIDA del robot, no hasta la punta de una herramienta.===")


if __name__ == "__main__":
    main()
