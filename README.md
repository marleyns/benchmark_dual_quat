# benchmark_dual_quat
Repositorio que contiene los scripts para el @benchmark de matrices homogéneas y cuaterniones duales en FK  para el cobot UR10.
# Scripts de Cinemática Directa y Cuaterniones Duales – UR10

Repositorio complementario del estudio sobre la equivalencia entre matrices de transformación homogénea y cuaterniones duales unitarios aplicados a la cinemática directa del manipulador UR10.

## Scripts incluidos

| Archivo                    | Lenguaje | Descripción breve                          |
|---------------------------|----------|--------------------------------------------|
| `download-FK.py`          | Python   | Extracción de datos de cinemática directa desde RoboDK |
| `dualquat_composicion.jl` | Julia    | Construcción y manipulación de cuaterniones duales     |

---

### 1. Script Python (`download-FK.py`)

Para la obtención de los datos de la cinemática directa (FK) del UR10 se empleó el API de Python de RoboDK. El marco de referencia utilizado es “UR10 Base”. Mediante la función `PoseWrt()` se recuperó la pose relativa entre la base del robot y la brida. Paralelamente, se extrajeron los ángulos articulares con `robot.Joints()` (convertidos a radianes) para construir las seis matrices de transformación homogénea locales. Estas matrices se contrastaron con el resultado de `robot.SolveFK()` para validar la coherencia de la cadena cinemática. Dado que la estación no cuenta con herramienta (TCP) conectada, las poses corresponden a la brida del manipulador. Finalmente, con `pose_2_quaternion()` se obtuvo el cuaternión de rotación `[qw, qx, qy, qz]` y el vector de traslación `[x, y, z]`, datos indispensables para la posterior construcción del cuaternión dual unitario en Julia.

**Requisitos:**
- RoboDK instalado y en ejecución
- API de Python de RoboDK configurada
- Estación con el robot UR10 cargado

**Ejecución:**
```bash
python download-FK.py
