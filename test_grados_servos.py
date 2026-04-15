#!/usr/bin/env python3
"""
Prueba y calibración por grados usando SafeController.

Toda interacción con el hardware pasa por la capa de seguridad:
SafeController → ArmController → PCA9685.

Útil para comprobar límites mecánicos reales y anotar ``angle_safe_*``,
``pulse_*`` y ``angle_home_deg`` antes de fijarlos en servo_config.json.

Antes, en el venv de la Pi::

    pip install -r requirements.txt

Ejecutar desde la raíz del proyecto::

    python3 test_grados_servos.py

    python3 test_grados_servos.py --home          # ir a HOME suave al inicio
    python3 test_grados_servos.py -j base -a 90   # un solo movimiento y salir
    python3 test_grados_servos.py --sim            # modo simulación (sin hardware)

Articulaciones disponibles: base, shoulder, elbow, wrist, gripper.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

try:
    from arm_system.safety.safe_controller import SafeController
except ImportError as exc:
    print(
        "Error de importación (¿faltan dependencias en el venv?):\n"
        f"  {exc}\n"
        "En la Raspberry Pi, desde la raíz del repo:\n"
        "  pip install -r requirements.txt\n"
        "Mínimo para PCA9685 / I2C:\n"
        "  pip install adafruit-blinka adafruit-circuitpython-pca9685",
        file=sys.stderr,
    )
    sys.exit(1)


def _mostrar_limites(ctrl: SafeController) -> None:
    print("\n--- Rangos seguros (servo_config.json) ---")
    joints = ctrl.joints
    if joints:
        for key in ctrl.iter_joint_keys():
            if key not in joints:
                continue
            j = joints[key]
            print(
                f"  {key:8s}  canal {j.channel}  "
                f"[{j.angle_safe_min_deg:.0f}° … {j.angle_safe_max_deg:.0f}°]  "
                f"home={j.angle_home_deg:.0f}°  "
                f"pulso {j.pulse_min_us}–{j.pulse_max_us} µs"
                + (f"  invert={j.invert}" if key != "gripper" else "")
            )
            if key == "gripper" and (
                j.angle_open_deg is not None or j.angle_close_deg is not None
            ):
                print(
                    f"            abrir≈{j.angle_open_deg}°  cerrar≈{j.angle_close_deg}°"
                )
    else:
        # Modo simulación: mostrar límites desde config cargada
        print("  [SIMULACIÓN — sin especificaciones de hardware]")
        for key in ctrl.iter_joint_keys():
            ang = ctrl.get_angle(key)
            print(f"  {key:8s}  ángulo actual={ang:.1f}°")
    print("---\n")


def _parse_line(line: str) -> tuple[str, list[str]] | None:
    line = line.strip()
    if not line:
        return None
    parts = line.split()
    cmd = parts[0].lower()
    return cmd, parts[1:]


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Probar grados de los servos via SafeController."
    )
    ap.add_argument("--home", action="store_true", help="Llevar a HOME suave al conectar.")
    ap.add_argument(
        "-j", "--joint",
        help="Un solo movimiento: articulación (base|shoulder|elbow|wrist|gripper).",
    )
    ap.add_argument("-a", "--angle", type=float, help="Ángulo en grados (con --joint).")
    ap.add_argument(
        "--no-smooth",
        action="store_true",
        help="Sin interpolación (pulso directo; más brusco).",
    )
    ap.add_argument(
        "--sim",
        action="store_true",
        help="Forzar modo simulación aunque haya hardware disponible.",
    )
    args = ap.parse_args()

    if args.joint is not None and args.angle is None:
        ap.error("--joint requiere --angle")
    if args.angle is not None and args.joint is None:
        ap.error("--angle requiere --joint")

    ctrl: SafeController | None = None
    try:
        ctrl = SafeController(simulation_mode=args.sim)

        sim_tag = " [SIMULACIÓN]" if ctrl.is_simulation else ""
        print(f"\nSafeController inicializado{sim_tag}.")
        _mostrar_limites(ctrl)

        if args.home:
            logging.info("Moviendo a HOME (suave)...")
            ctrl.go_home()

        if args.joint is not None:
            jn = args.joint.strip().lower()
            known = set(ctrl.iter_joint_keys())
            if jn not in known:
                print(
                    f"Articulación desconocida: {jn}. Válidas: {sorted(known)}",
                    file=sys.stderr,
                )
                sys.exit(2)
            smooth = not args.no_smooth
            ok = ctrl.move_safe(jn, args.angle, smooth=smooth)
            if ok:
                print(f"Aplicado: {jn} = {ctrl.get_angle(jn):.1f}° (smooth={smooth})")
            else:
                print(f"Movimiento rechazado por SafeController. Revisar logs.", file=sys.stderr)
            return

        known_joints = set(ctrl.iter_joint_keys())
        print(
            "Modo interactivo. Comandos:\n"
            "  <articulación> <grados>   p.ej.  base 90    shoulder 120\n"
            "  home                      ir a HOME suave\n"
            "  open / close              pinza (usa angle_open/close del JSON)\n"
            "  limits                    volver a mostrar rangos\n"
            "  status                    ángulos actuales\n"
            "  emergency                 activar emergency stop\n"
            "  reset                     reiniciar emergency stop\n"
            "  q / quit                  salir y apagar PWM\n"
        )

        while True:
            try:
                raw = input("grados> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            parsed = _parse_line(raw)
            if parsed is None:
                continue
            cmd, rest = parsed

            if cmd in ("q", "quit", "exit"):
                break

            if cmd in ("limits", "l", "limites"):
                _mostrar_limites(ctrl)
                continue

            if cmd == "status":
                angles = ctrl.get_all_angles()
                print("  Ángulos actuales:")
                for k, v in sorted(angles.items()):
                    print(f"    {k:8s} = {v:.1f}°")
                print(f"  Emergency: {ctrl.is_emergency}  Simulación: {ctrl.is_simulation}")
                continue

            if cmd == "home":
                ok = ctrl.go_home()
                print("OK: HOME" if ok else "RECHAZADO: emergency activo o error de hardware.")
                continue

            if cmd == "open":
                ok = ctrl.open_gripper()
                print(f"OK: pinza abierta → {ctrl.get_angle('gripper'):.1f}°" if ok else "RECHAZADO.")
                continue

            if cmd == "close":
                ok = ctrl.close_gripper()
                print(f"OK: pinza cerrada → {ctrl.get_angle('gripper'):.1f}°" if ok else "RECHAZADO.")
                continue

            if cmd == "emergency":
                ctrl.emergency_stop()
                print("*** EMERGENCY STOP activado. Usa 'reset' para reanudar. ***")
                continue

            if cmd == "reset":
                ctrl.reset_emergency()
                print("Emergency stop reiniciado.")
                continue

            if cmd not in known_joints:
                print(
                    f"No reconocido. Articulaciones: {', '.join(sorted(known_joints))}"
                )
                continue

            if len(rest) < 1:
                print(f"Falta el ángulo: p.ej.  {cmd} 90")
                continue

            try:
                angle = float(rest[0])
            except ValueError:
                print("Ángulo no numérico.")
                continue

            ok = ctrl.move_safe(cmd, angle, smooth=True)
            if ok:
                print(f"  → {cmd} = {ctrl.get_angle(cmd):.1f}° (lógico)")
            else:
                print(f"  → Movimiento rechazado. Revisar logs.")

    finally:
        if ctrl is not None:
            ctrl.close()
            print("SafeController cerrado (PWM liberado).")


if __name__ == "__main__":
    main()
