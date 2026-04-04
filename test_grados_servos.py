#!/usr/bin/env python3
"""
Prueba y calibración por grados (ArmController + arm_system/servo_config.json).

Útil para comprobar límites mecánicos reales y anotar ``angle_safe_*``, ``pulse_*``
y ``angle_home_deg`` antes de fijarlos en el JSON.

Ejecutar en la Raspberry Pi desde la raíz del proyecto::

    python3 test_grados_servos.py

    python3 test_grados_servos.py --home          # ir a HOME suave al inicio
    python3 test_grados_servos.py -j base -a 90   # un solo movimiento y salir

Articulaciones: base, shoulder, elbow, gripper (sin muñeca en este controlador).
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

from arm_system.control.arm_controller import ArmController


def _mostrar_limites(arm: ArmController) -> None:
    print("\n--- Rangos seguros (servo_config.json) ---")
    for key in arm.iter_joint_keys():
        j = arm.joints[key]
        print(
            f"  {key:8s}  canal {j.channel}  "
            f"[{j.angle_safe_min_deg:.0f}° … {j.angle_safe_max_deg:.0f}°]  "
            f"home={j.angle_home_deg:.0f}°  "
            f"pulso {j.pulse_min_us}–{j.pulse_max_us} µs"
            + (f"  invert={j.invert}" if key != "gripper" else "")
        )
        if key == "gripper" and (j.angle_open_deg is not None or j.angle_close_deg is not None):
            print(
                f"            abrir≈{j.angle_open_deg}°  cerrar≈{j.angle_close_deg}°"
            )
    print("---\n")


def _parse_line(line: str) -> tuple[str, list[str]] | None:
    line = line.strip()
    if not line:
        return None
    parts = line.split()
    cmd = parts[0].lower()
    return cmd, parts[1:]


def main() -> None:
    ap = argparse.ArgumentParser(description="Probar grados de los servos (ArmController).")
    ap.add_argument("--home", action="store_true", help="Llevar a HOME suave al conectar.")
    ap.add_argument("-j", "--joint", help="Un solo movimiento: articulación (base|shoulder|elbow|gripper).")
    ap.add_argument("-a", "--angle", type=float, help="Ángulo en grados (con --joint).")
    ap.add_argument(
        "--no-smooth",
        action="store_true",
        help="Sin interpolación (pulso directo; más brusco).",
    )
    args = ap.parse_args()

    if args.joint is not None and args.angle is None:
        ap.error("--joint requiere --angle")
    if args.angle is not None and args.joint is None:
        ap.error("--angle requiere --joint")

    arm: ArmController | None = None
    try:
        arm = ArmController()
        _mostrar_limites(arm)

        if args.home:
            logging.info("Moviendo a HOME (suave)...")
            arm.initialize_to_home_smooth()

        if args.joint is not None:
            jn = args.joint.strip().lower()
            if jn not in ArmController.KNOWN_JOINTS:
                print(f"Articulación desconocida: {jn}. Válidas: {ArmController.KNOWN_JOINTS}", file=sys.stderr)
                sys.exit(2)
            smooth = not args.no_smooth
            ang = arm.set_joint_angle(jn, args.angle, smooth=smooth)
            print(f"Aplicado: {jn} = {ang:.1f}° (smooth={smooth})")
            return

        print(
            "Modo interactivo. Comandos:\n"
            "  <articulación> <grados>   p.ej.  base 90    shoulder 120\n"
            "  home                      ir a HOME suave\n"
            "  open / close              pinza (usa angle_open/close del JSON)\n"
            "  limits                    volver a mostrar rangos\n"
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
                _mostrar_limites(arm)
                continue
            if cmd == "home":
                arm.initialize_to_home_smooth()
                print("OK: HOME")
                continue
            if cmd == "open":
                a = arm.open_gripper(smooth=True)
                print(f"OK: pinza abrir → {a:.1f}°")
                continue
            if cmd == "close":
                a = arm.close_gripper(smooth=True)
                print(f"OK: pinza cerrar → {a:.1f}°")
                continue

            if cmd not in ArmController.KNOWN_JOINTS:
                print(f"No reconocido. Articulaciones: {', '.join(ArmController.KNOWN_JOINTS)}")
                continue
            if len(rest) < 1:
                print("Falta el ángulo: p.ej.  base 90")
                continue
            try:
                angle = float(rest[0])
            except ValueError:
                print("Ángulo no numérico.")
                continue

            ang = arm.set_joint_angle(cmd, angle, smooth=True)
            print(f"  → {cmd} = {ang:.1f}° (lógico)")

    finally:
        if arm is not None:
            arm.close()
            print("PWM liberado (close).")


if __name__ == "__main__":
    main()
