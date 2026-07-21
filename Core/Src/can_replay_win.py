"""
can_replay_win.py
=========================================================================
Equivalente Windows a "canplayer -I sesion.log". Reinyecta un log
capturado con can_logger_win.py (o cualquier log formato candump)
respetando los tiempos originales entre frames -- tu "Replay Block".

USO:
    python can_replay_win.py sesion.log COM5 --bitrate 125000
=========================================================================
"""

import argparse
import time

import can

from can_log_to_dataframe import parsear_candump_log   # reusa el mismo parser


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', help='Archivo .log a reproducir')
    ap.add_argument('puerto', help='Puerto COM del CANable Pro, ej. COM5')
    ap.add_argument('--interface', default='slcan')
    ap.add_argument('--bitrate', type=int, default=125000)
    args = ap.parse_args()

    channel = args.puerto if args.interface == 'slcan' else 0
    bus = can.interface.Bus(channel=channel, interface=args.interface, bitrate=args.bitrate)

    frames = parsear_candump_log(args.log)
    if not frames:
        print("No se encontraron frames en el log.")
        return

    print(f"Reproduciendo {len(frames)} frames...")
    t0_log = frames[0][0]
    t0_real = time.time()

    for ts, can_id, data in frames:
        objetivo = t0_real + (ts - t0_log)
        espera = objetivo - time.time()
        if espera > 0:
            time.sleep(espera)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(msg)

    print("Replay terminado.")
    bus.shutdown()


if __name__ == '__main__':
    main()
