# -*- coding: utf-8 -*-
"""
can_logger_win.py  (v3 -- con decodificacion en vivo)
=========================================================================
Lee el puerto serie del ST-Link (el mismo de tus prints RPM=...), separa
las lineas que son tramas CAN (formato candump) del resto, y:
  1) las guarda en sesion.log en hex crudo (para can_log_to_dataframe.py
     y para que el archivo sea un trace real, reproducible)
  2) si le pasas --dbc, ademas las decodifica con cantools y las imprime
     en pantalla con nombre de senal y valor fisico -- igual que ya ves
     tu linea de RPM=...

NO necesita CANable ni python-can como interfaz CAN.
SI necesita: pyserial, y cantools si usas --dbc (pip install pyserial cantools).

USO:
    python can_logger_win.py COM7 -o sesion.log --dbc bcm_sim.dbc

    Ctrl+C para detener.
=========================================================================
"""

import argparse
import re
import time

import serial

LINEA_CANDUMP = re.compile(r'\((?P<ts>\d+\.\d+)\)\s+can0\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)')


def formatear_decodificado(nombre_msg, decoded):
    """'BatteryVoltage=14.00 OilPressure=45.0 CrankSignalQuality=98' """
    partes = []
    for sig, val in decoded.items():
        if isinstance(val, float):
            partes.append("%s=%.2f" % (sig, val))
        else:
            partes.append("%s=%s" % (sig, val))
    return "[CAN %s] %s" % (nombre_msg, " ".join(partes))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('puerto', help='Puerto COM del ST-Link, ej. COM7')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('-o', '--out', default='sesion.log')
    ap.add_argument('--dbc', default=None, help='Si se pasa, decodifica cada trama en vivo con esta DBC')
    args = ap.parse_args()

    db = None
    if args.dbc:
        import cantools
        db = cantools.database.load_file(args.dbc)
        print("DBC cargada: %s (%d mensajes)" % (args.dbc, len(db.messages)))

    print("Abriendo %s @ %d baud ..." % (args.puerto, args.baud))
    ser = serial.Serial(args.puerto, args.baud, timeout=1.0)

    frames_capturados = 0
    lineas_totales = 0
    t_inicio = time.time()

    print("Grabando tramas CAN en %s -- Ctrl+C para detener\n" % args.out)

    try:
        with open(args.out, 'w') as f:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    linea = raw.decode('utf-8', errors='replace').strip()
                except Exception:
                    continue
                if not linea:
                    continue
                lineas_totales += 1

                m = LINEA_CANDUMP.match(linea)
                if m:
                    f.write(linea + '\n')
                    f.flush()
                    frames_capturados += 1

                    if db is not None:
                        can_id = int(m.group('id'), 16)
                        data = bytes.fromhex(m.group('data'))
                        try:
                            msg = db.get_message_by_frame_id(can_id)
                            decoded = msg.decode(data, decode_choices=False)
                            print(formatear_decodificado(msg.name, decoded))
                        except Exception:
                            print("  [CAN 0x%X] (fuera de la DBC o error al decodificar)" % can_id)
                else:
                    print(linea)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nDetenido. Frames CAN capturados: %d (de %d lineas leidas)" % (frames_capturados, lineas_totales))


if __name__ == '__main__':
    main()
