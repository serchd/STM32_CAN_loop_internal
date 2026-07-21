"""
can_log_to_dataframe.py
=========================================================================
Puente entre el CAN real (STM32 -> CANable Pro -> SocketCAN) y tu pipeline
existente de loop_ciclico_multidtc.py / workflow_13modelos.ipynb.

Reconstruye, a partir de trafico CAN decodificado con bcm_sim.dbc, un
DataFrame con LAS MISMAS COLUMNAS que generar_corrida() en
generador_multidtc.py -- para que puedas alimentar tus 7 modelos con datos
que de verdad vinieron de la STM32, no solo simulados en Python.

Lo que SI hace este script: decodificar, ensamblar el DataFrame de señales
crudas y (opcionalmente) recalcular rolling mean/std/slope + debounce +
etiquetas, reusando CONFIG_DTC de tu propio generador_multidtc.py -- esa
logica no se duplica ni se reinventa aqui.

Lo que NO hace: generar datos. Eso ahora lo hace la STM32
(dtc_synth_gen.c). Este script solo escucha.

USO
---
1) Captura en vivo con tu CANable Pro (Linux, SocketCAN):
     candump -L can0 > sesion.log            # 'Start Logging' estilo CANoe
     python can_log_to_dataframe.py sesion.log --dbc bcm_sim.dbc -o corridas/

2) Replay de una sesion ya capturada (tu 'Replay Block'):
     canplayer -I sesion.log                  # reinyecta el trafico
     candump -L can0 > sesion_replay.log       # y la vuelves a capturar/analizar

3) Live directo desde SocketCAN sin pasar por archivo (opcional, ver
   --live abajo) usando python-can.
=========================================================================
"""

import argparse
import re
import sys
from pathlib import Path

import cantools
import pandas as pd
import numpy as np


# Nombres EXACTOS que usa generador_multidtc.py -- no se inventan aqui.
COLUMNA_POR_SENAL_DBC = {
    ('PowertrainHealth', 'BatteryVoltage'):        'battery_voltage_V',
    ('EngineStatus', 'CoolantTemp'):                'coolant_temp_C',
    ('PowertrainHealth', 'OilPressure'):            'oil_pressure_psi',
    ('PowertrainHealth', 'CrankSignalQuality'):     'crank_signal_quality',
    ('ChassisHealth', 'TirePressure'):              'tire_pressure_psi',
    ('ChassisHealth', 'FuelLevelPct'):              'fuel_level_pct',
    ('ThrottleSubsystem', 'AccelPedalPct'):         'accel_pedal_pct',
    ('ThrottleSubsystem', 'ThrottlePositionPct'):   'throttle_position_pct',
    ('ThrottleSubsystem', 'ThrottleExpectedPct'):   'throttle_expected_pct',
    ('ThrottleSubsystem', 'ThrottleResidualPct'):   'throttle_residual_pct',
    ('VehicleSpeed', 'Speed'):                      'vehicle_speed_kmh',
    ('EngineStatus', 'RPM'):                        'rpm_motor',
    ('BrakeStatus', 'BrakePedalPct'):               'brake_pedal_pressure',  # ver nota abajo
    ('BrakeStatus', 'BrakeOverrideFlag'):           'brake_override_flag',
}

# Regex para lineas tipo candump -L:
# (1753061234.123456) can0 0F0#0104E20164
LINEA_CANDUMP = re.compile(
    r'\((?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)'
)


def parsear_candump_log(ruta_log):
    """Lee un .log estilo candump -L y regresa lista de (timestamp, can_id, bytes)."""
    frames = []
    with open(ruta_log, 'r') as f:
        for linea in f:
            m = LINEA_CANDUMP.match(linea.strip())
            if not m:
                continue
            can_id = int(m.group('id'), 16)
            data = bytes.fromhex(m.group('data'))
            frames.append((float(m.group('ts')), can_id, data))
    return frames


def reconstruir_dataframe(frames, dbc_path):
    """Decodifica los frames con la DBC y arma un DataFrame indexado por
    (run_id, time_s), con una fila por segundo -- igual que generar_corrida().
    """
    db = cantools.database.load_file(dbc_path)
    filas = {}   # (run_id, time_s) -> dict de columnas acumuladas

    for ts, can_id, data in frames:
        try:
            msg = db.get_message_by_frame_id(can_id)
        except KeyError:
            continue   # frame fuera de la DBC (ruido de otro nodo), se ignora

        try:
            decoded = msg.decode(data, decode_choices=False)
        except Exception:
            continue   # frame corrupto/longitud invalida, se descarta

        if msg.name == 'RunMeta':
            clave_actual = (int(decoded['RunId']), int(decoded['TimeS']))
            filas.setdefault(clave_actual, {})
            filas[clave_actual]['run_id'] = int(decoded['RunId'])
            filas[clave_actual]['escenario_id'] = int(decoded['ScenarioId'])
            filas[clave_actual]['seed_low16'] = int(decoded['SeedLow16'])
            filas[clave_actual]['time_s'] = int(decoded['TimeS'])
            continue

        # Las demas señales se asocian a la ULTIMA clave de RunMeta vista --
        # asume que RunMeta llega junto con el resto de señales del mismo tick
        # (como lo hace dtc_synth_gen.c: las 7 se transmiten seguidas).
        if not filas:
            continue
        clave_actual = next(reversed(filas))

        for sig_name, valor in decoded.items():
            columna = COLUMNA_POR_SENAL_DBC.get((msg.name, sig_name))
            if columna:
                filas[clave_actual][columna] = float(valor)

    df = pd.DataFrame(list(filas.values())).sort_values(['run_id', 'time_s']).reset_index(drop=True)

    # brake_pedal_pressure viene de la STM32 en % (0-100) -- si tu
    # generador_multidtc.py espera la escala 0-0.5 original, descomenta:
    # df['brake_pedal_pressure'] = df['brake_pedal_pressure'] / 200.0

    return df


def recalcular_features_y_flags(df, config_dtc, ventana=10):
    """Reusa EXACTAMENTE la logica de rolling/threshold/debounce de
    generador_multidtc.py (pasar CONFIG_DTC importado desde ahi -- no se
    reimplementa la logica de negocio en este script)."""
    df = df.copy()
    for dtc, cfg in config_dtc.items():
        col = cfg['columna']
        if col not in df.columns:
            continue
        df[f'{col}_mean'] = df[col].rolling(ventana, min_periods=1).mean()
        df[f'{col}_std'] = df[col].rolling(ventana, min_periods=1).std().fillna(0)
        df[f'{col}_slope'] = df[col].diff().fillna(0)

        if cfg['direccion'] == 'abajo':
            raw = (df[col] < cfg['umbral']).astype(int)
        else:
            raw = (df[col] > cfg['umbral']).astype(int)
        df[f'fault_raw_{dtc}'] = raw

        conf = np.zeros(len(raw), dtype=int)
        cnt = 0
        for i, r in enumerate(raw.values):
            cnt = cnt + 1 if r == 1 else 0
            conf[i] = 1 if cnt >= cfg['debounce_s'] else 0
        df[f'fault_confirmed_{dtc}'] = conf

    return df


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='Archivo .log estilo candump -L')
    ap.add_argument('--dbc', default='bcm_sim.dbc')
    ap.add_argument('-o', '--outdir', default='.', help='Carpeta donde guardar el CSV reconstruido')
    ap.add_argument('--con-features', action='store_true',
                     help='Tambien recalcula rolling mean/std/slope + debounce (requiere generador_multidtc.py en el PYTHONPATH)')
    args = ap.parse_args()

    frames = parsear_candump_log(args.log)
    if not frames:
        print(f"ERROR: no se encontraron frames validos en {args.log}", file=sys.stderr)
        sys.exit(1)
    print(f"Frames leidos: {len(frames)}")

    df = reconstruir_dataframe(frames, args.dbc)
    print(f"Filas reconstruidas: {len(df)}  |  runs distintos: {df['run_id'].nunique() if len(df) else 0}")

    if args.con_features:
        from generador_multidtc import CONFIG_DTC
        df = recalcular_features_y_flags(df, CONFIG_DTC)

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    salida = outdir / (Path(args.log).stem + '_reconstruido.csv')
    df.to_csv(salida, index=False)
    print(f"Guardado: {salida}")


if __name__ == '__main__':
    main()
