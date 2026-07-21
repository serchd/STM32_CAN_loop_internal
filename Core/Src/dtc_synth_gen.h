#ifndef DTC_SYNTH_GEN_H
#define DTC_SYNTH_GEN_H

/**
 * dtc_synth_gen.h / .c
 * =============================================================================
 * Puerto a C, streaming (un tick por segundo, sin arreglos numpy), del
 * generador de vehiculos sinteticos multi-DTC (generador_multidtc.py).
 *
 * Reproduce, corrida tras corrida y en el MISMO orden que TIPOS_ESCENARIO,
 * un "vehiculo nuevo" con una semilla fresca -- igual que hacia tu script en
 * Python -- pero transmitiendo cada muestra por CAN en tiempo real usando la
 * DBC bcm_sim.dbc, en vez de escribir un CSV.
 *
 * IMPORTANTE -- lo que SI y NO se replica:
 *  - SI se replica: la FORMA de cada perfil de falla (progresiva/subita,
 *    umbral, ventana de aparicion) tal cual esta en generador_multidtc.py.
 *  - NO es bit-exacto al dataset de Python: el PRNG de este archivo
 *    (xorshift32 + Box-Muller) es distinto al PCG64 de numpy. Con la misma
 *    semilla en la STM32 obtienes SIEMPRE la misma corrida (reproducible en
 *    banco), pero no los mismos numeros que produciria numpy con esa semilla.
 *  - El calculo de rolling mean/std/slope, el debounce y las etiquetas
 *    fault_raw/fault_confirmed/fault_early NO se hacen aqui -- eso se deja
 *    igual que hoy, del lado de Python, sobre los datos ya decodificados de
 *    la DBC (ver notas de integracion con loop_ciclico_multidtc.py).
 * =============================================================================
 */

void StartDtcSynthGenTask(void *argument);

#endif /* DTC_SYNTH_GEN_H */
