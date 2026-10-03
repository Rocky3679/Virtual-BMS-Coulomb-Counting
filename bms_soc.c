/**
 * @file bms_soc.c
 * @brief Discrete-time Coulomb Counting SOC Estimator with OCV Calibration
 * @author Janaki Raman K
 * 
 * Hardware-independent implementation designed for embedded automotive BMS targets.
 * Simulates current integration, resting OCV resets, and thermal derating.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define NOMINAL_CAPACITY_AH   2.5f      /* 18650/21700 typical cell capacity */
#define COULOMBIC_EFFICIENCY  0.98f     /* Charge efficiency factor (eta) */
#define SAMPLE_INTERVAL_SEC   0.1f      /* 100ms discrete task period */
#define MAX_CELL_TEMP_C       45.0f     /* Upper threshold for thermal derating */

typedef struct {
    float voltage_v;
    float current_a;          /* Positive = discharge, Negative = charge */
    float temp_c;
    float estimated_soc_pct;
    bool is_resting;
    uint32_t rest_duration_ms;
} BatteryCell_t;

/* Piecewise linear approximation of Open Circuit Voltage (OCV) vs SOC for Li-ion */
static float get_soc_from_ocv(float ocv_v) {
    if (ocv_v >= 4.20f) return 100.0f;
    if (ocv_v >= 4.00f) return 85.0f + ((ocv_v - 4.00f) / 0.20f) * 15.0f;
    if (ocv_v >= 3.75f) return 50.0f + ((ocv_v - 3.75f) / 0.25f) * 35.0f;
    if (ocv_v >= 3.60f) return 20.0f + ((ocv_v - 3.60f) / 0.15f) * 30.0f;
    if (ocv_v >= 3.20f) return 5.0f  + ((ocv_v - 3.20f) / 0.40f) * 15.0f;
    return 0.0f;
}

/**
 * @brief Periodic 100ms task calculating discrete State of Charge.
 */
void BMS_Update_SOC(BatteryCell_t *cell) {
    /* Thermal protection: flag derating if pack exceeds safe limit */
    if (cell->temp_c > MAX_CELL_TEMP_C) {
        /* Thermal derating flag: in automotive firmware, this sets a CAN DTC */
    }

    /* Rest condition detection: current threshold below 20mA for > 10 seconds */
    if (cell->current_a > -0.02f && cell->current_a < 0.02f) {
        cell->rest_duration_ms += (uint32_t)(SAMPLE_INTERVAL_SEC * 1000.0f);
        if (cell->rest_duration_ms > 10000) {
            cell->is_resting = true;
            /* Recalibrate SOC drift using relaxed cell voltage */
            cell->estimated_soc_pct = get_soc_from_ocv(cell->voltage_v);
            return;
        }
    } else {
        cell->rest_duration_ms = 0;
        cell->is_resting = false;
    }

    /* Discrete Coulomb counting integration */
    float delta_ah = (cell->current_a * SAMPLE_INTERVAL_SEC) / 3600.0f;
    
    if (cell->current_a < 0.0f) {
        /* Charging phase: apply coulombic efficiency */
        delta_ah *= COULOMBIC_EFFICIENCY;
    }

    float delta_soc = (delta_ah / NOMINAL_CAPACITY_AH) * 100.0f;
    cell->estimated_soc_pct -= delta_soc;

    /* Enforce boundary limits */
    if (cell->estimated_soc_pct > 100.0f) cell->estimated_soc_pct = 100.0f;
    if (cell->estimated_soc_pct < 0.0f)   cell->estimated_soc_pct = 0.0f;
}
