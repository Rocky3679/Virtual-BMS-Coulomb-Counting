# Virtual BMS: Coulomb Counting & OCV SOC Estimator

Embedded C implementation of a discrete-time State-of-Charge (SOC) estimation algorithm for automotive Lithium-ion battery packs. Validated on synthetic EV drive-cycle profiles using lightweight cloud testbenches.

## Architectural Highlights

- **Discrete Integration Loop:** Executes at 10 Hz (100 ms period) to calculate current accumulation using discrete Coulomb counting.
- **OCV Drift Correction:** Mitigates open-loop integration drift by detecting cell relaxation phases ($I < 20\text{ mA}$ for $>10\text{ s}$) and recalibrating the baseline against Open Circuit Voltage curves.
- **Thermal Limiting:** Monitors cell temperature against safety limits ($>45^\circ\text{C}$) to flag overtemperature derating states.
- **Hardware Agnostic:** Written in portable ANSI C suitable for deployment to STM32, ESP32, or NXP S32K automotive microcontrollers.

## State Estimation Model

The core estimation loop calculates pack capacity at step $k$:

$$SOC[k] = SOC[k-1] - \left( \frac{\eta \cdot I[k] \cdot \Delta t}{Q_{\text{nom}}} \right) \times 100$$

Where:
- $\eta$: Coulombic efficiency ($0.98$ for charging, $1.0$ for discharging)
- $I[k]$: Instantaneous pack current (Amperes)
- $\Delta t$: Sample period ($0.1\text{ s}$)
- $Q_{\text{nom}}$: Nominal pack capacity ($2.5\text{ Ah}$)

## Execution & Verification

Run the Python verification testbench or compile locally via GCC:

```bash
gcc -O2 -Wall bms_soc.c -o bms_estimator
