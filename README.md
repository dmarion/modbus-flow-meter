# STM32L031 Water Flow Meter – Modbus RTU Server (Bare Metal)

Bare-metal firmware for **STM32L031** that measures water flow and consumption from a **YF-B5 water flow sensor** and exposes the data over **RS-485 Modbus RTU**.

---

## Features

- **Water Flow Measurement**
  - Counts pulses from YF-B5 hall sensor via hardware timer capture.
  - Computes instantaneous flow rate (L/min).
  - Accumulates total water consumption (liters).
  - MCU led blinks once for each dL

- **Modbus RTU Slave**
  - RS-485 half‑duplex on **LPUART1 (PA2/PA3)**.
  - Supports **only** Modbus function code `0x03` (Read Holding Registers).
  - **DE/RE driven by GPIO PB1** (no CTS/RTS required).

- **Bare Metal Implementation**
  - No HAL or CMSIS drivers.
  - Minimal C runtime, stripped `libnosys.a`.
  - Optimized for low power and small flash footprint.

---

## Hardware

| Component                           | Notes |
|-------------------------------------|-------|
| **STM32L031H6** (LQFP‑32)           | Main MCU |
| **YF‑B5 Water Flow Sensor**         | Hall‑effect pulse output proportional to flow |
| **EL817 Optocoupler**               | Isolates YF‑B5 output from MCU; LED side with series resistor; transistor side to timer input with pull‑up |
| **RS‑485 Transceiver** (e.g., SN65HVD75) | Half‑duplex; **DE/RE** tied and driven by **PA1** |
| **120 Ω termination DIP switch**    | Selectable RS‑485 bus termination |
| **BOOT0 tactile switch**            | Forces bootloader mode |
| **Power Supply**                    | 12 V → 3.3 V LDO (e.g., MCP1703A) |
| **SWD Header**                      | SWDIO + SWCLK for programming/debug |
| **Status / Power LEDs**             | MCU status and 3.3 V power indication |

### Pin Mapping

| MCU Pin | Function                         | Notes |
|--------:|----------------------------------|-------|
| **PA0** | **TIM2_CH1**                     | Pulse input from **EL817** (isolated YF‑B5 signal) |
| **PA1** | **GPIO output**                  | MCU LED |
| **PA2** | **LPUART1_TX**                   | RS‑485 **DI** (Modbus TX) |
| **PA3** | **LPUART1_RX**                   | RS‑485 **RO** (Modbus RX) |
| **PA4** | **GPIO input**                   | MODBUS addr bit 0 |
| **PA5** | **GPIO input**                   | MODBUS addr bit 1 |
| **PB1** | **GPIO output**                  | RS‑485 **DE/RE** control (active‑high) |
| **PB6** | **USART2_TX**                    | Debug console TX |
| **PB7** | **USART2_RX**                    | Debug console RX |
| **PA13**| **SWDIO**                        | Debug data line |
| **PA14**| **SWCLK**                        | Debug clock line |
| **BOOT0** | —                              | Connected to tactile switch |
| **VDD/VSS** | —                            | 3.3 V / GND |

---

## Modbus Register Map

| Address | Description            | Unit        |
|---------|------------------------|-------------|
| 0x0000  | Total volume **MSW**   | —           |
| 0x0001  | Total volume **LSW**   | 0.1 L       |
| 0x0002  | Instantaneous flow     | 0.1 L/min   |
| 0x0003  | RS‑485 RX error count  | errors      |

> **Scaling:** total volume and flow are reported in integer units of **0.1** (multiply by 0.1 to get human‑readable values).

---

## YF‑B5 Hall Sensor

The YF‑B5 typically outputs:

```
F [Hz] = 450 × Q [L/min]
```

---

## Building

### Requirements
- `arm-none-eabi-gcc` toolchain
- `openocd` for programming/debugging
- `gdb-multiarch` (or `arm-none-eabi-gdb`)

### Clone and build
```bash
git clone https://github.com/dmarion/modbus-flow-meter.git
cd <repo>/src
make
```

---

## Flashing

Start OpenOCD in a separate terminal:
```bash
make openocd
```

Then in another terminal:
```bash
make flash
```

---

## Debugging
```bash
make openocd
# In another terminal:
make debug
```

The debug console is available on **USART2 (PB6/PB7)** at the baud rate defined in the source.

---

## ESPHome Integration

Example ESPHome configuration for two STM32‑based flow meters connected via RS‑485:

```yaml
modbus:
  - id: modbus1
    uart_id: uart1
  - id: modbus2
    uart_id: uart2

uart:
  - id: uart1
    tx_pin: GPIO17
    rx_pin: GPIO16
    baud_rate: 9600
    stop_bits: 1
    parity: NONE
  - id: uart2
    tx_pin: GPIO19
    rx_pin: GPIO18
    baud_rate: 9600
    stop_bits: 1
    parity: NONE

modbus_controller:
  - id: stm32_s1
    address: 1
    modbus_id: modbus1
    update_interval: 5s
  - id: stm32_s2
    address: 2
    modbus_id: modbus2
    update_interval: 5s

sensor:
  - platform: modbus_controller
    modbus_controller_id: stm32_s1
    name: "S1 Water Volume"
    register_type: holding
    address: 0x0000           # MSW at 0x0000, LSW at 0x0001
    value_type: U_DWORD
    register_count: 2
    unit_of_measurement: "L"
    accuracy_decimals: 1
    state_class: total_increasing
    filters:
      - multiply: 0.1

  - platform: modbus_controller
    modbus_controller_id: stm32_s1
    name: "S1 Water Flow"
    register_type: holding
    address: 0x0002
    value_type: U_WORD
    unit_of_measurement: "L/min"
    accuracy_decimals: 1
    filters:
      - multiply: 0.1

  - platform: modbus_controller
    modbus_controller_id: stm32_s1
    name: "S1 RS485 RX Errors"
    register_type: holding
    address: 0x0003
    value_type: U_WORD
    unit_of_measurement: "errors"

  - platform: modbus_controller
    modbus_controller_id: stm32_s2
    name: "S2 Water Volume"
    register_type: holding
    address: 0x0000
    value_type: U_DWORD
    register_count: 2
    unit_of_measurement: "L"
    accuracy_decimals: 1
    state_class: total_increasing
    filters:
      - multiply: 0.1

  - platform: modbus_controller
    modbus_controller_id: stm32_s2
    name: "S2 Water Flow"
    register_type: holding
    address: 0x0002
    value_type: U_WORD
    unit_of_measurement: "L/min"
    accuracy_decimals: 1
    filters:
      - multiply: 0.1

  - platform: modbus_controller
    modbus_controller_id: stm32_s2
    name: "S2 RS485 RX Errors"
    register_type: holding
    address: 0x0003
    value_type: U_WORD
    unit_of_measurement: "errors"
```

---

## References
- [STM32L0 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00108281.pdf)
- [YF-B5 datasheet](https://www.hallsensors.de/pdf/YF-B5.pdf)
- [Modbus.org Protocol Docs](https://www.modbus.org/)
