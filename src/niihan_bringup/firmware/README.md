# STM32 control core and ZLAC8015D V4 backend

`niihan_core.c` is portable MCU logic with a nonblocking, acknowledged CANopen SDO scheduler. It implements profile velocity mode from the supplied V4 manual version 1.02, pages 19 and 42–45: asynchronous mode 0x200F, velocity mode 0x6060=3, controlword 0x6040, target speeds 0x60FF:01/02 in integer RPM and encoder positions 0x6064:01/02. Actual speed 0x606C uses 0.1 RPM and must not be interpreted as target-speed units. No EEPROM writes or automatic fault-reset commands are issued.

This is not a flashed firmware image. Integrate the core into an STM32Cube project for **NUCLEO-F446RE**:

1. Add core sources, a 115200-baud UART or USB CDC receive/transmit adapter, CAN controller and a suitable physical CAN transceiver/termination.
2. Implement `send(id,data)` as bounded CAN enqueue and `inhibit(true)` as the commissioned actuator-disable output. The inhibit path must actually work in hardware; the core cannot implement a physical safety circuit.
3. Call `niihan_can_receive` for matching eight-byte standard CAN frames; pass complete serial lines to `niihan_command`.
4. Call `niihan_tick(HAL_GetTick(), physical_estop_asserted)` at least every 5ms, independently of serial reception. Send `niihan_telemetry` at 50Hz.
5. Configure node ID, CAN bitrate, channel directions, encoder counts/revolution and measured chassis dimensions before connecting powered wheels. Default maximum is 30 RPM for bench commissioning, not a field-approved limit.
6. Keep external E-stop/braking independent of the Pi, MCU task and CAN bus. A hardware inhibit is asserted during faults, host-command timeout (250ms), missing CAN responses and physical E-stop indication.

The Pi link is CRC-protected ASCII, **not** the driver's CAN protocol:

- Command: `N1,sequence,left_rad_s,right_rad_s,enabled*CRC16\n`
- Telemetry: `T1,sequence,uptime_ms,left_ticks,right_ticks,estop,fault*CRC16\n`
- CRC: CCITT with initial 0xffff, covering bytes before `*`; four uppercase hexadecimal digits.

Host tests compile the portable core with mocked CAN and inhibit callbacks. They do not validate Cube/HAL wiring, CAN bus timing, braking or powered motors. V4 documentation contains an inconsistent packed-speed subindex description; the implementation uses separate 0x60FF:01 and :02 as illustrated in the velocity-mode routine.

```bash
gcc -std=c11 -Wall -Wextra niihan_core.c test_core.c -lm -o /tmp/niihan-core-test
/tmp/niihan-core-test
```
