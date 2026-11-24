# UART Parallel Data Interface (UART PDI)

Hardware interface for bit-level read and write access to wide parallel registers (`din` / `dout`) over standard 8N1 UART.

## Architecture

- **`uart_pdi`**: Top-level integration module.
  - **`uart_pdi_control`**: FSM handling protocol parsing, wide register sampling (`din`), register updating (`dout`), and data serialization.
  - **`deserializer`**: UART RX to parallel byte receiver with internal FIFO and Ready/Valid interface.
  - **`serializer`**: Parallel byte to UART TX transmitter with internal FIFO and Ready/Valid interface.
  - **`FIFO`**: Synchronous FIFO buffer used by serializer and deserializer.

---

## Protocol Specification

- **Format**: 8N1 UART (1 start bit, 8 data bits LSB-first, no parity, 1 stop bit).
- **Opcodes**: `'R'` (`0x52`), `'W'` (`0x57`), `'F'` (`0x46` delimiter).
- **Addressing**: `idx` (bit index) and `len` (bit count) are 4-byte **big-endian** integers.

### READ Operation
Reads `len` bits from `din` starting at `idx`, transmitted LSB-first within each byte (zero-padded on the last byte if `len` is not a multiple of 8), followed by `'F'`.

```
RX (Input):  | 'R' | idx[3] | idx[2] | idx[1] | idx[0] | len[3] | len[2] | len[1] | len[0] |
TX (Output): | data[0] | data[1] | ... | data[N] | 'F' |
```

### WRITE Operation
Writes `len` bits into `dout` starting at `idx`. Unwritten bits retain their previous values (`dout = 0` on reset). The transaction must end with `'F'`.

```
RX (Input):  | 'W' | idx[3] | idx[2] | idx[1] | idx[0] | len[3] | len[2] | len[1] | len[0] | data[0] | ... | data[N] | 'F' |
```

---

## Simulation (`xrun`)

Execute from the repository root:

### Main Testbench (`uart_pdi_tb`)
```bash
xrun -sv FIFO/FIFO.v serializer/serializer.sv deserializer/deserializer.sv uart_pdi_control.sv uart_pdi.sv uart_pdi_tb.sv
```

### Submodule Testbenches

- **Serializer**:
  ```bash
  xrun -sv FIFO/FIFO.v serializer/serializer.sv serializer/serializer_tb.sv
  ```

- **Deserializer**:
  ```bash
  xrun -sv FIFO/FIFO.v deserializer/deserializer.sv deserializer/deserializer_tb.sv
  ```

- **FIFO**:
  ```bash
  xrun -sv FIFO/FIFO.v FIFO/fifo_tb.v
  ```
