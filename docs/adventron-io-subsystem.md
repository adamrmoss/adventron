# Adventron I/O Subsystem — RP2040 SPI Hub and Peripheral Mapping

## Connected Peripherals Overview

The RP2040 provides a unified SPI hub for the following devices:
- **FT813** display controller (video control, asset uploads)
- **YMF825** FM synthesizer (audio registers)
- **DS3234** real-time clock (timekeeping, alarms)
- **microSD** card (block storage)
- **MCP3208** 12‑bit SPI ADC (paddles/analog inputs)

Direct to W65C265S GPIO:
- **Two Atari‑style joystick ports** (digital inputs)
- **Eight front‑panel toggle switches** (configuration inputs)
- **UART1** 6‑pin header (expansion); **UART0** reserved for RP2040 control

---

## 1. Memory Map and Decode (Bank $00)

- **CS0B**: **$00:DF00–$00:DF1F** — RP2040 SPI Hub Register Window (byte‑wide on D[7:0])
- **UART0**: Hardware serial link CPU↔RP2040 for command and status

CS0B is used for high‑throughput FIFO moves; UART0 carries structured control packets.

### 1.2 External Chip Selects
| Signal | Default Range | Typical Use |
|---------|----------------|--------------|
| CS0B | $00:DF00–$00:DF1F | RP2040 register window (32 B I/O) |
| CS1B | $00:DF20–$00:DF3F | General I/O or peripheral expansion |
| CS2B | $00:DF40–$00:DF5F | General I/O or peripheral expansion |
| CS3B | $00:DF60–$00:DF7F | Optional SRAM or small peripheral |
| CS4B | $00:DF80–$00:DF9F | Boot/utility ROM or NVRAM |
| CS5B | $00:000000–$00:3FFFFF | 4 MB main SRAM |
| CS6B | $00:400000–$00:BFFFFF | 8 MB removable ROM/Flash cartridge |
| CS7B | $00:C00000–$00:FFFFFF | 4 MB system ROM/Flash |

### 1.1 Built‑in I/O Ports‑in I/O Ports
The W65C265S provides four 8‑bit general‑purpose ports (PORTA–PORTD) memory‑mapped into its internal register space at:
- **PORTA Data:** $00:DF60  
- **PORTB Data:** $00:DF61  
- **PORTC Data:** $00:DF62  
- **PORTD Data:** $00:DF63  

Each has corresponding Data Direction Registers at $00:DF70–$00:DF73 and Control Registers at $00:DF80–$00:DF83. These ports remain independent of CS0B–CS7B and are used for direct digital I/O, such as joysticks and toggle switches.

---

## 2. W65C265S ↔ RP2040 Interface

### 2.1 Signals & Levels
- **UART0_TX/ RX** (CPU 5 V ↔ RP 3.3 V): CPU‑TX → divider (10 kΩ/20 kΩ) → RP‑RX; RP‑TX (3.3 V) → CPU‑RX (5 V VIH met)
- **CS0B window**: `/CS0B, /RD, /WR, A[4:0] (internal), D[7:0]` via **74LVC16T245** (bidirectional)
- **/IRQ0**: RP2040 → CPU GPIO (level‑active high)

### 2.2 CS0B Register Window ($00:DF00 + off)
| Off | Name   | Dir | Description |
|-----|--------|-----|-------------|
| 0x00| **DATA**   | R/W | 8‑bit FIFO port (streaming payload) |
| 0x01| **STATUS** | R   | [7]=IRQ [6]=BUSY [5]=RX_TH [4]=TX_TH [3]=DONE [2]=ERR [1:0]=STATE |
| 0x02| **CMD**    | W   | Trigger for active operation (set by UART command) |
| 0x03| **CS_SEL** | W   | Per‑device CS bitmap (firmware may override by UART) |
| 0x04–0x1F| — | — | Reserved |

### 2.3 UART0 Command Packets
Fixed framing, binary; little‑endian lengths.
```
[SOF=0xA5] [OP] [LEN_L] [LEN_H] [PAYLOAD ...] [CRC8]
```
**OP codes**
- 0x10 SD_READ   : PAYLOAD = LBA[3:0], COUNT (blocks)
- 0x11 SD_WRITE  : PAYLOAD = LBA[3:0], COUNT
- 0x20 FT_WR     : PAYLOAD = ADDR[2:0], LEN[1:0]  (asset upload follows via CS0B DATA)
- 0x21 FT_RD     : PAYLOAD = ADDR[2:0], LEN[1:0]
- 0x30 RTC_RW    : PAYLOAD = REG, N, DIR (0=read,1=write), data if write
- 0x40 YMF_WR    : PAYLOAD = REG, N, data
- 0x50 ADC_READ  : PAYLOAD = CHAN_MASK (bits 0..7); returns 2*N bytes (12‑bit right‑justified)
- 0x60 HUB_CFG   : PAYLOAD = SPI_MODE, CLKDIV, IRQ masks
- 0x7F HUB_PING  : PAYLOAD = none (echo)

**Completion:** RP2040 returns an ACK packet with **RESULT** code; bulk data is exchanged via CS0B DATA FIFO when directed by OP.

---

## 3. UART Interfaces

### 3.1 W65C265S UART Allocation
- **UART0** — Dedicated internal link to the RP2040 SPI hub (3.3 V TTL).
- **UART1** — Reserved for external debugging and USB‑C OTG serial interface. The USB‑C connector integrates 5 V power input and optional USB‑to‑serial function using a **CP2102N** or **CH340E** bridge. TX/RX from UART1 connect through this bridge to the USB‑C data pins for direct PC communication.

The USB‑C port therefore provides both **power (5 V input)** and **debug serial (UART1)** on a single connector.

## 4. RP2040 GPIO Mapping

### 3.1 Shared SPI (SPI0)
- **SCK**  = GPIO10
- **MOSI** = GPIO11
- **MISO** = GPIO12

### 3.2 Chip‑Selects and Interrupts
| Device  | CS GPIO | Other pins |
|---------|---------|------------|
| microSD | GPIO2   | CD=GPIO3, WP=GPIO4 |
| FT813   | GPIO5   | INT=GPIO6 |
| DS3234  | GPIO7   | INT=GPIO8 |
| YMF825  | GPIO9   | RESET=GPIO13 |
| MCP3208 | GPIO14  | — |

(Keep SCK/MOSI/MISO short trunk; add 22 Ω series on SCK/MOSI near RP2040.)

---

## 4. Per‑Device Wiring & Parameters

### 4.1 microSD (SPI mode)
- Connector: Molex **503398‑1892**
- Lines: CS(GPIO2), SCK(GPIO10), MOSI(GPIO11), MISO(GPIO12), CD(GPIO3), WP(GPIO4)
- Mode 0, **25 MHz** nominal after init; 3.3 V; 100 nF + 10 µF local

### 4.2 FT813 (SPI)
- CS(GPIO5), INT(GPIO6) → RP; SCK/MOSI/MISO shared
- Mode 0, **24–30 MHz** asset bursts; use HOLD_CS for long streams

### 4.3 DS3234 (SPI)
- CS(GPIO7), INT(GPIO8)
- Mode 3, **≤20 MHz**; battery‑backed; 0.1 µF + 10 µF near VCC

### 4.4 YMF825 (SPI)
- CS(GPIO9), RESET(GPIO13)
- Mode 0, **≤4 MHz**; analog out to amp as per system doc

### 4.5 MCP3208 (12‑bit SPI ADC)
- CS(GPIO14), SCK/MOSI/MISO shared
- Mode 0, **≤1.6 MHz** SCLK typical; 3.3 V
- **Channels**: CH0..CH3 = Paddles Port A(0..1) & Port B(0..1); CH4..CH7 = spare/expansion
- 1 kΩ series + 10 nF to ground per channel as anti‑alias RC (optional)

---

## 5. Joysticks and Toggle Switches (Direct to CPU)

### 5.1 DB9 Joystick Ports (Atari wiring)
- **Pins**: 1=Up, 2=Down, 3=Left, 4=Right, 6=Fire, 7=+5 V, 8=GND (others NC)
- **Protection**: 220 Ω series per signal; 10 kΩ pull‑ups to **3.3 V** at CPU input pins
- **Read**: CPU polls port bits each VBlank (or timer IRQ @ 1 kHz); no debouncing required for direction lines; optional 2 ms debounce on Fire

### 5.2 Front‑Panel Toggle Switches (8×)
- SPDT to ground; 10 kΩ pull‑ups to 3.3 V; 100 nF to ground near header
- Mapped to CPU GPIO; polled at 100 Hz in a timer ISR; 5‑sample majority debounce

---

## 6. Firmware Responsibilities

### 6.1 RP2040 Hub (Core 0 = bus/IRQ, Core 1 = SPI engines)
- UART packet parser (SOF/CRC8)
- Command dispatcher (OP table above)
- SPI drivers: SD (25 MHz), FT813 (30 MHz burst), DS3234 (20 MHz), YMF825 (4 MHz), MCP3208 (≤1.6 MHz)
- CS0B FIFO engine: move data between SPI and CS0B DATA; generate /IRQ0 on RX_TH/DONE/ERR
- Error handling: timeouts, CRC, under/overflow → RESULT codes; clear BUSY/DONE bits

**Pseudocode sketch**
```
while (1) {
  if (uart_pkt_available()) {
    parse(); set_BUSY();
    switch(OP) {
      case SD_READ:  sd_read_blocks(LBA, n, fifo_out); break;
      case SD_WRITE: sd_write_blocks(LBA, n, fifo_in);  break;
      case FT_WR:    ft_write(addr, len, fifo_in);      break;
      case FT_RD:    ft_read(addr, len, fifo_out);      break;
      case RTC_RW:   rtc_xfer(...);                     break;
      case YMF_WR:   ymf_write_burst(...);              break;
      case ADC_READ: adc_scan(mask, fifo_out);          break;
      default:       result = ERR_BADOP;
    }
    set_DONE(); raise_IRQ();
  }
  service_fifo();
}
```

### 6.2 W65C265S Side
- **Select device** by sending UART packet (OP + parameters)
- **For bulk**: wait for IRQ (STATUS.DONE or RX_TH/TX_TH), then stream via CS0B: write to DATA for device writes; read from DATA for device reads
- **Polling**: fallback by reading STATUS when IRQs are masked
- **Examples**
  - SD 512‑byte read: send `OP=SD_READ(LBA,1)` → on IRQ, read 512 bytes from DATA
  - FT asset upload: send `OP=FT_WR(addr,len)` → write `len` bytes to DATA as TX_SPACE permits
  - ADC read: send `OP=ADC_READ(mask)` → read 2*N bytes from DATA

---

## 7. Electrical & Timing Notes
- Maintain short, impedance‑controlled SPI trunk; place RP2040 near connectors
- Series 22 Ω on SCK/MOSI; optional on MISO if ringing observed
- Keep CS0B traces short; place level shifter adjacent to RP2040
- /IRQ0 routed with pull‑down; de‑glitch in firmware

---

## 8. I/O Subsystem BoM (Delta)

| Ref | Component           | Example Part          | Notes |
|-----|---------------------|-----------------------|-------|
| U‑IO | RP2040              | RP2040‑QFN56          | SPI hub MCU |
| U‑LS | Level shifter       | 74LVC16T245           | CS0B bidirectional |
| X‑12 | 12 MHz XO           | ECS‑120‑18‑33Q        | RP2040 clock |
| J‑SD | microSD connector   | Molex 503398‑1892     | Card detect & WP |
| U‑RTC| RTC                 | DS3234S#              | SPI RTC |
| U‑FT | Display controller  | FT813Q                | SPI/QSPI video IC |
| U‑YMF| Audio synthesizer   | YMF825                | SPI audio |
| U‑ADC| ADC                 | MCP3208‑CI/SL         | 8‑ch, 12‑bit, SPI |
| ESD  | ESD protection      | USBLC6‑2SC6 (×2)      | SD/EXT lines |

---

## 9. Summary
The I/O subsystem uses the RP2040 as an SPI hub at 3.3 V, commanded by the W65C265S over UART0, with high‑rate data moved through a CS0B FIFO window. All serial devices standardize on SPI, including the **MCP3208** 12‑bit ADC. Joysticks and switches remain direct to the CPU for lowest latency.

