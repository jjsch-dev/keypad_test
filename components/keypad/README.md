# keypad — GTX312L capacitive keypad component (Stage 3)

ESP32-S3 component for the lock's 12-key capacitive touch panel
("TOUCH_BKN001T V2.1"), driven by a GreenChip **GTX312L** over I2C.
All protocol facts are wire-proven with the BP5 PIO sniffer (captures in
`PROJECT_NOTES.md`, Stage 3). OEM behavior is the reference spec (D2).

## Scope

- **Step 1 (bring-up):** I2C master at OEM-parity 37 kHz, self-test, raw
  register access, wire-proven OEM config presets (ACTIVE / IDLE / boot).
- **Step 2 (events):** INT falling-edge ISR + keypad task implementing the
  OEM-mirror state machine (IDLE 330 ms scan → wake → ACTIVE 20 ms polling,
  ~6 s idle timeout), D12 acceptance debounce, FreeRTOS queue with
  `PRESSED`/`RELEASED` events, `keypad_mask_touch()` for the RF window and
  `keypad_prepare_sleep()` for deep-sleep integration (D9).

## Wiring (bench, DevKitC-1)

| Panel (GTX312L) | DevKitC-1 | Kconfig default |
|---|---|---|
| SDA | GPIO8 | `KEYPAD_SDA_GPIO` |
| SCL | GPIO9 | `KEYPAD_SCL_GPIO` |
| INT | GPIO4 (RTC-capable!) | `KEYPAD_INT_GPIO` |
| VCC | 3V3 | confirm level on the panel first |
| GND | GND | common ground is mandatory |

Notes:
- Clock defaults to **37 kHz (OEM parity)**. The lock board uses the weak
  GTX312L *internal* pull-ups; the S3 internal pull-ups are enabled as a
  safety net. Add external 4k7–10k only if you raise the clock.
- INT must be an **RTC-capable GPIO** (0–21 on the S3) so it can wake the
  chip from deep sleep later (touch-gated power policy, D9).

## The self-test (why it looks like this)

The OEM CPU runs two checks at power-on before configuring anything
(captured in `2026-08-06_touch_boot.txt`), and we mirror them:

1. **Read `01h` (CHIP_ID), expect `0xB2`.** The ID byte is
   `1011` (fixed) + `001` (CHIP_ID field) + `0`, which matches a GTX312L
   with its **CTRL pin OPEN** — exactly this board. A wrong value means
   wiring, pull-ups, clock or a different chip: the test fails hard.
2. **Read `20h` (SENSITIVITY1), expect `0x0F` (reset value).** This proves
   the chip is still in its power-on-reset state, i.e. no stale config.
   The chip keeps registers across MCU resets (no power cycle), so after a
   reflash this read may show `0x0A` instead — harmless: the boot sequence
   rewrites the full config anyway. We log a warning and continue.

After the checks, `keypad_boot_config()` replays the OEM power-on writes:
the undocumented magic `0Bh = 0x5B` + `0Bh = 0x00`, then an ACTIVE pass
(sens `0x08`, fast scan) and finally the IDLE pass (sens `0x0A`, 330 ms
sensing), both captured at the lock's boot.

## Console commands (keypad_test)

```
selftest            CHIP_ID + SENSITIVITY1 checks (OEM boot parity)
scan                probe all 7-bit I2C addresses (bench diagnostic)
bootcfg             full OEM power-on sequence (magic + active + idle)
config idle|active  write one OEM config preset
dump                hex dump of registers 01h..2Dh
read <reg>          read one register (hex)
write <reg> <val>   write one register (hex)
monitor [n]         raw TOUCH_OUT + INT view, n polls (default 500 = 10 s)
events [seconds]    run the debounced event engine (default 30 s) + stats
mask on|off         mute/unmute touch (RF card-reader window)
```

Typical bench session:

```
keypad> selftest     # PASS expected
keypad> dump         # 01:B2  20:0F (fresh POR)
keypad> bootcfg      # now the chip is in OEM idle config
keypad> monitor 500  # touch keys: watch masks, ghosts and the INT line
keypad> events 60    # engine on: PRESSED/RELEASED out of the box
```

## Key map (wire-proven, 12/12)

PCB silkscreen **K_n = GTX312L channel n** (bit n-1 of regs 02h/03h):

| Key | PCB | Bit | Key | PCB | Bit |
|---|---|---|---|---|---|
| 1 | K12 | 11 | 7 | K8 | 7 |
| 2 | K11 | 10 | 8 | K9 | 8 |
| 3 | K2 | 1 | 9 | K3 | 2 |
| 4 | K10 | 9 | * | K6 | 5 |
| 5 | K4 | 3 | 0 | K7 | 6 |
| 6 | K1 | 0 | # | K5 | 4 |

## Acceptance rule (D12, wire-proven)

A key press is an **episode** (touch mask != 0). Acceptance requires:

1. Episode length >= `ACCEPT_MIN_POLLS` consecutive polls (default 4 = 80 ms,
   the OEM debounce window measured on the wire).
2. The bit **intersection** across all episode samples, after dropping any
   bit whose **column-neighbor-above** is also set (ghosts always grow
   downward: columns `K12>K10>K8>K6`, `K11>K4>K9>K7`, `K2>K1>K3>K5`),
   leaves exactly one bit.

`PRESSED` fires as soon as an episode qualifies (OEM beep timing);
`RELEASED` fires when the mask returns to 0. This rejects short grazes
(~60 ms no-beep case), transient settling ghosts, and RF noise (multi-bit
or bursty). Single-bit stable RF noise can still pass — that is why
`keypad_mask_touch()` must be engaged while the RF reader polls cards.
