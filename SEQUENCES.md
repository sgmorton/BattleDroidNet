# BattleDroid Sequence Syntax Guide

Sequence files are plain `.txt` files stored on the Master Droid's SD card. They define timed actions for the entire fleet.

## File Format
Lines must follow the timestamp format, followed by a dash and then the command.

```text
MM.SS.cc - COMMAND(target, args...)
```

- **MM**: Minutes (00-99)
- **SS**: Seconds (00-59)
- **cc**: Centiseconds (00-99)

## Targets
Every command must specify a target recipient:
- `1` to `8`: A specific Droid ID.
- `*`: Broadcast to **all** connected droids.

## Commands

### 1. Servo Move: `SM`
Moves a specific servo on the target droid(s).
**Syntax:** `SM(target, servo, position, duration)`

- **servo**: Either `headturn`, `headtilt`, or a numeric index (`0` or `1`).
- **position**: Target percentage (`0` to `100`). `50` is center.
- **duration**: Time to complete the move in milliseconds.

**Example:**
`00.01.50 - SM(2, headturn, 75, 1000)`  
*(At 1.5s, Droid 2 turns head to 75% over 1 second)*

---

### 2. Play Audio: `PA`
Triggers audio playback on the target droid(s).
**Syntax:** `PA(target, file_prefix)`

- **file_prefix**: The base name of the MP3 file on the droid's local SD card.
    - If target is `*`, each droid looks for `prefix_X.mp3` (where X is its ID).
    - If target is a number, it looks for `prefix.mp3`.

**Example:**
`00.05.00 - PA(*, intro)`  
*(At 5s, all droids play their respective "intro" files)*

---

### 3. Talk: `TK`
Triggers the randomized head-tilt "talking" animation.
**Syntax:** `TK(target, duration)`

- **duration**: How long the animation should last in milliseconds.

**Example:**
`00.10.00 - TK(2, 3000)`  
*(At 10s, Droid 2 starts talking for 3 seconds)*

---

### 4. Reset: `RS`
Returns all servos to their neutral position (50%).
**Syntax:** `RS(target)`

**Example:**
`00.12.00 - RS(*)`  
*(At 12s, all droids reset to center)*

## Full Example Sequence (`dance.txt`)
```text
00.00.00 - RS(*)
00.01.00 - PA(*, groove)
00.01.50 - SM(1, headturn, 25, 500)
00.01.50 - SM(2, headturn, 75, 500)
00.02.00 - TK(*, 2000)
00.05.00 - RS(*)
```
