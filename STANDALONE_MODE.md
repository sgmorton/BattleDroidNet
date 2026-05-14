# BattleDroid Standalone Mode Documentation

Standalone Mode allows a BattleDroid to operate entirely independently of the fleet network. In this mode, the droid will autonomously perform random sequences at configurable intervals.

---

## Configuration: `standalone.ini`

Create a file named `standalone.ini` in the root of your SD card to configure Standalone Mode.

| Setting | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `Sound` | Boolean | `True` | If `False`, the droid will skip all audio playback during autonomous sequences. |
| `Movement` | Boolean | `True` | If `False`, all servo commands will be ignored; the droid stays still. |
| `MinDelay` | Integer | `5` | Minimum wait time (**seconds**) between sequences. Actual wait is `MinDelay` + a random offset up to `MinDelay` to keep behavior unpredictable. |

### Example `standalone.ini`
```ini
Sound=True
Movement=True
MinDelay=10
```

---

## Provisioning: `/standalone` Folder

When the droid switches into Standalone Mode it automatically provisions the SD card:

1. All files in the root **except** `settings.ini` and `standalone.ini` are deleted.
2. All files from the `/standalone` folder are copied to the root.
3. The droid reboots its sequence scanner against the freshly provisioned root.

This lets you keep a dedicated set of autonomous sequences and audio files that are completely separate from your fleet files.

---

## File Naming Requirements

> These rules apply to **all** files used in Standalone Mode.

### Sequence Files (`.seq`)
| Rule | Detail |
| :--- | :--- |
| Extension | Must end in `.seq` (lowercase) |
| Name length | Up to 20 characters before the extension |
| Characters | Letters, numbers, hyphens, underscores only — **no spaces** |
| Location | Root of SD card (after provisioning copies them from `/standalone`) |

**Valid examples:** `intro.seq`, `greeting-01.seq`, `patrol_loop.seq`  
**Invalid examples:** `my sequence.seq` ❌ (space), `verylongsequencenamethatexceedslimit.seq` ❌ (too long)

### WAV Audio Files (`.wav`)
| Rule | Detail |
| :--- | :--- |
| Extension | `.wav` or `.WAV` — `.mp3` is **not** supported |
| Name length | **Maximum 15 characters** including the `.wav` extension |
| Characters | Letters, numbers, hyphens, underscores only — **no spaces** |
| Format | PCM, 16-bit, any sample rate (8kHz – 44.1kHz), mono or stereo |
| Location | Root of SD card |

**Valid examples:** `roger.wav`, `lovetohelp.wav`, `BD1.wav`  
**Invalid examples:** `Love To Help You.wav` ❌ (spaces + too long), `verylongname.wav` ❌ (too long)

> **Why 15 characters?** The ESP-NOW radio packet format reserves exactly 15 bytes for the filename field. Longer names will be silently truncated, causing `WAV NOT FOUND` errors on the receiving droid.

### Default Audio File
If a `PA` command specifies no filename, the droid plays `BD<ID>.wav` (e.g., `BD1.wav` for Droid 1). Place this file in the root as a fallback sound.

---

## Sequence File Format

Sequence files are plain text files where each line defines a timed action.

```
MM.SS.cc - COMMAND(target, args...)
```

| Field | Description |
| :--- | :--- |
| `MM` | Minutes (00–99) |
| `SS` | Seconds (00–59) |
| `cc` | Centiseconds (00–99) |
| `target` | `*` = all droids, `1`–`8` = specific droid ID |

A timestamp of `00.00.00` means **execute immediately** when that line is reached.

---

## Sequence Commands

### `RS` — Reset Servos
Returns all servos on the target droid(s) to the neutral center position (50%).

```
RS(target)
```

**Example:**
```
00.00.00 - RS(*)
```
*Immediately reset all servos to center.*

---

### `SM` — Servo Move
Moves a named servo to a target position over a given duration.

```
SM(target, servo, position, duration)
```

| Argument | Values | Description |
| :--- | :--- | :--- |
| `servo` | `headturn`, `headtilt`, `torsoturn` | Which servo to move |
| `position` | `0` – `100` | Target position. `50` = center, `0` = full left/down, `100` = full right/up |
| `duration` | milliseconds | Time to sweep to the target position |

**Examples:**
```
00.01.00 - SM(*, headturn, 25, 800)
00.01.50 - SM(1, headtilt, 70, 500)
00.02.00 - SM(*, torsoturn, 75, 1200)
```

---

### `PA` — Play Audio
Plays a `.wav` file from the droid's local SD card.

```
PA(target, filename)
```

| Argument | Values | Description |
| :--- | :--- | :--- |
| `filename` | WAV filename (max 15 chars with extension) | The `.wav` extension is added automatically if omitted |

**Examples:**
```
00.00.50 - PA(*, roger.wav)
00.03.00 - PA(1, lovetohelp.wav)
00.05.00 - PA(*)
```
*The last example with no filename plays the droid's default `BD<ID>.wav` file.*

---

### `T` — Talk Animation
Triggers the randomized head-jitter "talking" animation for a set duration. Use this alongside a `PA` command to make the droid look like it is speaking.

```
T(target, duration)
```

| Argument | Values | Description |
| :--- | :--- | :--- |
| `duration` | milliseconds | How long the talking animation lasts |

**Example:**
```
00.00.50 - T(*, 4500)
```
*Trigger talking animation for 4.5 seconds — pair this with a PA command of similar length.*

---

## Master Sample Sequence

This example demonstrates every available command. Save it as `demo.seq` in your `/standalone` folder.

```
00.00.00 - RS(*)
00.00.20 - SM(*, headturn, 25, 800)
00.00.20 - SM(*, torsoturn, 30, 1000)
00.00.50 - PA(*, lovetohelp.wav)
00.00.50 - T(*, 5500)
00.06.50 - SM(*, headturn, 75, 800)
00.06.50 - SM(*, torsoturn, 70, 1000)
00.07.50 - PA(*, roger.wav)
00.07.50 - T(*, 2000)
00.10.00 - SM(*, headtilt, 70, 600)
00.10.00 - SM(*, headturn, 50, 1000)
00.11.00 - PA(*)
00.11.00 - T(*, 3000)
00.14.50 - SM(*, headtilt, 30, 600)
00.15.00 - SM(*, torsoturn, 50, 1500)
00.16.50 - RS(*)
```

**What this sequence does, step by step:**

| Timestamp | Action |
| :--- | :--- |
| `0.0s` | All droids reset to center |
| `0.2s` | Head turns left (25%), torso turns left (30%) |
| `0.5s` | Plays `lovetohelp.wav` + starts 5.5s talk animation |
| `6.5s` | Head turns right (75%), torso turns right (70%) |
| `7.5s` | Plays `roger.wav` + starts 2s talk animation |
| `10.0s` | Head tilts up (70%), head turns back to center |
| `11.0s` | Plays default `BD<ID>.wav` + starts 3s talk animation |
| `14.5s` | Head tilts down (30%) |
| `15.0s` | Torso sweeps back to center over 1.5 seconds |
| `16.5s` | Full reset to center — sequence ends |

---

## Autonomous Behavior

1. **Scan**: The droid scans the SD root for all `.seq` files (up to 16 found).
2. **Pick**: One is selected at random.
3. **Execute**: The sequence runs. The OLED top line shows the sequence name during playback.
4. **Wait**: After the sequence **and** all audio finishes, a countdown timer appears on the OLED (`NEXT: Xs`). The wait is `MinDelay` + a random offset up to `MinDelay` seconds.
5. **Repeat**: A new random sequence is selected and the cycle continues indefinitely.

---

## How to Enable

| Method | Steps |
| :--- | :--- |
| **Web Controller** | Change the mode selector to **STAND ALONE** |
| **On-Board Menu** | Use navigation buttons to select **STAND ALONE** from the Mode menu |
| **Serial Command** | Send `CMD:STANDALONE` via the serial console |

> The droid will immediately provision the SD card from the `/standalone` folder when the mode is switched — no restart required.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
| :--- | :--- | :--- |
| `WAV NOT FOUND` on OLED | Filename too long, has spaces, or file missing | Check 15-char limit, no spaces, file is in SD root |
| Audio cuts off early | File format not 16-bit PCM WAV | Re-export audio as **PCM WAV, 16-bit** |
| No sequences run | No `.seq` files in root | Ensure `/standalone` folder has `.seq` files and provisioning ran |
| Sequence runs but no sound | `Sound=False` in `standalone.ini` | Set `Sound=True` |
| Servos don't move | `Movement=False` in `standalone.ini` | Set `Movement=True` |
