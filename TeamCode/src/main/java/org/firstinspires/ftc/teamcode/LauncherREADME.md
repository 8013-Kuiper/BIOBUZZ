# Ball Launcher PID – Setup, Tuning & Usage Guide

Velocity control for a **6000 RPM flywheel motor** (e.g. goBILDA 5203 Yellow Jacket 1:1) on a **REV Control Hub**, with live tuning and graphs in **FTC Dashboard**.

| File | What it is |
|---|---|
| `LauncherPID.java` | The controller. Reads the encoder, runs PID + feedforward, sets motor power. Not an OpMode – use it in any TeleOp or Auto. |
| `LauncherPIDTest.java` | A TeleOp ("Launcher PID Test") for testing and tuning: gamepad controls, Driver Station telemetry, Dashboard graphs. |

---

## 1. Setup

### 1.1 Add FTC Dashboard to the project

Open `build.dependencies.gradle` and add:

```gradle
repositories {
    maven { url = 'https://maven.brott.dev/' }
}

dependencies {
    implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'
}
```

Click **Sync Now** in Android Studio.

### 1.2 Add the files

Copy both `.java` files into:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

If your package is different, change the `package` line at the top of both files.

### 1.3 Configure the robot

1. On the Driver Station, open **Configure Robot**.
2. Add the flywheel motor on whichever Control Hub motor port it's plugged into.
3. Name it **`launcher`** (or change `MOTOR_NAME` in `LauncherPID.java`).
4. Make sure the motor's **encoder cable** is plugged into the encoder port with the **same number** as the motor port.

### 1.4 Check the motor constants

At the top of `LauncherPID.java`:

| Constant | Default | Change if… |
|---|---|---|
| `TICKS_PER_REV` | `28.0` | You use a different motor. 28 is correct for the goBILDA 6000 RPM (1:1). |
| `MAX_RPM` | `6000.0` | You want to cap the highest target the code will accept. |
| `MOTOR_REVERSED` | `false` | "Forward" spins the flywheel the wrong way. |

### 1.5 Open FTC Dashboard

1. Connect your laptop to the **Control Hub's Wi-Fi**.
2. Open a browser to **http://192.168.43.1:8080/dash**
3. You should see an OpMode selector, a **Config** panel, a **Telemetry** panel and a **Graph** panel. You can start/stop OpModes from here too.

---

## 2. Running the test OpMode

Select **Launcher PID Test** on the Driver Station (or Dashboard), press **Init**, then **Start**.

### Controls (gamepad 1)

| Button | Action |
|---|---|
| **A** | Launcher ON / OFF |
| **B** | Reverse direction (ramps down through zero, then back up) |
| **Dpad Up / Down** | Target +/- 100 RPM |
| **Right / Left Bumper** | Target +/- 500 RPM |
| **X** | Jump to preset (4500 RPM) |

Step sizes and the preset are in the Dashboard Config panel under **LauncherPIDTest**.

### What the telemetry means

| Name | Meaning |
|---|---|
| **Target RPM** | The speed you asked for (negative = reverse). |
| **Ramped Target RPM** | What the controller is actually chasing right now. It climbs toward Target at `RAMP_RPM_PER_SEC`. |
| **Real RPM** | Measured flywheel speed (filtered). |
| **Error RPM** | Ramped Target − Real. You want this near 0. |
| **Power** | Motor power being sent, −1 to 1. If this sits at 1.0 the motor is maxed out. |
| **At Speed** | YES when the flywheel is within `AT_SPEED_TOLERANCE_RPM` of the target. |
| **Battery** | Battery voltage. |
| **Loop** | Time per loop in ms. Lower = smoother control (aim for under ~20 ms). |

---

## 3. How the controller works (short version)

Every loop the controller calculates motor power as:

```
power = feedforward + PID

feedforward = (kV × target + kS) × (12 V / battery voltage)
PID         = kP × error  +  kI × (sum of error over time)  −  kD × (how fast RPM is changing)
```

- **Feedforward (kV, kS)** is the controller's *guess* of the power needed for a given speed. A good feedforward does about 90% of the work by itself.
- **kP** pushes harder the further you are from the target. It fixes what the feedforward gets wrong and recovers speed after a ball is shot.
- **kI** slowly removes any small error that's left over. Usually not needed if kV is right.
- **kD** dampens overshoot. Usually not needed for a flywheel.
- **Voltage compensation** scales the feedforward so the launcher shoots the same at 13.5 V as at 12 V.

---

## 4. Tuning tutorial

You'll tune in this order: **kS → kV → kP → (kD) → (kI)**. All values live in the Dashboard **Config** panel under **LauncherPID**. Changes apply instantly while the OpMode runs.

### Before you start

- Use a **charged battery** (12.5 V+).
- Make sure nothing can get caught in the flywheel. Keep hands clear.
- Do **not** load balls yet – tune the empty flywheel first.
- In the Dashboard **Graph** panel, add these lines: **`rampedTargetRPM`**, **`actualRPM`**. Later also **`power`** and **`error`**.

### Step 1 – Zero out the PID

In Config → LauncherPID set:

```
kP = 0
kI = 0
kD = 0
```

Now only the feedforward is driving the motor.

### Step 2 – Find kS (static friction)

1. Temporarily set **`kV = 0`**.
2. Start the OpMode, press **A** to turn the launcher on (any target above 0 works).
3. Starting from `kS = 0`, raise **kS** in small steps (0.01) until the flywheel **just barely starts to turn**.
4. Back off slightly so it's right at the edge. Typical values: **0.02 – 0.08**.

### Step 3 – Find kV (power per RPM)

1. Set **`kV = 0.000167`** (that's 1 ÷ 6000, the starting guess).
2. Set the target to about **3000 RPM** and turn the launcher on. Wait for it to settle.
3. Look at the graph:
   - **actualRPM settles below the target** → increase kV a little.
   - **actualRPM settles above the target** → decrease kV a little.
4. Change kV in small steps (e.g. 0.000005) until actual ≈ target.
5. **Check at other speeds** – try 2000 and 4500 RPM. If it's close at all of them, kV is good. If it's right at one speed but off at another, adjust kS slightly and repeat.

> **Shortcut:** at steady speed, `kV ≈ (power − kS) ÷ RPM`. Read `power` and `actualRPM` from telemetry and do the math.

When kV and kS are good, the flywheel should already hold speed within ~100–200 RPM with **no PID at all**.

### Step 4 – Tune kP (recovery speed)

1. Set **`RAMP_RPM_PER_SEC = 0`** so target changes are instant "steps". This makes the graph easy to read.
2. Set **`kP = 0.0002`**.
3. Toggle between two speeds using the bumpers (e.g. 3000 ↔ 3500 RPM) and watch the graph:

| What you see | What to do |
|---|---|
| Slow, lazy climb to the target | Increase kP |
| Overshoots then settles | kP is about right – maybe lower slightly |
| Keeps oscillating / sounds like it's "hunting" | kP too high – lower it |
| `power` slams between −1 and 1 | kP way too high |

4. Typical working range: **0.0002 – 0.002**.
5. **Real-world test:** once balls are allowed, shoot one and watch `actualRPM` drop and recover. A good kP gets back to speed quickly without overshooting much.

### Step 5 – (Optional) kD

Only if you still get overshoot you can't fix by lowering kP:

- Start at **`kD = 0.00001`** and increase slowly.
- If the graph or motor gets noisy/jittery, kD is too high – D amplifies encoder noise.

### Step 6 – (Optional) kI

Only if there's a small steady error left that kV can't remove (e.g. always 30 RPM low):

- Start at **`kI = 0.0001`**.
- `I_ZONE_RPM` (default 400) means the integral only builds up when you're already close, so it won't wind up during spin-up.
- `I_LIMIT` (default 0.25) caps how much power the integral can add.
- If it overshoots after a shot and slowly drifts back, kI is too high.

### Step 7 – Filter and ramp

- **`FILTER_ALPHA`** (default 0.3) smooths the RPM reading. If `rawRPM` on the graph is very jumpy, try 0.5. Higher = smoother but slower to react. Don't go above ~0.7.
- **`RAMP_RPM_PER_SEC`** – set it back to something gentle for matches (e.g. **6000** = 0 → full speed in about 1 second). This protects the motor and reduces current spikes when you start up or reverse.

### Step 8 – SAVE YOUR VALUES

> ⚠️ **Dashboard changes are lost when the robot restarts or the code is redeployed.**

Copy your final numbers from the Config panel into the `public static` fields at the top of `LauncherPID.java`, then redeploy.

### Finding your real top speed

A 6000 RPM motor will **not** reach 6000 RPM with a flywheel on it. To find the realistic max:

1. Set the target to 6000 and turn it on.
2. Watch `power`. When it sits at **1.0**, the motor is maxed out – read `actualRPM`. That's your top speed at that battery voltage.
3. Pick shooting speeds **10–15% below** that, so the controller has headroom to recover after each shot.

---

## 5. Using `LauncherPID` in your own TeleOp

```java
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Recommended: bulk reads for a faster loop
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        LauncherPID launcher = new LauncherPID(hardwareMap);
        Servo feeder = hardwareMap.get(Servo.class, "feeder");

        launcher.setTargetRPM(4500);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Spin up while right trigger is held
            launcher.setEnabled(gamepad2.right_trigger > 0.5);

            // Only feed a ball when the flywheel is at speed
            if (gamepad2.a && launcher.isAtSpeed()) {
                feeder.setPosition(1.0);
            } else {
                feeder.setPosition(0.0);
            }

            // ... drive code etc ...

            launcher.update();   // REQUIRED – call once every loop

            telemetry.addData("Launcher RPM", "%.0f", launcher.getActualRPM());
            telemetry.addData("Ready", launcher.isAtSpeed());
            telemetry.update();
        }

        launcher.stop();
    }
}
```

> If you add bulk caching here, make sure you do **not** also have it set up somewhere else in a way that never calls `clearBulkCache()` – stale cached readings will break the PID.

### Method reference

| Method | What it does |
|---|---|
| `update()` | **Call once every loop.** Reads the encoder, runs the PID, sets power. |
| `setTargetRPM(rpm)` | Set target speed (0 to `MAX_RPM`). |
| `changeTargetRPM(delta)` | Add/subtract from the target. |
| `setEnabled(bool)` / `toggleEnabled()` | Turn the launcher on/off. Off = ramps down then coasts. |
| `setDirection(+1 / -1)` / `reverseDirection()` | Set or flip spin direction. |
| `stop()` | Cut power immediately and reset. |
| `isAtSpeed()` | `true` when on, done ramping, and within `AT_SPEED_TOLERANCE_RPM`. |
| `getActualRPM()` | Filtered measured RPM (signed). |
| `getTargetRPM()` | Requested RPM (signed; 0 when off). |
| `getRampedTargetRPM()` | What the controller is chasing right now. |
| `getErrorRPM()`, `getPower()`, `getVoltage()`, `getLoopTimeMs()` | Debug info. |

---

## 6. Parameter reference (`LauncherPID`)

| Parameter | Default | Description |
|---|---|---|
| `MOTOR_NAME` | `"launcher"` | Name in the robot configuration. |
| `MOTOR_REVERSED` | `false` | Flip motor direction. |
| `TICKS_PER_REV` | `28.0` | Encoder ticks per output revolution. |
| `MAX_RPM` | `6000.0` | Highest allowed target. |
| `kV` | `1/6000` | Feedforward power per RPM (at 12 V). |
| `kS` | `0.03` | Feedforward power to beat friction. |
| `kP` | `0.0004` | Proportional gain (power per RPM of error). |
| `kI` | `0.0` | Integral gain. |
| `kD` | `0.0` | Derivative gain. |
| `I_LIMIT` | `0.25` | Max power from the integral term. |
| `I_ZONE_RPM` | `400` | Only integrate when error is below this (0 = always). |
| `RAMP_RPM_PER_SEC` | `6000` | How fast the target can change (0 = instant). |
| `FILTER_ALPHA` | `0.3` | RPM smoothing (0 = none, higher = smoother). |
| `VOLTAGE_COMP` | `true` | Scale feedforward with battery voltage. |
| `NOMINAL_VOLTAGE` | `12.0` | Voltage that kV/kS are tuned for. |
| `AT_SPEED_TOLERANCE_RPM` | `100` | How close counts as "at speed". |

---

## 7. Troubleshooting

| Problem | Likely cause / fix |
|---|---|
| **Real RPM always 0** | Encoder cable unplugged, or plugged into a different port number than the motor. |
| **Flywheel spins the wrong way** | Set `MOTOR_REVERSED = true`. |
| **Real RPM is negative while spinning forward / motor runs away to full power** | Encoder direction doesn't match motor direction (rare with goBILDA motors – check wiring or an external encoder). Stop immediately. |
| **Power stuck at 1.0, can't reach target** | Target is above the real top speed (see "Finding your real top speed"), or the battery is low. |
| **RPM reading very jumpy** | Raise `FILTER_ALPHA` (try 0.5). Check for a loose encoder cable. |
| **Oscillates / "hunts" around the target** | kP too high, or kD too high. Lower them. |
| **Slow to recover after a shot** | kP too low, or kV too low. |
| **Overshoots after every shot, then drifts back** | kI too high. Lower it or set to 0. |
| **Shoots differently on fresh vs. tired battery** | Make sure `VOLTAGE_COMP = true` and kV was tuned correctly. |
| **Dashboard page won't load** | Laptop not on the Control Hub Wi-Fi, or wrong address. Use `http://192.168.43.1:8080/dash`. Dashboard can also be toggled off from the Driver Station menu – make sure it's enabled. |
| **"LauncherPID" missing from Config panel** | Fields must be `public static` and **not** `final`. Redeploy, then refresh the browser page. |
| **Tuned values disappeared** | Dashboard edits aren't saved. Copy them into `LauncherPID.java` (Step 8). |
| **Loop time is high (> 25 ms)** | Make sure bulk caching is on, and avoid `sleep()` or heavy code (like vision processing) in the same loop. |