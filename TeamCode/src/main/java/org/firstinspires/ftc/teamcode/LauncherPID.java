package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Ball launcher flywheel – custom PID + feedforward velocity controller
 * for a 6000 RPM motor (e.g. goBILDA 5203 Yellow Jacket 1:1, 28 ticks/rev).
 *
 * Reusable: create it in any OpMode, call update() once every loop.
 *
 * LauncherPID launcher = new LauncherPID(hardwareMap);
 * launcher.setTargetRPM(4500);
 * launcher.setEnabled(true);
 * while (opModeIsActive()) { launcher.update(); ... }
 *
 * DIRECTION
 * Direction is changed with DcMotor.setDirection(), which flips BOTH the
 * power and the encoder reading. Because of that, every RPM value in this
 * class is positive in whatever direction the motor is currently set to.
 * Flipping direction while the wheel is spinning would make the measured
 * RPM instantly negative and the PID would slam full power the other way,
 * so a direction change is queued: the wheel ramps down and coasts until
 * it is below REVERSE_SAFE_RPM, then setDirection() is called and the wheel
 * spins back up.
 *
 * CONTROL LAW
 * power = (kV * target + kS) * (12V / batteryV) <- feedforward
 * + kP * error + kI * integral(error) - kD * d(actual)/dt <- PID
 * (D acts on the measurement, so a target change doesn't cause a "kick".)
 *
 * All "public static" fields are live-tunable in FTC Dashboard's Config tab
 * under "LauncherPID". Copy final values back into this file – dashboard
 * edits are lost when the robot restarts.
 */
@Config
public class LauncherPID {

    // ---------------- Hardware ----------------
    public static String MOTOR_NAME = "launcher"; // name in the robot configuration
    public static boolean MOTOR_REVERSED = false; // flip if FORWARD spins the wrong way (applied on next direction change)
    public static double TICKS_PER_REV = 28.0; // 6000 RPM motor encoder at output shaft
    public static double MAX_RPM = 6000.0; // free speed – loaded speed will be lower

    // ---------------- Feedforward ----------------
    public static double kV = 1.0 / 6000.0; // power per RPM at 12 V (start: 1 / free speed)
    public static double kS = 0.03; // power to overcome static friction

    // ---------------- PID ----------------
    public static double kP = 0.0004; // power per RPM of error
    public static double kI = 0.0; // power per (RPM * second)
    public static double kD = 0.0; // power per (RPM / second)
    public static double I_LIMIT = 0.25; // max power the integral term may contribute
    public static double I_ZONE_RPM = 400.0; // integrate only when |error| < this (0 = always)

    // ---------------- Signal shaping ----------------
    public static double RAMP_RPM_PER_SEC = 6000.0; // target slew limit (0 = instant, use for step tests)
    public static double FILTER_ALPHA = 0.3; // velocity low-pass: 0 = none, 0.9 = heavy
    public static boolean VOLTAGE_COMP = true; // scale feedforward by 12 V / battery
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double AT_SPEED_TOLERANCE_RPM = 100.0; // for isAtSpeed()
    public static double REVERSE_SAFE_RPM = 300.0; // wheel must be slower than this before setDirection()

    // ---------------- Hardware objects ----------------
    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------- Commands ----------------
    private double targetRpm = 0.0; // always positive
    private boolean enabled = false;
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD; // applied to the motor
    private DcMotorSimple.Direction requestedDirection = DcMotorSimple.Direction.FORWARD; // what the driver asked for

    // ---------------- Controller state ----------------
    private double rampedTarget = 0.0; // RPM actually fed to the controller
    private double rawRpm = 0.0;
    private double filteredRpm = 0.0;
    private double lastRpm = 0.0;
    private double integral = 0.0;
    private double power = 0.0;
    private double voltage = NOMINAL_VOLTAGE;
    private double lastTime;
    private double dt = 0.0;

    public LauncherPID(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        applyDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// We run our own PID, so turn the hub's built-in velocity PID OFF.
// RUN_WITHOUT_ENCODER still lets us READ the encoder.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, don't brake a flywheel

        battery = hardwareMap.voltageSensor.iterator().next();
        lastTime = timer.seconds();
    }

// ======================= Commands =======================

    /** Target speed in RPM (0..MAX_RPM). */
    public void setTargetRPM(double rpm) {
        targetRpm = Range.clip(rpm, 0.0, MAX_RPM);
    }

    public void changeTargetRPM(double deltaRpm) {
        setTargetRPM(targetRpm + deltaRpm);
    }

    public void setEnabled(boolean on) {
        if (on != enabled) integral = 0.0;
        enabled = on;
    }

    public void toggleEnabled() {
        setEnabled(!enabled);
    }

    /**
     * Request FORWARD or REVERSE. If the wheel is spinning it ramps down first;
     * motor.setDirection() is called in update() once it's safe.
     */
    public void setDirection(DcMotorSimple.Direction dir) {
        requestedDirection = dir;
    }

    public void reverseDirection() {
        setDirection(requestedDirection == DcMotorSimple.Direction.FORWARD
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
    }

// ======================= Loop =======================

    /** Call exactly once per loop. Reads the encoder, runs the PID, sets motor power. */
    public void update() {
        double now = timer.seconds();
        dt = Math.max(now - lastTime, 1e-3);
        lastTime = now;

// ---- Measure ----
        rawRpm = motor.getVelocity() / TICKS_PER_REV * 60.0; // ticks/s -> RPM
        filteredRpm = FILTER_ALPHA * filteredRpm + (1.0 - FILTER_ALPHA) * rawRpm;
        voltage = battery.getVoltage();

// ---- Direction change: only once ramped down AND the wheel has slowed ----
        if (isReversing()
                && rampedTarget < 1.0
                && Math.abs(filteredRpm) < REVERSE_SAFE_RPM) {
            applyDirection(requestedDirection);
// Encoder sign just flipped – restart the measurement and controller from zero
            filteredRpm = 0.0;
            lastRpm = 0.0;
            integral = 0.0;
        }

// ---- Commanded target with optional ramp ----
        double desired = (enabled && !isReversing()) ? targetRpm : 0.0;
        if (RAMP_RPM_PER_SEC > 0) {
            double maxChange = RAMP_RPM_PER_SEC * dt;
            rampedTarget += Range.clip(desired - rampedTarget, -maxChange, maxChange);
        } else {
            rampedTarget = desired;
        }

// ---- Control ----
        if (rampedTarget < 1.0) {
// Off, target 0, or waiting to reverse -> just coast
            power = 0.0;
            integral = 0.0;
            lastRpm = filteredRpm;
        } else {
            power = computePower(rampedTarget, filteredRpm);
        }
        motor.setPower(power);
    }

    /** Feedforward + PID. Returns motor power in [-1, 1]. */
    private double computePower(double target, double actual) {
        double error = target - actual;

// Feedforward (target is always positive here)
        double ff = kV * target + kS;
        if (VOLTAGE_COMP && voltage > 1.0) ff *= NOMINAL_VOLTAGE / voltage;

// Integral with I-zone + anti-windup clamp
        if (kI != 0.0 && (I_ZONE_RPM <= 0 || Math.abs(error) < I_ZONE_RPM)) {
            integral += error * dt;
            double iMax = Math.abs(I_LIMIT / kI);
            integral = Range.clip(integral, -iMax, iMax);
        }

// Derivative on measurement
        double dMeasured = (actual - lastRpm) / dt;
        lastRpm = actual;

        double output = ff + kP * error + kI * integral - kD * dMeasured;
        return Range.clip(output, -1.0, 1.0);
    }

    /** Sets the real motor direction, accounting for MOTOR_REVERSED (mounting). */
    private void applyDirection(DcMotorSimple.Direction dir) {
        direction = dir;
        DcMotorSimple.Direction hw = dir;
        if (MOTOR_REVERSED) {
            hw = (dir == DcMotorSimple.Direction.FORWARD)
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD;
        }
        motor.setDirection(hw);
    }

    /** Immediately cut power and reset everything. */
    public void stop() {
        enabled = false;
        rampedTarget = 0.0;
        integral = 0.0;
        power = 0.0;
        motor.setPower(0.0);
    }

// ======================= Getters =======================

    public boolean isEnabled() { return enabled; }
    /** Direction currently applied to the motor. */
    public DcMotorSimple.Direction getDirection() { return direction; }
    /** Direction the driver asked for (may still be pending). */
    public DcMotorSimple.Direction getRequestedDirection() { return requestedDirection; }
    /** True while waiting for the wheel to slow down before switching direction. */
    public boolean isReversing() { return requestedDirection != direction; }
    /** Requested speed (0 when off). */
    public double getTargetRPM() { return enabled ? targetRpm : 0.0; }
    /** Requested speed, even while off. */
    public double getSetpointRPM() { return targetRpm; }
    /** Ramped target the controller is actually chasing. */
    public double getRampedTargetRPM() { return rampedTarget; }
    /** Filtered measured speed in the current motor direction. */
    public double getActualRPM() { return filteredRpm; }
    public double getRawRPM() { return rawRpm; }
    public double getErrorRPM() { return rampedTarget - filteredRpm; }
    public double getPower() { return power; }
    public double getIntegral() { return integral; }
    public double getVoltage() { return voltage; }
    public double getLoopTimeMs() { return dt * 1000.0; }

    /** True when on, not reversing, done ramping, and within tolerance – use this to gate feeding a ball. */
    public boolean isAtSpeed() {
        return enabled
                && !isReversing()
                && Math.abs(targetRpm - rampedTarget) < 1.0
                && Math.abs(targetRpm - filteredRpm) < AT_SPEED_TOLERANCE_RPM;
    }
}
