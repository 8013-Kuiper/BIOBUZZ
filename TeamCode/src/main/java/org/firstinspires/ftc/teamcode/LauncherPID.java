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
 *   LauncherPID launcher = new LauncherPID(hardwareMap);
 *   launcher.setTargetRPM(4500);
 *   launcher.setEnabled(true);
 *   while (opModeIsActive()) { launcher.update(); ... }
 *
 * CONTROL LAW
 *   power = (kV * target + kS * sign(target)) * (12V / batteryV)   <- feedforward
 *         +  kP * error + kI * integral(error) - kD * d(actual)/dt  <- PID
 *   (D acts on the measurement, so a target change doesn't cause a "kick".)
 *
 * All "public static" fields are live-tunable in FTC Dashboard's Config tab
 * under "LauncherPID". Copy final values back into this file – dashboard
 * edits are lost when the robot restarts.
 */
@Config
public class LauncherPID {

    // ---------------- Hardware ----------------
    public static String  MOTOR_NAME      = "launcher"; // name in the robot configuration
    public static boolean MOTOR_REVERSED  = false;      // flip if "forward" spins the wrong way
    public static double  TICKS_PER_REV   = 28.0;       // 6000 RPM motor encoder at output shaft
    public static double  MAX_RPM         = 6000.0;     // free speed – loaded speed will be lower

    // ---------------- Feedforward ----------------
    public static double kV = 1.0 / 6000.0;  // power per RPM at 12 V (start: 1 / free speed)
    public static double kS = 0.03;          // power to overcome static friction

    // ---------------- PID ----------------
    public static double kP = 0.0004;        // power per RPM of error
    public static double kI = 0.0;           // power per (RPM * second)
    public static double kD = 0.0;           // power per (RPM / second)
    public static double I_LIMIT = 0.25;     // max power the integral term may contribute
    public static double I_ZONE_RPM = 400.0; // integrate only when |error| < this (0 = always)

    // ---------------- Signal shaping ----------------
    public static double  RAMP_RPM_PER_SEC = 6000.0; // target slew limit (0 = instant, use for step tests)
    public static double  FILTER_ALPHA     = 0.3;    // velocity low-pass: 0 = none, 0.9 = heavy
    public static boolean VOLTAGE_COMP     = true;   // scale feedforward by 12 V / battery
    public static double  NOMINAL_VOLTAGE  = 12.0;
    public static double  AT_SPEED_TOLERANCE_RPM = 100.0; // for isAtSpeed()

    // ---------------- Hardware objects ----------------
    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------- Commands ----------------
    private double targetRpm = 0.0;   // always positive; direction is separate
    private int direction = 1;        // +1 forward, -1 reverse
    private boolean enabled = false;

    // ---------------- Controller state ----------------
    private double rampedTarget = 0.0; // signed RPM actually fed to the controller
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
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // We run our own PID, so turn the hub's built-in velocity PID OFF.
        // RUN_WITHOUT_ENCODER still lets us READ the encoder.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, don't brake a flywheel

        battery = hardwareMap.voltageSensor.iterator().next();
        lastTime = timer.seconds();
    }

    // ======================= Commands =======================

    /** Target speed in RPM (0..MAX_RPM). Direction is set separately. */
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

    /** +1 = forward, -1 = reverse. Ramps through zero instead of slamming. */
    public void setDirection(int dir) {
        int newDir = dir >= 0 ? 1 : -1;
        if (newDir != direction) integral = 0.0; // old integral pushes the wrong way
        direction = newDir;
    }

    public void reverseDirection() {
        setDirection(-direction);
    }

    // ======================= Loop =======================

    /** Call exactly once per loop. Reads the encoder, runs the PID, sets motor power. */
    public void update() {
        double now = timer.seconds();
        dt = Math.max(now - lastTime, 1e-3);
        lastTime = now;

        // ---- Commanded (signed) target with optional ramp ----
        double desired = enabled ? direction * targetRpm : 0.0;
        if (RAMP_RPM_PER_SEC > 0) {
            double maxChange = RAMP_RPM_PER_SEC * dt;
            rampedTarget += Range.clip(desired - rampedTarget, -maxChange, maxChange);
        } else {
            rampedTarget = desired;
        }

        // ---- Measure ----
        rawRpm = motor.getVelocity() / TICKS_PER_REV * 60.0; // ticks/s -> RPM
        filteredRpm = FILTER_ALPHA * filteredRpm + (1.0 - FILTER_ALPHA) * rawRpm;
        voltage = battery.getVoltage();

        // ---- Control ----
        if (!enabled && Math.abs(rampedTarget) < 1.0) {
            // Fully off once ramped down -> just coast
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

        // Feedforward
        double ff = kV * target + (Math.abs(target) > 1.0 ? kS * Math.signum(target) : 0.0);
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

    /** Immediately cut power and reset everything. */
    public void stop() {
        enabled = false;
        rampedTarget = 0.0;
        integral = 0.0;
        power = 0.0;
        motor.setPower(0.0);
    }

    // ======================= Getters =======================

    public boolean isEnabled()          { return enabled; }
    public int getDirection()           { return direction; }
    /** Requested speed, signed by direction (0 when off). */
    public double getTargetRPM()        { return enabled ? direction * targetRpm : 0.0; }
    /** Requested speed magnitude, even while off. */
    public double getSetpointRPM()      { return targetRpm; }
    /** Ramped target the controller is actually chasing (signed). */
    public double getRampedTargetRPM()  { return rampedTarget; }
    /** Filtered measured speed (signed). */
    public double getActualRPM()        { return filteredRpm; }
    public double getRawRPM()           { return rawRpm; }
    public double getErrorRPM()         { return rampedTarget - filteredRpm; }
    public double getPower()            { return power; }
    public double getIntegral()         { return integral; }
    public double getVoltage()          { return voltage; }
    public double getLoopTimeMs()       { return dt * 1000.0; }

    /** True when on, done ramping, and within tolerance – use this to gate feeding a ball. */
    public boolean isAtSpeed() {
        return enabled
                && Math.abs(getTargetRPM() - rampedTarget) < 1.0
                && Math.abs(getTargetRPM() - filteredRpm) < AT_SPEED_TOLERANCE_RPM;
    }
}