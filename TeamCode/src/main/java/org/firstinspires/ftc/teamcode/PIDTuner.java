package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * PID Tuner OpMode for GoBilda 5203 Yellow Jacket motor
 *
 * Usage:
 * 1. Run this opmode with your motor
 * 2. Use gamepad to adjust PID values in real-time
 * 3. Observe the motor response on telemetry
 * 4. Find values that give smooth, fast response without oscillation
 */
@TeleOp(name="PID Tuner (5203)", group="Tuning")
public class PIDTuner extends OpMode {

    private DcMotorEx testMotor;
    private PIDController pid;

    // GoBilda 5203 Yellow Jacket specifications
    private static final int COUNTS_PER_REV = 28;
    private static final double MAX_RPM = 6000.0;

    // Tuning parameters (change these to start from different values)
    private double kP = 0.0005;
    private double kI = 0.00001;
    private double kD = 0.0001;
    private double kF = 0.00001;

    // Test parameters
    private static final double TEST_RPM = 3000.0; // Test at half speed

    // Tuning increments (adjust for finer/coarser control)
    private static final double DPAD_INCREMENT_P = 0.00005;
    private static final double DPAD_INCREMENT_D = 0.00005;
    private static final double SLIDER_INCREMENT_I = 0.000005;
    private static final double SLIDER_INCREMENT_F = 0.000005;

    // Button state tracking for edge detection
    // (Gamepad has no copy() method, so track the previous A state with a boolean)
    private boolean aWasPressed = false;

    private long lastTelemetryTime = 0;
    private boolean tuningMode = true; // false = test mode

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing PID Tuner for GoBilda 5203");

        testMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        testMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        testMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);

        pid = new PIDController(kP, kI, kD, kF);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Mode", tuningMode ? "TUNING" : "TEST");
        telemetry.addData("Press A", "to toggle modes");
    }

    @Override
    public void start() {
        lastTelemetryTime = 0;
        // Sync edge detection with the gamepad state at start time
        aWasPressed = gamepad1.a;
    }

    @Override
    public void loop() {
        handleInput();

        if (tuningMode) {
            updateTuning();
        } else {
            runTest();
        }

        // Telemetry at controlled rate
        if (System.nanoTime() / 1_000_000 - lastTelemetryTime >= 50) {
            updateTelemetry();
            lastTelemetryTime = System.nanoTime() / 1_000_000;
        }
    }

    private void handleInput() {
        // Toggle between tuning and test mode (rising edge of A)
        if (gamepad1.a && !aWasPressed) {
            tuningMode = !tuningMode;
            pid.reset();
        }
        aWasPressed = gamepad1.a;

        if (!tuningMode && gamepad1.b) {
            // Emergency stop in test mode
            testMotor.setPower(0);
        }
    }

    private void updateTuning() {
        // Adjust P gain (DPAD)
        if (gamepad1.dpad_up) kP += DPAD_INCREMENT_P;
        if (gamepad1.dpad_down && kP > 0.00005) kP -= DPAD_INCREMENT_P;

        // Adjust I gain (L-RBUMPER)
        if (gamepad1.left_bumper) kI += SLIDER_INCREMENT_I;
        if (gamepad1.right_bumper && kI > 0) kI -= SLIDER_INCREMENT_I;

        // Adjust D gain (Y/B)
        if (gamepad1.y) kD += DPAD_INCREMENT_D;
        if (gamepad1.b && kD > 0) kD -= DPAD_INCREMENT_D;

        // Adjust F gain (START + DPAD)
        if (gamepad1.start && gamepad1.dpad_up) kF += SLIDER_INCREMENT_F;
        if (gamepad1.start && gamepad1.dpad_down && kF > 0) kF -= SLIDER_INCREMENT_F;

        // Apply new gains
        pid.setPGain(kP);
        pid.setIGain(kI);
        pid.setDGain(kD);
        pid.setFGain(kF);
    }

    private void runTest() {
        // Only set the target when it changes; setTargetRPM() resets I/D state
        if (pid.getTargetRPM() != TEST_RPM) {
            pid.setTargetRPM(TEST_RPM);
        }
        double currentRPM = getCurrentRPM();
        double power = pid.update(currentRPM);

        testMotor.setPower(power);
    }

    private double getCurrentRPM() {
        double velocityTicksPerSec = testMotor.getVelocity();
        return (velocityTicksPerSec / COUNTS_PER_REV) * 60.0;
    }

    private void updateTelemetry() {
        telemetry.clearAll();

        if (tuningMode) {
            telemetry.addData("Mode", "TUNING - Adjust gains in real-time");

            // Show increments
            telemetry.addData("\nAdjust using:",
                    "DPAD_UP/DOWN: P +/- %.5f", DPAD_INCREMENT_P);
            telemetry.addData("L-RBUMPER: I +/- %.7f", SLIDER_INCREMENT_I);
            telemetry.addData("Y/B: D +/- %.5f", DPAD_INCREMENT_D);
            telemetry.addData("START+DPAD: F +/- %.7f", SLIDER_INCREMENT_F);

            // Display current gains
            telemetry.addData("\nCurrent Gains:", "P=%.5f I=%.7f D=%.6f F=%.8f", kP, kI, kD, kF);
            telemetry.addData("Max RPM:", "%.0f", MAX_RPM);
        } else {
            telemetry.addData("Mode", "TEST - Running at " + TEST_RPM + " RPM");

            double currentRPM = getCurrentRPM();
            double error = Math.abs(TEST_RPM - currentRPM);

            telemetry.addData("Current RPM:", "%.1f", currentRPM);
            telemetry.addData("Error:", "%.1f RPM", error);
            telemetry.addData("Power:", "%.3f", testMotor.getPower());
            telemetry.addData("Velocity (ticks/s):", "%.0f", testMotor.getVelocity());

            // Motor status
            telemetry.addData("Current (Amps):", "%.1f", testMotor.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("\nPress B to stop motor");
            telemetry.addLine("Press A to return to tuning");
        }

        telemetry.update();
    }
}