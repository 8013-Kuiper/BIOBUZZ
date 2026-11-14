package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Ball Launcher OpMode for GoBilda 5203 Yellow Jacket motor
 *
 * Features:
 * - PID control with feedforward compensation
 * - Two preset RPM settings that can be saved to file
 * - Real-time telemetry showing target/actual RPM
 * - Direction control (CW/CCW)
 */
@TeleOp(name="Ball Launcher", group="Launcher")
public class BallLauncherOpMode extends LinearOpMode {

    // Hardware
    private DcMotorEx launcherMotor;

    // PID Controller
    private PIDController pidController;

    // GoBilda 5203 Yellow Jacket specifications
    private static final int COUNTS_PER_REV = 28;
    private static final double MAX_RPM = 6000.0;

    // Preset RPM values (will be loaded from file if available)
    public static double PRESET_RPM_1 = 3000.0;  // Default: half speed
    public static double PRESET_RPM_2 = 5000.0;  // Default: high speed
    public static int currentPreset = 1;

    // Direction control
    private boolean direction = true;  // true = forward (CW), false = reverse (CCW)

    // State variables
    private double targetRPM;
    private ElapsedTime runtime = new ElapsedTime();
    private long lastTelemetryTime = 0;

    // Previous gamepad button states for rising-edge detection.
    // (Gamepad has no copy() method, so track each button with a boolean.)
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevStartDpad = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Load presets from file (if they exist)
        PIDController.loadPresetsFromFile();

        // Set initial target based on current preset
        updateTargetRPM();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Preset", currentPreset + ": " + String.format("%.0f RPM", targetRPM));
        telemetry.addData("Direction", direction ? "CW" : "CCW");
        telemetry.update();

        // Wait for start
        waitForStart();

        runtime.reset();
        lastTelemetryTime = 0;

        // Main control loop
        while (opModeIsActive()) {
            handleInput();

            double currentRPM = getCurrentRPM();
            // Only set the target when it changes; setTargetRPM() resets I/D state
            if (pidController.getTargetRPM() != targetRPM) {
                pidController.setTargetRPM(targetRPM);
            }
            double power = pidController.update(currentRPM);

            // Apply direction and power
            launcherMotor.setPower(power * (direction ? 1.0 : -1.0));

            // Update telemetry at controlled rate
            if (System.nanoTime() / 1_000_000 - lastTelemetryTime >= 100) {
                updateTelemetry(currentRPM);
                lastTelemetryTime = System.nanoTime() / 1_000_000;
            }
        }

        // Stop motor on exit
        launcherMotor.setPower(0);
    }

    private void initializeHardware() {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        // Configure motor for velocity control
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);

        // Initialize PID controller with tuned values for GoBilda 5203
        double kP = 0.0005;
        double kI = 0.00001;
        double kD = 0.0001;
        double kF = 0.00001;

        pidController = new PIDController(kP, kI, kD, kF);
    }

    private void handleInput() {
        // Save presets to file (START + DPAD_UP or DPAD_DOWN, rising edge)
        boolean startDpad = gamepad1.start && (gamepad1.dpad_up || gamepad1.dpad_down);
        if (startDpad && !prevStartDpad) {
            PIDController.savePresetsToFile();
        }
        prevStartDpad = startDpad;

        // Switch between presets (DPAD buttons)
        if (gamepad1.dpad_up && !prevDpadUp) {
            currentPreset = 1;
            updateTargetRPM();
            PIDController.savePresetsToFile();
        }
        prevDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !prevDpadDown) {
            currentPreset = 2;
            updateTargetRPM();
            PIDController.savePresetsToFile();
        }
        prevDpadDown = gamepad1.dpad_down;

        // Change direction (Y button)
        if (gamepad1.y && !prevY) {
            direction = !direction;
        }
        prevY = gamepad1.y;

        // Reset PID controller (X button)
        if (gamepad1.x && !prevX) {
            pidController.reset();
        }
        prevX = gamepad1.x;
    }

    private void updateTargetRPM() {
        targetRPM = currentPreset == 1 ? PRESET_RPM_1 : PRESET_RPM_2;
    }

    private double getCurrentRPM() {
        double velocityTicksPerSec = launcherMotor.getVelocity();
        return (velocityTicksPerSec / COUNTS_PER_REV) * 60.0;
    }

    private void updateTelemetry(double currentRPM) {
        telemetry.clearAll();

        // Status
        telemetry.addData("Status", runtime.seconds() > 1 ? "Running" : "Initializing");
        telemetry.addData("Preset", currentPreset + ": " + String.format("%.0f RPM", targetRPM));
        telemetry.addData("Direction", direction ? "CW (forward)" : "CCW (reverse)");

        // RPM values
        telemetry.addData("Target RPM:", "%.1f", targetRPM);
        telemetry.addData("Actual RPM:", "%.1f", currentRPM);
        telemetry.addData("Speed Error:", "%.1f RPM", Math.abs(targetRPM - currentRPM));

        // Motor status
        telemetry.addData("Power Output:", "%.3f", launcherMotor.getPower());
        telemetry.addData("Velocity (ticks/s):", "%.0f", launcherMotor.getVelocity());
        telemetry.addData("Current (Amps):", "%.1f", launcherMotor.getCurrent(CurrentUnit.AMPS));

        // PID coefficients
        telemetry.addData("\nPID Coefficients:", pidController.getTelemetryString());

        // Controls info
        telemetry.addData("\nControls:",
                "DPAD_UP: Preset 1 (%.0f RPM)", PRESET_RPM_1);
        telemetry.addData("",
                "DPAD_DOWN: Preset 2 (%.0f RPM)", PRESET_RPM_2);
        telemetry.addData("",
                "Y: Toggle Direction");
        telemetry.addData("",
                "X: Reset PID");

        telemetry.update();
    }
}