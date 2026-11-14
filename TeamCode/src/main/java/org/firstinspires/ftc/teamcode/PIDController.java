package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * PID Controller implementation for REV Control Hub FTC Robotics
 * Optimized for GoBilda 5203 Yellow Jacket motor (6000 RPM max)
 *
 * Usage:
 * - Set target RPM using setTargetRPM()
 * - Call update() with current RPM to get output power (-1.0 to 1.0)
 * - Apply output power to motor
 */
public class PIDController {
    private double kP;          // Proportional gain
    private double kI;          // Integral gain
    private double kD;          // Derivative gain
    private double kF;          // Feedforward gain (for counteracting friction and voltage drop)

    private double targetValue; // Target RPM
    private double errorSum;    // Sum of errors for integral term
    private double prevError;   // Previous error for derivative term
    private long prevTimeNs;    // Time of previous update in nanoseconds

    private boolean initialized;
    private final double maxOutput;

    // Preset settings storage file.
    // Uses java.io (NOT java.nio.file): Files/Paths/Path only exist on Android
    // API 26+, so they crash with NoClassDefFoundError on older controllers.
    // A relative path resolves to the RobotController app's private data
    // directory, which the app can always read and write.
    private static final String PRESET_FILE = "launcher_presets.txt";

    /**
     * Create a new PID controller
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feedforward gain (optional, helps with steady-state error)
     * @param maxOutput Maximum output magnitude (default 1.0 for full power)
     */
    public PIDController(double kP, double kI, double kD, double kF, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.targetValue = 0;
        this.errorSum = 0;
        this.prevError = 0;
        this.prevTimeNs = System.nanoTime();
        this.initialized = false;
        this.maxOutput = maxOutput;
    }

    /**
     * Create a new PID controller with default max output of 1.0
     */
    public PIDController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    /**
     * Create a new PID controller without feedforward
     */
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 1.0);
    }

    /**
     * Load preset settings from file (if exists)
     * @return true if loaded successfully, false otherwise
     */
    public static boolean loadPresetsFromFile() {
        File file = new File(PRESET_FILE);
        if (!file.exists()) {
            return false;
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.trim().isEmpty() || line.startsWith("#")) continue; // Skip comments and empty lines

                String[] parts = line.split("=");
                if (parts.length == 2) {
                    String key = parts[0].trim();
                    double value = Double.parseDouble(parts[1].trim());

                    switch (key) {
                        case "preset1_rpm":
                            BallLauncherOpMode.PRESET_RPM_1 = value;
                            break;
                        case "preset2_rpm":
                            BallLauncherOpMode.PRESET_RPM_2 = value;
                            break;
                        case "current_preset":
                            BallLauncherOpMode.currentPreset = (int)value;
                            break;
                    }
                }
            }

            return true;
        } catch (IOException | NumberFormatException e) {
            // File unreadable or malformed - use defaults
            return false;
        }
    }

    /**
     * Save current settings to file
     */
    public static void savePresetsToFile() {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(new File(PRESET_FILE)))) {
            writer.write("# Ball Launcher Preset Settings\n");
            writer.write("# Format: key=value (one per line)\n");
            writer.write("\n");
            writer.write("preset1_rpm=" + BallLauncherOpMode.PRESET_RPM_1 + "\n");
            writer.write("preset2_rpm=" + BallLauncherOpMode.PRESET_RPM_2 + "\n");
            writer.write("current_preset=" + BallLauncherOpMode.currentPreset + "\n");
        } catch (IOException e) {
            // Log the failure so a save problem is visible instead of silent
            e.printStackTrace();
        }
    }

    /**
     * Update the PID controller and get output power
     * @param currentValue Current RPM measurement
     * @return Motor power output (-maxOutput to +maxOutput)
     */
    public double update(double currentValue) {
        long currentTimeNs = System.nanoTime();

        // Calculate time delta in seconds (limit max dt to 100ms)
        double deltaTime = Math.min((currentTimeNs - prevTimeNs) / 1_000_000_000.0, 0.1);

        if (deltaTime <= 0) {
            deltaTime = 0.01; // Default to 10ms if invalid
        }

        double error = targetValue - currentValue;

        // Calculate P term
        double pTerm = kP * error;

        // Calculate I term (with anti-windup clamping)
        errorSum += error * deltaTime;

        // Clamp integral to prevent windup (max 30% of output range)
        double maxIntegral = 0.3 / Math.max(kI, 1e-10);
        if (errorSum > maxIntegral) {
            errorSum = maxIntegral;
        } else if (errorSum < -maxIntegral) {
            errorSum = -maxIntegral;
        }
        double iTerm = kI * errorSum;

        // Calculate D term (with derivative filtering to reduce noise)
        double derivative = (error - prevError) / deltaTime;
        double dTerm = kD * derivative;

        // Calculate feedforward term
        // For GoBilda 5203: ~1.5V drop at max load, so compensate accordingly
        double fTerm = kF * Math.signum(targetValue) * Math.abs(targetValue);

        // Sum all terms
        double output = pTerm + iTerm + dTerm + fTerm;

        // Clamp output to valid range
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < -maxOutput) {
            output = -maxOutput;
        }

        // Update state for next iteration
        prevError = error;
        prevTimeNs = currentTimeNs;
        initialized = true;

        return output;
    }

    /**
     * Set the target RPM (positive value, direction handled separately)
     */
    public void setTargetRPM(double rpm) {
        this.targetValue = Math.abs(rpm);
        reset();
    }

    /**
     * Get current target RPM
     */
    public double getTargetRPM() {
        return targetValue;
    }

    /**
     * Reset the controller state (integral, derivative)
     */
    public void reset() {
        this.errorSum = 0;
        this.prevError = 0;
        this.prevTimeNs = System.nanoTime();
    }

    /**
     * Check if the controller is close to target
     */
    public boolean atSetpoint(double tolerance) {
        double error = Math.abs(targetValue - (prevError > 0 ? targetValue - prevError : targetValue + prevError));
        return error <= tolerance && initialized;
    }

    // Getters and setters for tuning
    public void setPGain(double kP) {
        this.kP = kP;
    }

    public void setIGain(double kI) {
        this.kI = kI;
        reset();
    }

    public void setDGain(double kD) {
        this.kD = kD;
    }

    public void setFGain(double kF) {
        this.kF = kF;
    }

    public double getPGain() { return kP; }
    public double getIGain() { return kI; }
    public double getDGain() { return kD; }
    public double getFGain() { return kF; }

    /**
     * Get PID coefficients as a formatted string for telemetry
     */
    public String getTelemetryString() {
        return String.format("P: %.5f  I: %.7f  D: %.6f  F: %.8f", kP, kI, kD, kF);
    }

    /**
     * Get PID coefficients in array format for easy tuning
     */
    public double[] getGains() {
        return new double[]{kP, kI, kD, kF};
    }
}