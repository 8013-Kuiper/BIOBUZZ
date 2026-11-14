package org.firstinspires.ftc.teamcode;

/**
 * Helper configuration class for the GoBilda 5203 Yellow Jacket ball launcher motor.
 *
 * This class centralizes all motor-specific constants and hardware mapping names
 * to make it easier to configure and maintain the launcher system.
 */
public class BallLauncherConfig {

    // Motor specifications - GoBilda 5203 Yellow Jacket
    public static final int COUNTS_PER_REV = 28;           // Built-in encoder counts per revolution
    public static final double MAX_RPM = 6000.0;            // Maximum no-load RPM
    public static final double NO_LOAD_CURRENT_A = 1.5;     // Approximate current at max speed

    // Default PID gains (starting point for tuning)
    public static final double DEFAULT_KP = 0.0005;
    public static final double DEFAULT_KI = 0.00001;
    public static final double DEFAULT_KD = 0.0001;
    public static final double DEFAULT_KF = 0.00001;

    // Motor hardware mapping name (configure in Robot Configuration)
    public static final String MOTOR_NAME = "launcherMotor";

    // Default preset RPM values
    public static final double PRESET_RPM_1 = 3000.0;      // Lower speed setting
    public static final double PRESET_RPM_2 = 5000.0;      // Higher speed setting

    // Preset file path for saving/loading settings
    public static final String PRESET_FILE = "launcher_presets.txt";

    /**
     * Calculate RPM from encoder velocity (ticks/second)
     */
    public static double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / COUNTS_PER_REV) * 60.0;
    }

    /**
     * Calculate encoder velocity (ticks/second) from RPM
     */
    public static double rpmToTicks(double rpm) {
        return (rpm / 60.0) * COUNTS_PER_REV;
    }
}