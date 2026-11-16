package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Test / tuning OpMode for LauncherPID.
 *
 * CONTROLS (gamepad1)
 * A – launcher ON / OFFx
 * B – reverse direction (spins down, motor.setDirection(), spins back up)
 * Dpad Up/Down – target RPM +/- SMALL_STEP_RPM
 * RB / LB – target RPM +/- LARGE_STEP_RPM
 * X – jump to PRESET_RPM
 *
 * FTC DASHBOARD (http://192.168.43.1:8080/dash on the Control Hub Wi-Fi)
 * Config tab: "LauncherPID" -> gains, ramp, filter
 * "LauncherPIDTest" -> TARGET_RPM, step sizes, preset
 * Graph tab: targetRPM, rampedTargetRPM, actualRPM, error, power
 */
@Config
@TeleOp(name = "Launcher PID Test", group = "Launcher")
public class LauncherPIDTest extends LinearOpMode {

    public static double TARGET_RPM = 3000.0; // editable from gamepad AND dashboard
    public static double SMALL_STEP_RPM = 100.0;
    public static double LARGE_STEP_RPM = 500.0;
    public static double PRESET_RPM = 4500.0;

    // Gamepad edge detection
    private boolean lastA, lastB, lastX, lastUp, lastDown, lastRB, lastLB;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

// Bulk reads: one hub transaction per loop -> faster, smoother PID
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        LauncherPID launcher = new LauncherPID(hardwareMap);

        telemetry.addLine("Launcher PID Test ready");
        telemetry.addLine("A: on/off | B: reverse | Dpad: +/-" + (int) SMALL_STEP_RPM
                + " | Bumpers: +/-" + (int) LARGE_STEP_RPM + " | X: preset");
        telemetry.addLine("Dashboard: http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

// ---- Gamepad (rising-edge presses) ----
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;

            if (a && !lastA) launcher.toggleEnabled();
            if (b && !lastB) launcher.reverseDirection();
            if (x && !lastX) TARGET_RPM = PRESET_RPM;
            if (up && !lastUp) TARGET_RPM += SMALL_STEP_RPM;
            if (down && !lastDown) TARGET_RPM -= SMALL_STEP_RPM;
            if (rb && !lastRB) TARGET_RPM += LARGE_STEP_RPM;
            if (lb && !lastLB) TARGET_RPM -= LARGE_STEP_RPM;

            lastA = a; lastB = b; lastX = x;
            lastUp = up; lastDown = down; lastRB = rb; lastLB = lb;

// Push target to the controller (it clamps 0..MAX_RPM), then read back the clamp
            launcher.setTargetRPM(TARGET_RPM);
            TARGET_RPM = launcher.getSetpointRPM();

// ---- Run the PID ----
            launcher.update();

// ---- Driver Station telemetry ----
            telemetry.addData("Launcher", launcher.isEnabled() ? "ON" : "OFF");
            telemetry.addData("Direction", launcher.isReversing()
                    ? "SWITCHING TO " + launcher.getRequestedDirection() + "..."
                    : launcher.getDirection().toString());
            telemetry.addData("At Speed", launcher.isAtSpeed() ? "YES" : "no");
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Ramped Target RPM", "%.0f", launcher.getRampedTargetRPM());
            telemetry.addData("Real RPM", "%.0f", launcher.getActualRPM());
            telemetry.addData("Error RPM", "%.0f", launcher.getErrorRPM());
            telemetry.addData("Power", "%.3f", launcher.getPower());
            telemetry.addData("Battery", "%.2f V", launcher.getVoltage());
            telemetry.addData("Loop", "%.1f ms", launcher.getLoopTimeMs());
            telemetry.update();

// ---- Dashboard graphs (raw numbers so they can be plotted) ----
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("targetRPM", launcher.getTargetRPM());
            packet.put("rampedTargetRPM", launcher.getRampedTargetRPM());
            packet.put("actualRPM", launcher.getActualRPM());
            packet.put("rawRPM", launcher.getRawRPM());
            packet.put("error", launcher.getErrorRPM());
            packet.put("power", launcher.getPower());
            packet.put("integral", launcher.getIntegral());
            packet.put("reversing", launcher.isReversing() ? 1 : 0);
            packet.put("batteryV", launcher.getVoltage());
            packet.put("loopMs", launcher.getLoopTimeMs());
            dashboard.sendTelemetryPacket(packet);
        }

        launcher.stop();
    }
}