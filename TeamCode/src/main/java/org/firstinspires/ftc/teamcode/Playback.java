package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Playback")
public class Playback extends LinearOpMode {

    double startX = 0;
    double startY = 0;
    double startAngle = 0;

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    CRServo frontLeftCR;
    CRServo frontRightCR;
    CRServo rearLeftCR;
    CRServo rearRightCR;

    AnalogInput frontLeftPos;
    AnalogInput frontRightPos;
    AnalogInput rearLeftPos;
    AnalogInput rearRightPos;

    IMU imu;

    private final ElapsedTime runtime = new ElapsedTime();
    private static final String RECORDING_FILE = Record.FILE_NAME;
    private static final double MAX_PLAYBACK_SECONDS = 30.0;

    private static class RecordedFrame {
        final double timeMs;
        final Gamepad gamepad1;
        final Gamepad gamepad2;

        RecordedFrame(double timeMs, Gamepad gamepad1, Gamepad gamepad2) {
            this.timeMs = timeMs;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }
    }

    private ArrayList<RecordedFrame> loadRecording() throws IOException {
        ArrayList<RecordedFrame> frames = new ArrayList<>();
        File file = AppUtil.getInstance().getSettingsFile(RECORDING_FILE);

        if (!file.exists()) {
            throw new IOException("Recording file not found: " + file.getAbsolutePath());
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String header = reader.readLine();
            if (header == null) {
                throw new IOException("Recording file is empty.");
            }

            String line;
            int lineNumber = 1;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.trim().isEmpty()) {
                    continue;
                }

                String[] values = line.split(",", -1);
                if (values.length < 47) {
                    throw new IOException("Invalid recording row " + lineNumber
                            + ": expected at least 47 columns, got " + values.length);
                }

                try {
                    double timeMs = Double.parseDouble(values[0]);
                    frames.add(new RecordedFrame(
                            timeMs,
                            parseGamepad(values, 1),
                            parseGamepad(values, 24)
                    ));
                } catch (RuntimeException e) {
                    throw new IOException("Could not parse recording row "
                            + lineNumber + ": " + e.getMessage(), e);
                }
            }
        }

        if (frames.isEmpty()) {
            throw new IOException("Recording contains no input samples.");
        }

        return frames;
    }

    private Gamepad parseGamepad(String[] v, int index) {
        Gamepad g = new Gamepad();

        g.left_stick_x = Float.parseFloat(v[index++]);
        g.left_stick_y = Float.parseFloat(v[index++]);
        g.right_stick_x = Float.parseFloat(v[index++]);
        g.right_stick_y = Float.parseFloat(v[index++]);
        g.left_trigger = Float.parseFloat(v[index++]);
        g.right_trigger = Float.parseFloat(v[index++]);

        g.a = Boolean.parseBoolean(v[index++]);
        g.b = Boolean.parseBoolean(v[index++]);
        g.x = Boolean.parseBoolean(v[index++]);
        g.y = Boolean.parseBoolean(v[index++]);

        g.dpad_up = Boolean.parseBoolean(v[index++]);
        g.dpad_down = Boolean.parseBoolean(v[index++]);
        g.dpad_left = Boolean.parseBoolean(v[index++]);
        g.dpad_right = Boolean.parseBoolean(v[index++]);

        g.left_bumper = Boolean.parseBoolean(v[index++]);
        g.right_bumper = Boolean.parseBoolean(v[index++]);
        g.left_stick_button = Boolean.parseBoolean(v[index++]);
        g.right_stick_button = Boolean.parseBoolean(v[index++]);

        g.back = Boolean.parseBoolean(v[index++]);
        g.start = Boolean.parseBoolean(v[index++]);
        g.guide = Boolean.parseBoolean(v[index++]);
        g.share = Boolean.parseBoolean(v[index++]);
        g.ps = Boolean.parseBoolean(v[index]);

        return g;
    }

    private void copyGamepad(Gamepad from, Gamepad to) {
        to.left_stick_x = from.left_stick_x;
        to.left_stick_y = from.left_stick_y;
        to.right_stick_x = from.right_stick_x;
        to.right_stick_y = from.right_stick_y;
        to.left_trigger = from.left_trigger;
        to.right_trigger = from.right_trigger;

        to.a = from.a;
        to.b = from.b;
        to.x = from.x;
        to.y = from.y;

        to.dpad_up = from.dpad_up;
        to.dpad_down = from.dpad_down;
        to.dpad_left = from.dpad_left;
        to.dpad_right = from.dpad_right;

        to.left_bumper = from.left_bumper;
        to.right_bumper = from.right_bumper;
        to.left_stick_button = from.left_stick_button;
        to.right_stick_button = from.right_stick_button;

        to.back = from.back;
        to.start = from.start;
        to.guide = from.guide;
        to.share = from.share;
        to.ps = from.ps;
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    //Mechanism Declarations
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx flywheel;
    double intakePower;
    double flywheelPower;
    double distance = 59;

    double RPM = 2500.0;
    public final double MAX_RPM = 5000.0;

    //Tuning Constants
    //Proportional gain: converts degrees of error into motor rotation power [-1, 1]
    final double AIM_COEFFICIENT = 0.035;

    //Constant offset (in degrees) to compensate for camera mounting position or shooter bias
    //Positive shifts the aim point to the right; negative shifts it to the left
    final double BEARING_OFFSET_DEG = -2.0;

    //Tolerance (in degrees) for setting isAimed = true
    final double AIM_TOLERANCE_DEG = 5.0;

    //Control & State Variables
    boolean redAllianceTarget = true;
    boolean isAimed = false;
    double aimRotation = 0.0; // Range [-1, 1], positive = Clockwise (CW)

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftCR = hardwareMap.get(CRServo.class, "frontLeftCR");
        frontRightCR = hardwareMap.get(CRServo.class, "frontRightCR");
        rearLeftCR = hardwareMap.get(CRServo.class, "rearLeftCR");
        rearRightCR = hardwareMap.get(CRServo.class, "rearRightCR");

        frontLeftPos = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
        frontRightPos = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
        rearLeftPos = hardwareMap.get(AnalogInput.class, "rearLeftAnalog");
        rearRightPos = hardwareMap.get(AnalogInput.class, "rearRightAnalog");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SwervePod fl = new SwervePod("frontLeft", frontLeftCR, frontLeft, frontLeftPos);
        SwervePod fr = new SwervePod("frontRight", frontRightCR, frontRight, frontRightPos);
        SwervePod rl = new SwervePod("rearLeft", rearLeftCR, rearLeft, rearLeftPos);
        SwervePod rr = new SwervePod("rearRight", rearRightCR, rearRight, rearRightPos);

        SwerveDrive drive = new SwerveDrive(fl, fr, rl, rr);

        //SwervePod uses TeleOP's static control flags.
        TeleOP.updateOdometry = true;
        TeleOP.resetPods = false;
        TeleOP.isNoInput = false;
        TeleOP.fieldCentric = false;
        TeleOP.holdHeading = true;
        SwervePod.saveRollover = true;

        Odometry odometry = new Odometry(fl, fr, rl, rr, drive, imu);
        odometry.setStartingPosition(startX, startY, startAngle);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        fl.readLogFile();
        fr.readLogFile();
        rl.readLogFile();
        rr.readLogFile();

        //Mechanism Initialization
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Camera
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTagProcessor)
                .build();
        //End Mechanism Initialization

        ArrayList<RecordedFrame> frames;
        try {
            frames = loadRecording();
        } catch (IOException e) {
            telemetry.addLine("PLAYBACK FILE ERROR");
            telemetry.addData("File", RECORDING_FILE);
            telemetry.addData("Reason", e.getMessage());
            telemetry.update();
            stopDrive();
            return;
        }

        Gamepad replayGamepad1 = new Gamepad();
        Gamepad replayGamepad2 = new Gamepad();
        Controller driver = new Controller(replayGamepad1);
        Controller operator = new Controller(replayGamepad2);

        telemetry.addData("Recording", RECORDING_FILE);
        telemetry.addData("Samples", frames.size());
        telemetry.addData("Duration (s)",
                frames.get(frames.size() - 1).timeMs / 1000.0);
        telemetry.addLine("Ready. Press Start to play recording.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        double angle;
        double power;
        double rotation;
        double heading = 0;
        double desiredHeading = 0;
        double powerMultiplier;
        double powerSubtractor;
        boolean rotationError = false;

        SwervePod selectedPod = fl;
        String SelectedPod = "Front Left";

        runtime.reset();
        int frameIndex = 0;
        double lastRecordedMs = frames.get(frames.size() - 1).timeMs;

        try {
            while (opModeIsActive()
                    && runtime.seconds() < MAX_PLAYBACK_SECONDS
                    && runtime.milliseconds() <= lastRecordedMs) {

                double elapsedMs = runtime.milliseconds();

                //Advance only when the next recorded timestamp has arrived.
                while (frameIndex + 1 < frames.size()
                        && frames.get(frameIndex + 1).timeMs <= elapsedMs) {
                    frameIndex++;
                }

                RecordedFrame frame = frames.get(frameIndex);
                copyGamepad(frame.gamepad1, replayGamepad1);
                copyGamepad(frame.gamepad2, replayGamepad2);

                driver.update();
                operator.update();

                fl.update();
                fr.update();
                rl.update();
                rr.update();

                if (TeleOP.updateOdometry) {
                    odometry.update();
                }

                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                powerSubtractor =
                        (1.0 - driver.getRightTrigger()) * Record.POWER_SUBTRACTOR;
                powerMultiplier =
                        Math.min(1.0 - (driver.getLeftTrigger() - 0.30), 1.0)
                                - powerSubtractor;

                if (driver.getAToggle()) {
                    powerMultiplier /= 2.0;
                }

                angle = driver.getAngle();
                power = Math.pow(driver.getResultant(), 2) * powerMultiplier;
                rotation = Math.pow(driver.getRotation(), 3) * powerMultiplier;
                TeleOP.isNoInput = driver.isNoInput() && !rotationError;

                //Mechanism Code
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

                aimRotation = 0.0;
                isAimed = false;
                boolean foundTargetTag = false;

                for (AprilTagDetection detection : detections) {
                    //BIOBUZZ Hive tags return as AprilTagClusterDetection objects
                    if (detection instanceof AprilTagClusterDetection) {
                        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

                        if (cluster.metadata != null) {
                            //Filter for the selected alliance (RED vs BLUE)
                            String name = cluster.metadata.name.toUpperCase();
                            boolean isCorrectAlliance = redAllianceTarget ? name.contains("RED") : name.contains("BLUE");

                            if (isCorrectAlliance) {
                                if (cluster.ftcPose != null) {
                                    //Ignore the upside-down (unscorable) Hive cell on the far side
                                    boolean isUpright = Math.abs(cluster.ftcPose.roll) < 90.0;

                                    if (isUpright) {
                                        foundTargetTag = true;

                                        //Update Distance
                                        distance = cluster.ftcPose.range; // Point-to-point distance (inches)

                                        //Update Auto-Aim
                                        //Negate bearing so targets to the right produce positive (CW) values
                                        double rawBearingCW = -cluster.ftcPose.bearing;
                                        double error = rawBearingCW + BEARING_OFFSET_DEG;

                                        //Update isAimed regardless of whether the right stick is pressed
                                        isAimed = Math.abs(error) <= AIM_TOLERANCE_DEG;

                                        //Calculate proportional rotation only when right stick is pressed
                                        if (gamepad1.right_stick_button) {
                                            double rawOutput = error * AIM_COEFFICIENT;
                                            aimRotation = Math.max(-1.0, Math.min(1.0, rawOutput));
                                        }

                                        break; // Stop looping once we process our valid alliance target
                                    }
                                }
                            }
                        }
                    }
                }

                if (driver.getDpadUpPressed()) {
                    RPM += 50;
                } else if (driver.getDpadDownPressed()) {
                    RPM -= 50;
                }

                if (driver.getLeftBumperToggle()) { intakePower = 1.0; } else { intakePower = 0.0; }

                double transferPower = driver.getRightTrigger() * 0.3;
                if (driver.getB()) { transferPower *= 3; }

                if (RPM > MAX_RPM) { RPM = MAX_RPM; }

                if (driver.getAToggle()) {
                    flywheelPower = RPM / MAX_RPM;
                } else {
                    flywheelPower = 0;
                }

                intake.setPower(intakePower);
                transfer.setPower(transferPower);
                flywheel.setPower(flywheelPower);
                //End Mechanism Code

                if (driver.getLeftBumper()) {
                    TeleOP.isNoInput = false;
                    drive.xLock();
                    drive.update();
                } else {
                    if (TeleOP.fieldCentric || TeleOP.holdHeading) {
                        heading = odometry.getHeading();

                        if (TeleOP.fieldCentric) {
                            angle -= heading;
                        }

                        if (TeleOP.holdHeading) {
                            if (Math.abs(rotation) > 0.05) {
                                desiredHeading = heading;
                            } else {
                                double error = desiredHeading - heading;
                                while (error > 180) error -= 360;
                                while (error < -180) error += 360;
                                rotation = error * 0.02;
                                rotationError = Math.abs(error) > 5;
                            }
                        }
                    }
                    if (!driver.getRightStickPressed()) { //This statement is a mechanism control. It can be deleted as it is not needed for the drivetrain
                        drive.drive(angle, power, rotation);
                    } else {
                        desiredHeading = heading;
                        TeleOP.isNoInput = false;
                        drive.drive(0, 0, aimRotation);
                    }
                    drive.update();
                }

                if (driver.getDpadRightPressed()) {
                    if (selectedPod == fl) {
                        selectedPod = fr;
                        SelectedPod = "Front Right";
                    } else if (selectedPod == fr) {
                        selectedPod = rl;
                        SelectedPod = "Rear Left";
                    } else if (selectedPod == rl) {
                        selectedPod = rr;
                        SelectedPod = "Rear Right";
                    } else {
                        selectedPod = fl;
                        SelectedPod = "Front Left";
                    }
                } else if (driver.getDpadLeftPressed()) {
                    if (selectedPod == fl) {
                        selectedPod = rr;
                        SelectedPod = "Rear Right";
                    } else if (selectedPod == fr) {
                        selectedPod = fl;
                        SelectedPod = "Front Left";
                    } else if (selectedPod == rl) {
                        selectedPod = fr;
                        SelectedPod = "Front Right";
                    } else {
                        selectedPod = rl;
                        SelectedPod = "Rear Left";
                    }
                }

                if (driver.getYPressed()) {
                    selectedPod.rollover += 1;
                } else if (driver.getXPressed()) {
                    selectedPod.rollover -= 1;
                }

                SwervePod.saveRollover = driver.getBToggle();
                TeleOP.resetPods = !driver.getRightBumper();
                TeleOP.fieldCentric = driver.getPSToggle();
                TeleOP.holdHeading = driver.getStartToggle();
                TeleOP.updateOdometry = driver.getBackToggle();

                telemetry.addData("Playback time", "%.2f / %.2f s",
                        runtime.seconds(), MAX_PLAYBACK_SECONDS);
                telemetry.addData("Sample", "%d / %d",
                        frameIndex + 1, frames.size());
                telemetry.addData("Selected Pod", selectedPod);
                telemetry.addData("Pod Angle", "%.2f°", selectedPod.getPodAngle());
                telemetry.addData("Angle", "%.2f°", angle);
                telemetry.addData("Power", "%.3f", power);
                telemetry.addData("Rotation", "%.3f", rotation);
                telemetry.addData("Field Centric", TeleOP.fieldCentric);
                telemetry.addData("Hold Heading", TeleOP.holdHeading);
                telemetry.addData("IMU Heading", "%.2f°", odometry.getHeading());
                telemetry.addData("Odometry (X, Y)",
                        "%.3f, %.3f", odometry.getSDX(), odometry.getSDY());
                telemetry.update();
            }
        } finally {
            // Do not leave the drivetrain powered if playback ends or Stop is pressed.
            stopDrive();

            fl.saveRollover(fl.getRollover());
            fr.saveRollover(fr.getRollover());
            rl.saveRollover(rl.getRollover());
            rr.saveRollover(rr.getRollover());
        }
    }
}