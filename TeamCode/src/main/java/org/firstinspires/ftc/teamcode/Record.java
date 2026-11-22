package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@TeleOp(group = "A", name = "TeleOP Record")
public class Record extends LinearOpMode {

    public static final String FILE_NAME = "RecordedAuto.txt";

    //Declaring the drivetrain motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    //Declaring the servo motors
    CRServo frontLeftCR;
    CRServo frontRightCR;
    CRServo rearLeftCR;
    CRServo rearRightCR;

    //Declaring analog inputs
    AnalogInput frontLeftPos;
    AnalogInput frontRightPos;
    AnalogInput rearLeftPos;
    AnalogInput rearRightPos;

    //Declaring sensors
    IMU imu;

    //Declaring variables
    double angle;
    double power;
    double rotation;

    //Control system variables
    double heading;
    double desiredHeading;
    boolean rotationError = false;
    double powerMultiplier;
    double powerSubtractor;
    public static boolean isNoInput = false;
    public static boolean resetPods = true;
    public static boolean updateOdometry = true;
    public static boolean fieldCentric = false;
    public static boolean holdHeading = true;
    public static final double POWER_SUBTRACTOR = 0.0;

    //Robot information
    public static final double LENGTH = 14.25;
    public static final double WIDTH = 14.25;

    public ElapsedTime runTime = new ElapsedTime();

    //Recording buffer: inputs stay in RAM until saveRecording() is called.
    private final StringBuilder recording = new StringBuilder(256 * 1024);
    private final ElapsedTime recordingTimer = new ElapsedTime();

    private void addRecordingHeader() {
        recording.append(
                "timeMs," +
                        "g1_leftX,g1_leftY,g1_rightX,g1_rightY,g1_leftTrigger,g1_rightTrigger," +
                        "g1_a,g1_b,g1_x,g1_y,g1_dpadUp,g1_dpadDown,g1_dpadLeft,g1_dpadRight," +
                        "g1_leftBumper,g1_rightBumper,g1_leftStickButton,g1_rightStickButton," +
                        "g1_back,g1_start,g1_guide,g1_share,g1_ps," +
                        "g2_leftX,g2_leftY,g2_rightX,g2_rightY,g2_leftTrigger,g2_rightTrigger," +
                        "g2_a,g2_b,g2_x,g2_y,g2_dpadUp,g2_dpadDown,g2_dpadLeft,g2_dpadRight," +
                        "g2_leftBumper,g2_rightBumper,g2_leftStickButton,g2_rightStickButton," +
                        "g2_back,g2_start,g2_guide,g2_share,g2_ps\n"
        );
    }

    private void appendGamepad(Gamepad g) {
        recording.append(String.format(Locale.US,
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f," +
                        "%b,%b,%b,%b,%b,%b,%b,%b," +
                        "%b,%b,%b,%b,%b,%b,%b,%b,%b,",
                g.left_stick_x, g.left_stick_y,
                g.right_stick_x, g.right_stick_y,
                g.left_trigger, g.right_trigger,
                g.a, g.b, g.x, g.y,
                g.dpad_up, g.dpad_down, g.dpad_left, g.dpad_right,
                g.left_bumper, g.right_bumper,
                g.left_stick_button, g.right_stick_button,
                g.back, g.start, g.guide, g.share, g.ps
        ));
    }

    private void recordInputs() {
        recording.append(String.format(Locale.US, "%.2f,", recordingTimer.milliseconds()));
        appendGamepad(gamepad1);
        appendGamepad(gamepad2);
        recording.append('\n');
    }

    private void saveRecording() {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file, false))) {
            writer.write(recording.toString());
            telemetry.addData("Recording saved", file.getAbsolutePath());
        } catch (IOException e) {
            telemetry.addData("Recording save error", e.getMessage());
            telemetry.addData("Error", e.toString());
        }
        telemetry.update();
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
        //Map drivetrain motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRightMotor");

        //Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //Set motor zero-power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Map servos
        frontLeftCR = hardwareMap.get(CRServo.class, "frontLeftCR");
        frontRightCR = hardwareMap.get(CRServo.class, "frontRightCR");
        rearLeftCR = hardwareMap.get(CRServo.class, "rearLeftCR");
        rearRightCR = hardwareMap.get(CRServo.class, "rearRightCR");

        //Map analog inputs
        frontLeftPos = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
        frontRightPos = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
        rearLeftPos = hardwareMap.get(AnalogInput.class, "rearLeftAnalog");
        rearRightPos = hardwareMap.get(AnalogInput.class, "rearRightAnalog");

        //Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        //Reset encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize pods and drivetrain
        SwervePod fl = new SwervePod("frontLeft", frontLeftCR, frontLeft, frontLeftPos);
        SwervePod fr = new SwervePod("frontRight", frontRightCR, frontRight, frontRightPos);
        SwervePod rl = new SwervePod("rearLeft", rearLeftCR, rearLeft, rearLeftPos);
        SwervePod rr = new SwervePod("rearRight", rearRightCR, rearRight, rearRightPos);

        SwerveDrive drive = new SwerveDrive(fl, fr, rl, rr);

        SwervePod selectedPod = fl;
        String SelectedPod = "Front Left";

        //Initialize controllers
        Controller driver = new Controller(gamepad1);
        Controller operator = new Controller(gamepad2);

        //Initialize odometry
        Odometry odometry = new Odometry(fl, fr, rl, rr, drive, imu);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Read rollover log files
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

        //Wait for Start while displaying rollover information
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Front Left Rollover", fl.savedRollover);
            telemetry.addData("Front Right Rollover", fr.savedRollover);
            telemetry.addData("Rear Left Rollover", rl.savedRollover);
            telemetry.addData("Rear Right Rollover", rr.savedRollover);
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        //Start the 30-second recording period when Start is pressed.
        recording.setLength(0);
        addRecordingHeader();
        runTime.reset();
        recordingTimer.reset();

        try {
            while (isStarted() && !isStopRequested() && runTime.seconds() < 30) {
                //Record both gamepads once per loop.
                recordInputs();

                //Update controllers
                driver.update();
                operator.update();

                //Update pods
                fl.update();
                fr.update();
                rl.update();
                rr.update();

                if (updateOdometry) {
                    odometry.update();
                }

                //Clear bulk cache
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                powerSubtractor = (1.0 - driver.getRightTrigger()) * POWER_SUBTRACTOR;
                powerMultiplier = Math.min((1.0 - (driver.getLeftTrigger() - 0.30)), 1.0) - powerSubtractor;
                if (driver.getAToggle()) {
                    powerMultiplier /= 2.0;
                }

                angle = driver.getAngle();
                power = Math.pow(driver.getResultant(), 2) * powerMultiplier;
                rotation = Math.pow(driver.getRotation(), 3) * powerMultiplier;
                TeleOP.isNoInput = driver.isNoInput() && !rotationError;

                //Mechanism Code
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

                double realRPM = flywheel.getVelocity() / 28 * 60;
                boolean spunUp = Math.abs(realRPM - RPM) < 200.0;

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

                telemetry.addLine("-----AprilTag-----");
                if (redAllianceTarget) { telemetry.addLine("Alliance : Red"); }
                else { telemetry.addLine("Alliance : Blue"); }
                telemetry.addData("Ready to Shoot", spunUp && isAimed);
                telemetry.addData("Is Aimed", isAimed);
                telemetry.addData("Found Tag", foundTargetTag);
                telemetry.addData("Distance", distance);
                telemetry.addLine();

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

                telemetry.addLine("-----Mechanism Info-----");
                telemetry.addData("Target RPM", Math.round(RPM));
                telemetry.addData("Real RPM", (flywheel.getVelocity() / 28 * 60));
                telemetry.addLine();
                //End Mechanism Code

                if (driver.getLeftBumper()) {
                    TeleOP.isNoInput = false;
                    drive.xLock();
                    drive.update();
                } else {
                    if (fieldCentric || holdHeading) {
                        heading = odometry.getHeading();
                        if (fieldCentric) {
                            angle -= heading;
                        }
                        if (holdHeading) {
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

                //Select pod
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

                //Adjust rollover
                if (driver.getYPressed()) {
                    selectedPod.rollover += 1;
                } else if (driver.getXPressed()) {
                    selectedPod.rollover -= 1;
                }

                SwervePod.saveRollover = driver.getBToggle();
                TeleOP.resetPods = !driver.getRightBumper();
                fieldCentric = driver.getPSToggle();
                holdHeading = driver.getStartToggle();
                TeleOP.updateOdometry = driver.getBackToggle();

                //Telemetry
                telemetry.addLine("-----Pod Direction-----");
                telemetry.addData("Selected Pod", SelectedPod);
                telemetry.addData("Voltage", (Math.round(selectedPod.getServoVoltage() * 1000) / 1000.0 + " Volts"));
                telemetry.addData("Cont. Voltage", (Math.round(selectedPod.getContVoltage() * 1000) / 1000.0 + " Volts"));
                telemetry.addData("Rollover Count", selectedPod.getRollover());
                telemetry.addData("Servo Position", (Math.round(selectedPod.getServoAngle() * 100) / 100.0 + "°"));
                telemetry.addData("Pod Position", (Math.round(selectedPod.getPodAngle() * 100) / 100.0 + "°"));
                telemetry.addLine();

                telemetry.addLine("--------Inputs--------");
                telemetry.addData("Angle", (Math.round(angle * 10) / 10.0 + "°"));
                telemetry.addData("Error", (Math.round(selectedPod.getError() * 100.0) / 100.0 + "°"));
                telemetry.addData("Direction", selectedPod.direction);
                telemetry.addData("Power", (Math.round(power * 100) / 100.0));
                telemetry.addData("Rotation", (Math.round(driver.getRotation() * 100) / 100.0));
                telemetry.addData("Field Centric, Heading Hold", (fieldCentric + ", " + holdHeading));
                telemetry.addLine();

                if (SwervePod.saveRollover) {
                    telemetry.addLine("------File Saving------");
                    telemetry.addData("Rollover Saving", SwervePod.saveRollover);
                    telemetry.addData("Save Count", selectedPod.saveCount);
                    telemetry.addData("Saved Rollover", selectedPod.savedRollover);
                    telemetry.addLine();
                }

                if (updateOdometry) {
                    telemetry.addLine("------Odometry------");
                    telemetry.addData("Heading", (odometry.getHeading() + "°"));
                    telemetry.addData("Encoder Heading", odometry.getSDHeading());
                    telemetry.addData("Encoder Position (X, Y)", ((Math.round((odometry.getSDX()) * 1000.0) / 1000.0 + " ft, ") + (Math.round((odometry.getSDY()) * 1000.0) / 1000.0 + " ft")));
                    telemetry.addLine();
                }
                telemetry.update();
            }
        } finally {
            //Save buffered input data whether the 30 seconds finish or Stop is pressed.
            saveRecording();
        }

        //Save pod rollover values as in the original class.
        fl.saveRollover(fl.getRollover());
        fr.saveRollover(fr.getRollover());
        rl.saveRollover(rl.getRollover());
        rr.saveRollover(rr.getRollover());
    }
}