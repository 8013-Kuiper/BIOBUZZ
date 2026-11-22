package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(group = "A", name = "TeleOp")

public class TeleOP extends LinearOpMode {

    //Declaring The Drivetrain Motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    //Declaring The Servo Motors
    CRServo frontLeftCR;
    CRServo frontRightCR;
    CRServo rearLeftCR;
    CRServo rearRightCR;

    //Declaring Analog Inputs
    AnalogInput frontLeftPos;
    AnalogInput frontRightPos;
    AnalogInput rearLeftPos;
    AnalogInput rearRightPos;

    //Declaring Sensors
    IMU imu;

    //Declaring Variables
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
    public static double deltaTime = 0.0;
    public static final double POWER_SUBTRACTOR = 0.0; //This is how much power will be subtracted when the right trigger isn't pressed

    //Robot Information
    public static final double LENGTH = 14.25; //distance from wheel center to wheel center, inches
    public static final double WIDTH = 14.25;

    //Mechanism Declarations
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx flywheel;
    double intakePower;
    double flywheelPower;
    double distance = 59;

    double RPM = 2500.0;
    double xRPM = 4300.0;
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
        //Instructions that run once upon initialization go here

        //Mapping the drivetrain motors to their ports
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRightMotor");

        //Setting the default direction for each motor (negative b/c direction mirrored)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //Setting the behavior for the motors. This way, the robot is harder to push
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Mapping the servos
        frontLeftCR = hardwareMap.get(CRServo.class, "frontLeftCR");
        frontRightCR = hardwareMap.get(CRServo.class, "frontRightCR");
        rearLeftCR = hardwareMap.get(CRServo.class, "rearLeftCR");
        rearRightCR = hardwareMap.get(CRServo.class, "rearRightCR");

        //Mapping the analog inputs
        frontLeftPos = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
        frontRightPos = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
        rearLeftPos = hardwareMap.get(AnalogInput.class, "rearLeftAnalog");
        rearRightPos = hardwareMap.get(AnalogInput.class, "rearRightAnalog");

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        //Reset Encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize Pods
        SwervePod fl = new SwervePod("frontLeft", frontLeftCR, frontLeft, frontLeftPos);
        SwervePod fr = new SwervePod("frontRight", frontRightCR, frontRight, frontRightPos);
        SwervePod rl = new SwervePod("rearLeft", rearLeftCR, rearLeft, rearLeftPos);
        SwervePod rr = new SwervePod("rearRight", rearRightCR, rearRight, rearRightPos);

        SwerveDrive drive = new SwerveDrive(fl, fr, rl, rr);

        SwervePod selectedPod = fl;
        String SelectedPod = "Front Left";

        //Initialize Controllers
        Controller driver = new Controller(gamepad1);
        Controller operator = new Controller(gamepad2);

        //Initialize Odometry
        Odometry odometry = new Odometry(fl, fr, rl, rr, drive, imu);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); }

        //Read logfiles
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

        while (!isStarted() && !isStopRequested()) {
            //Instructions to be repeated upon initialization until started
            telemetry.addData("Front Left Rollover", fl.savedRollover);
            telemetry.addData("Front Right Rollover", fr.savedRollover);
            telemetry.addData("Rear Left Rollover", rl.savedRollover);
            telemetry.addData("Rear Right Rollover", rr.savedRollover);
            telemetry.addLine();
            telemetry.addData("Front Left Voltage", fl.getServoVoltage());
            telemetry.addData("Front Right Voltage", fr.getServoVoltage());
            telemetry.addData("Rear Left Voltage", rl.getServoVoltage());
            telemetry.addData("Rear Right Voltage", rr.getServoVoltage());
            telemetry.update();
        }

        while (isStarted() && !isStopRequested()) {
            //Things to be repeated when play pressed
            driver.update(); //Update controllers
            operator.update();

            fl.update(); //Update pods
            fr.update();
            rl.update();
            rr.update();

            if (updateOdometry) { odometry.update(); } //Update odometry (if applicable)
            for (LynxModule hub : allHubs) { hub.clearBulkCache(); } //Clear cache

            powerSubtractor =  (1.0 - driver.getRightTrigger()) * POWER_SUBTRACTOR;
            powerMultiplier = Math.min((1.0 - (driver.getLeftTrigger() - 0.30)), 1.0) - powerSubtractor;
            //if (driver.getAToggle()) { powerMultiplier /= 2.0; }

            angle = driver.getAngle();
            power = Math.pow(driver.getResultant(), 2) * powerMultiplier;
            rotation = Math.pow(driver.getRotation(), 3) * powerMultiplier;
            isNoInput = driver.isNoInput() && !rotationError;

            //Mechanism Code
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            double realRPM = flywheel.getVelocity() / 28 * 60;
            boolean spunUp = Math.abs(realRPM - RPM) < 200.0;
            boolean xSpunUp = Math.abs(realRPM - xRPM) < 200.0;

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

            if (driver.getDpadUpPressed() || operator.getDpadUpPressed()) { RPM += 50; }
            else if (driver.getDpadDownPressed() || operator.getDpadDownPressed()) { RPM -= 50; }
            if (operator.getDpadLeftPressed()) { xRPM -= 50; }
            else if (operator.getDpadRightPressed()) { xRPM += 50; }

            if (driver.getLeftBumperToggle()) { intakePower = 1.0; } else { intakePower = 0.0; }
            intakePower = Math.max(intakePower, operator.getLeftTrigger());
            if (operator.getLeftBumper()) { intakePower *= -1.0; }

            double transferPower = driver.getRightTrigger() * 0.3;
            transferPower = Math.max(transferPower, operator.getRightTrigger());
            if (driver.getB() || operator.getB()) { transferPower *= 3.0; }
            if (operator.getRightBumper()) { transferPower *= -1.0; }
            if (operator.getA() && !spunUp && !driver.getAToggle()) { transferPower = 0; }
            if (operator.getX() && !xSpunUp && !driver.getAToggle()) { transferPower = 0; }

            if (RPM > MAX_RPM) { RPM = MAX_RPM; }
            if (driver.getAToggle() || operator.getA()) { flywheelPower = RPM / MAX_RPM; }
            else { flywheelPower = 0; }
            if (operator.getX()) { flywheelPower = xRPM / MAX_RPM; }
            if (operator.getY()) { flywheelPower = -0.2; }

            intake.setPower(intakePower);
            transfer.setPower(transferPower);
            flywheel.setPower(flywheelPower);

            telemetry.addLine("-----Mechanism Info-----");
            telemetry.addData("Target RPM (A)", Math.round(RPM));
            telemetry.addData("Real RPM", (flywheel.getVelocity() / 28 * 60));
            telemetry.addLine();
            //End Mechanism Code

            if (driver.getLeftBumper()) {
                isNoInput = false;
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
                    isNoInput = false;
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

            //SwervePod.saveRollover = driver.getBToggle();
            resetPods = !driver.getRightBumper();
            fieldCentric = driver.getPSToggle();
            holdHeading = driver.getStartToggle();
            redAllianceTarget = !driver.getShareToggle();

            //temporary data for telemetry and troubleshooting
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
        fl.saveRollover(fl.getRollover());
        fr.saveRollover(fr.getRollover());
        rl.saveRollover(rl.getRollover());
        rr.saveRollover(rr.getRollover());
    }
}