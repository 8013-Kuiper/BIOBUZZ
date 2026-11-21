package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.List;

@TeleOp(group = "A", name = "TeleOP")

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
    public static final double POWER_SUBTRACTOR = 0.0; //This is how much power will be subtracted when the right trigger isn't pressed

    //Robot Information
    public static final double LENGTH = 14.25; //distance from wheel center to wheel center, inches
    public static final double WIDTH = 14.25;
    public static final double MK1_COEFFICIENT = 0.005;
    public static final double MK2_COEFFICIENT = 0.002;

    //Mechanism Declarations
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx flywheel;

    double RPM = 3000.0;
    public final double MAX_RPM = 5000.0;
    public static double deltaTime;

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
        SwervePod fl = new SwervePod("frontLeft", frontLeftCR, frontLeft, frontLeftPos, MK1_COEFFICIENT);
        SwervePod fr = new SwervePod("frontRight", frontRightCR, frontRight, frontRightPos, MK2_COEFFICIENT);
        SwervePod rl = new SwervePod("rearLeft", rearLeftCR, rearLeft, rearLeftPos, MK2_COEFFICIENT);
        SwervePod rr = new SwervePod("rearRight", rearRightCR, rearRight, rearRightPos, MK1_COEFFICIENT);

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

        while (!isStarted()) {
            //Instructions to be repeated upon initialization until started
            telemetry.addData("Front Left Rollover", fl.savedRollover);
            telemetry.addData("Front Right Rollover", fr.savedRollover);
            telemetry.addData("Rear Left Rollover", rl.savedRollover);
            telemetry.addData("Rear Right Rollover", rr.savedRollover);
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
            if (driver.getAToggle()) { powerMultiplier /= 2.0; }

            angle = driver.getAngle();
            power = Math.pow(driver.getResultant(), 2) * powerMultiplier;
            rotation = Math.pow(driver.getRotation(), 3) * powerMultiplier;
            isNoInput = driver.isNoInput() && !rotationError;

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
                drive.drive(angle, power, rotation);
                drive.update();
            }

            if (driver.getDpadUpPressed()) {
                selectedPod = fl;
                SelectedPod = "Front Left";
            } else if (driver.getDpadDownPressed()) {
                selectedPod = rr;
                SelectedPod = "Rear Right";
            } else if (driver.getDpadLeftPressed()) {
                selectedPod = rl;
                SelectedPod = "Rear Left";
            } else if (driver.getDpadRightPressed()) {
                selectedPod = fr;
                SelectedPod = "Front Right";
            }
            if (driver.getYPressed()) {
                selectedPod.rollover += 1;
            } else if (driver.getXPressed()) {
                selectedPod.rollover -= 1;
            }

            SwervePod.saveRollover = driver.getBToggle();
            resetPods = !driver.getRightBumper();
            fieldCentric = driver.getPSToggle();
            holdHeading = driver.getStartToggle();
            updateOdometry = driver.getBackToggle();

            //Mechanism Code
            double intakePower = operator.getLeftTrigger();
            double transferPower = operator.getLeftY() * 0.3;
            double flywheelPower = operator.getRightTrigger();
            RPM += operator.getRightY();
            if (operator.getX()) { transferPower *= 3; }

            if (RPM > MAX_RPM) { RPM = MAX_RPM; }

            if (operator.getAToggle()) {
                flywheelPower = RPM / MAX_RPM;
            }

            if (operator.getRightBumper()) {
                flywheelPower = -0.1;
            }
            if (operator.getLeftBumper()) {
                intakePower = -0.3;
            }

            intake.setPower(intakePower);
            transfer.setPower(transferPower);
            flywheel.setPower(flywheelPower);

            telemetry.addLine("-----Mechanism Info-----");
            telemetry.addData("Target RPM", Math.round(RPM));
            telemetry.addData("Real RPM", (flywheel.getVelocity() / 28 * 60));
            telemetry.addLine();

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