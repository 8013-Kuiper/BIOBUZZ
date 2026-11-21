package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Test Auto")
public class TestAutonomous extends LinearOpMode {

    //Declaring Starting Positions
    double startX = 0;
    double startY = 0;
    double startAngle = 0;

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
    private final ElapsedTime runtime = new ElapsedTime();

    //Declaring Variables
    int step = 0;
    double time = runtime.seconds();

    @Override
    public void runOpMode() throws InterruptedException{
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

        TeleOP.updateOdometry = true;
        SwervePod.saveRollover = true;
        TeleOP.resetPods = false;

        //Initialize Odometry
        Odometry odometry = new Odometry(fl, fr, rl, rr, drive, imu);
        odometry.setStartingPosition(startX, startY, startAngle);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Read logfiles
        fl.readLogFile();
        fr.readLogFile();
        rl.readLogFile();
        rr.readLogFile();

        waitForStart();

        //Autonomous Loop
        while (opModeIsActive()) {


            fl.update(); //Update pods
            fr.update();
            rl.update();
            rr.update();
            odometry.update();

            switch (step) {

                case 0:
                    if (odometry.gotoPosition(0, 2, 0, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;

                case 1:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 2:
                    if (odometry.gotoPosition(0, 0, 0, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;

                case 3:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 4:
                    if (odometry.gotoPosition(-2, 0, 0, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;

                case 5:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 6:
                    if (odometry.gotoPosition(0, 0, 0, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;


                case 7:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 8:
                    if (odometry.gotoPosition(0, 0, 90, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;

                case 9:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 10:
                    if (odometry.gotoPosition(-2, 2, 0, 0.3))
                        step++;
                    time = runtime.seconds();
                    break;

                case 11:
                    if (runtime.seconds() - time > 1)
                        step ++;
                    break;

                case 12:
                    if (odometry.gotoPosition(0, 0, 0, 0.5))
                        step++;
                    time = runtime.seconds();
                    break;

                case 13: //Always do this last
                    drive.drive(0, 0,  0);
                    TeleOP.resetPods = true;
                    step ++;
                    break;
            }

            drive.update();

            telemetry.addData("Encoder Position (X, Y, Angle)", ((Math.round((odometry.getSDX()) * 1000.0) / 1000.0 + " ft, ") + (Math.round((odometry.getSDY()) * 1000.0) / 1000.0 + " ft ") + (odometry.getSDHeading() + "°")));
            telemetry.addData("IMU Heading", (odometry.getHeading() + "°"));
            telemetry.update();
        }
    }
}