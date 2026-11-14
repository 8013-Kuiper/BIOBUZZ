package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class DriveConstants extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

        public void initRobot() {
            frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
            backLeft = hardwareMap.dcMotor.get("backLeftMotor");
            frontRight = hardwareMap.dcMotor.get("frontRightMotor");
            backRight = hardwareMap.dcMotor.get("backRightMotor");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
}
