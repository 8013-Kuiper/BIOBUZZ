package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Odometry {
    private SwervePod fl, fr, rl, rr;
    private SwerveDrive drive;
    private IMU imu;
    public double startingAngle = 0; //degrees
    public double startingYPosition = 0;
    public double startingXPosition = 0; //feet
    private double SDx;
    private double SDy;
    private double heading = 0;
    private double SDHeading = 0;
    private int stallCount = 0;
    public int headingMethod = 1; //0 for Encoder, 1 for IMU
    public ElapsedTime runTime = new ElapsedTime();
    private static final double LENGTH = TeleOP.LENGTH; //distance from wheel center to wheel center, inches
    private static final double WIDTH = TeleOP.WIDTH;
    private static boolean[] enabledPods = {
            true,  // FL
            false, // FR
            false, // RL
            true   // RR
    };
    public Odometry(SwervePod fl, SwervePod fr, SwervePod rl, SwervePod rr, SwerveDrive Drive, IMU imu) {
        this.fl = fl;
        this.fr = fr;
        this.rl = rl;
        this.rr = rr;
        this.drive = Drive;
        this.imu = imu;
    }

    public void setStartingPosition(double x, double y, double angle) {
        startingAngle = angle;
        startingXPosition = x;
        startingYPosition = y;
    }

    public void update() {
        //Find position based off of IMU or Encoders
        double headingRad = 0;
        if (headingMethod == 0) {
            headingRad = Math.toRadians(getSDHeading());
        } else if (headingMethod == 1){
            headingRad = Math.toRadians(getHeading());
        }

        double flDeltaX = fl.getDeltaX();
        double flDeltaY = fl.getDeltaY();
        double frDeltaX = fr.getDeltaX();
        double frDeltaY = fr.getDeltaY();
        double rlDeltaX = rl.getDeltaX();
        double rlDeltaY = rl.getDeltaY();
        double rrDeltaX = rr.getDeltaX();
        double rrDeltaY = rr.getDeltaY();

        //Wheel motion vectors
        double[][] wheelDelta = {
                {flDeltaX, flDeltaY},
                {frDeltaX, frDeltaY},
                {rlDeltaX, rlDeltaY},
                {rrDeltaX, rrDeltaY}
        };

        //Find position based off of motor encoders
        double SDdxRobot = 0;
        double SDdyRobot = 0;

        int count = 0;
        for (int k = 0; k < 4; k ++) {
            if (!enabledPods[k])
                continue;
            SDdxRobot += wheelDelta[k][0];
            SDdyRobot += wheelDelta[k][1];
            count ++;
        }

        SDdxRobot /= count;
        SDdyRobot /= count;

        double SDdxField = SDdxRobot * Math.cos(headingRad) + SDdyRobot * Math.sin(headingRad);
        double SDdyField = -SDdxRobot * Math.sin(headingRad) + SDdyRobot * Math.cos(headingRad);

        SDx += SDdxField;
        SDy += SDdyField;

        //Find angle based on encoder position
        //Wheel positions relative to center
        double[][] wheelPos = {
                { LENGTH / 24.0,  WIDTH / 24.0}, // FL - divided by 24 to convert inches to feet (12) and halve (2)
                { LENGTH / 24.0, -WIDTH / 24.0}, // FR
                {-LENGTH / 24.0,  WIDTH / 24.0}, // RL
                {-LENGTH / 24.0, -WIDTH / 24.0}  // RR
        };

        //Radius from center to wheel
        double radius = Math.sqrt(Math.pow(LENGTH / 24.0, 2) + Math.pow(WIDTH / 24.0, 2));

        int acc = 0;
        double totalTheta = 0;
        for (int i = 0; i < 4; i++) {

            if (!enabledPods[i])
                continue;

            double tangentAngle = 90 - Math.toDegrees(Math.atan2(wheelPos[i][0], wheelPos[i][1]));

            double dx = wheelDelta[i][0];
            double dy = wheelDelta[i][1];

            double resultant = Math.sqrt(dx * dx + dy * dy);
            double angle = Math.toDegrees(Math.atan2(dy, dx));
            double rotationVector = resultant * Math.cos(Math.toRadians(angle - tangentAngle));

            double dTheta = rotationVector / radius;

            totalTheta += dTheta;
            acc++;
        }
        SDHeading += Math.toDegrees(totalTheta / acc);
    }

    //Tune these values as needed
    double positionThreshold = 0.1;
    double headingThreshold = 2.0;
    final double POSITION_CORRECTION_COEFFICIENT = 0.8;
    final double ANGLE_CORRECTION_COEFFICIENT = 0.02;
    final double STALL_THRESHOLD = 0.05; //Speed at which the robot (if it hasn't reached its spot) will boost power to do so (ft/s)
    final double ANGULAR_STALL_THRESHOLD = 3; //Same as STALL_THRESHOLD but in degrees/sec
    final double STOP_THRESHOLD = 0.2; //Speed at which the robot has to slow to before moving on (to prevent overshooting) (ft/s)
    final double POWER_BOOST = 3.0; //How much the power is boosted during a stall
    double oldX = startingXPosition;
    double oldY = startingYPosition;
    double oldHeading = startingAngle;
    double oldTime = 0;
    double deltaTime = 0;
    public boolean gotoPosition(double x, double y, double heading, double power) {
        //movement logic
        while (heading > 180)
            heading -= 360;
        while (heading < -180)
            heading += 360;

        double time = runTime.seconds();
        deltaTime = time - oldTime;
        TeleOP.deltaTime = deltaTime;

        double currentHeading = getHeading();
        if (headingMethod == 0) {
            currentHeading = getSDHeading();
        } else if (headingMethod == 1){
            currentHeading = getHeading();
        }

        double errorX = x - getSDX();
        double errorY = y - getSDY();
        double errorAngle = heading - currentHeading;

        while (errorAngle > 180)
            errorAngle -= 360;

        while (errorAngle < -180)
            errorAngle += 360;

        double velocityX = (getSDX() - oldX) / (time - oldTime);
        double velocityY = (getSDY() - oldY) / (time - oldTime);
        double velocity = Math.sqrt(velocityX * velocityX + velocityY * velocityY);
        double angularVelocity = (currentHeading - oldHeading) / (time - oldTime);

        if (!(Math.abs(errorX) > positionThreshold || Math.abs(errorY) > positionThreshold || Math.abs(errorAngle) > headingThreshold)) {
            drive.drive(0, 0, 0);
            drive.update();
            if (velocity < STOP_THRESHOLD) {
                TeleOP.isNoInput = true;
                return true;
            }
        } else { TeleOP.isNoInput = false; }

        double powerX = 0;
        if (Math.abs(errorX) > positionThreshold) {
            powerX = POSITION_CORRECTION_COEFFICIENT * errorX;
            if (Math.abs(velocityX) < STALL_THRESHOLD) { powerX *= POWER_BOOST * (stallCount / 10.0 + 1); }
        }
        double powerY = 0;
        if (Math.abs(errorY) > positionThreshold) {
            powerY = POSITION_CORRECTION_COEFFICIENT * errorY;
            if (Math.abs(velocityY) < STALL_THRESHOLD) { powerY *= POWER_BOOST * (stallCount / 10.0 + 1); }
        }
        double powerRotation = 0;
        if (Math.abs(errorAngle) > headingThreshold) {
            powerRotation = ANGLE_CORRECTION_COEFFICIENT * errorAngle;
            if (Math.abs(angularVelocity) < ANGULAR_STALL_THRESHOLD) { powerRotation *= POWER_BOOST * (stallCount / 10.0 + 1); }
        }
        if (Math.abs(errorX) < STALL_THRESHOLD || Math.abs(errorY) < STALL_THRESHOLD || Math.abs(angularVelocity) < ANGULAR_STALL_THRESHOLD) {
            stallCount ++;
        } else {
            stallCount = 0;
        }

        double angle = (Math.toDegrees(Math.atan2(powerY, -powerX)) - 90);
        double resultant = Math.min(Math.sqrt(Math.pow(powerX, 2) + Math.pow(powerY, 2)), power);
        double rotation = Math.min(powerRotation, power);
        rotation = Math.max(rotation, -power);

        angle -= currentHeading;

        drive.drive(angle, resultant, rotation);
        drive.update();

        oldTime = time;
        oldX = getSDX();
        oldY= getSDY();
        oldHeading = currentHeading;

        return false;
    }

    public double getSDX() { //Returns the encoder displacement in the x direction in inches
        return SDx + startingXPosition;
    }
    public double getSDY() { //Returns the encoder displacement in the y direction in inches
        return SDy + startingYPosition;
    }
    public double getSDHeading() {
        return (SDHeading + startingAngle) % 360; }
    public double getHeading() { //degrees
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = -orientation.getYaw(AngleUnit.DEGREES) + startingAngle;
        return heading;
    }
    public double getDeltaTime() {
        return deltaTime;
    }
}