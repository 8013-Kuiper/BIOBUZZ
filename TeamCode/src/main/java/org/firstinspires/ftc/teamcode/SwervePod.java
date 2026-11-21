package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class SwervePod {
    private CRServo turnServo;
    private DcMotorEx driveMotor;
    private AnalogInput analog;
    private String name;
    private int motorDirection;

    double voltage; //This holds the voltage reading
    int rollover = 0; //Holds the number of times the servo has crossed the rollover threshold
    double continuousVoltage = 0; //Holds the continuous voltage
    double lastVoltage = 0; //This stores the voltage from the last loop
    double lastDelta = 0; //This stores the change in voltage from the last loop
    double servoAngle; //The angle of the servo
    int savedRollover; //The current rollover saved to disk
    double podRPM; //Holds the approximate pod RPM for the purpose of spinning the motor the same amount in the same direction
    int direction; //This is used to reverse the motor direction
    double error; //This is the difference between the servo target and the servo angle
    double lastMotorPower = 0;

    //Telemetry Variables
    boolean isBusy; //If the servo error is greater than 30 degrees this value is true
    int saveCount = 0; //The number of times the rollover is written to disk
    //End Telemetry Variables

    //Odometry Variables
    int lastEncoder = 0; //Holds the encoder position from the last loop
    double deltaX; //Holds the change in robot-centric X position since the last read
    double deltaY; //Holds the change in robot-centric Y position since the last read
    //End Odometry Variables

    //Control System Variables
    double servoTarget = 0; //The target angle of the servo
    public static boolean saveRollover = true; //Determines if the rollover should be saved to disk
    //End Control System Variables

    //Hardware Information Variables
    private static final double MAX_VOLTAGE = 3.273; //The maximum voltage the analog output reaches before rolling over
    //private static final double GEAR_RATIO = -25.0 / 12.0; //The ratio between the servo and the pod, negative b/c gears reverse rotation
    private static final double GEAR_RATIO = -25.0 / 12.0; //The ratio between the servo and the pod (negative b/c gears flip direction)
    private static final double ABSOLUTE_GEAR_RATIO = Math.abs(GEAR_RATIO);
    private static final double DRIVE_GEAR_RATIO = 3.0 / 2.0; //The ratio between the motor and the wheel
    private static final double WHEEL_DIAMETER = 72.0; //Wheel diameter in millimeters
    private static final double ENCODER_RESOLUTION = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 17.0))) * 28.0); //ticks per revolution
    private static final double MAX_RPM = 435.0; //Max Motor RPM
    private static final double SERVO_RPM = 86.96; //Max Servo RPM (86.96 for 6V, 71.43 for 4.8V)
    private static final int DIRECTION = -1; //Servo rotation direction with positive power, -1 for CCW and 1 for CW
    private static final double MAX_ACCELERATION = 1; //How much the power can ramp up during one second
    //End Hardware Information Variables

    //Tuning Variables
    private static final double ERROR_THRESHOLD = 120.0; //Servo angle threshold where power scales down linearly by CORRECTION_COEFFICIENT * error
    private double CORRECTION_COEFFICIENT = 0.005;
    //End Tuning Variables

    public SwervePod(String name, CRServo turnServo, DcMotorEx driveMotor, AnalogInput analog, double CORRECTION_COEFFICIENT) {
        this.turnServo = turnServo;
        this.driveMotor = driveMotor;
        this.analog = analog;
        this.name = name;
        this.CORRECTION_COEFFICIENT = CORRECTION_COEFFICIENT;
        if (driveMotor.getDirection().equals(DcMotor.Direction.FORWARD)) {
            this.motorDirection = 1;
        } else {
            this.motorDirection = -1;
        }
    }

    public void update() { //Run this every loop
        voltage = analog.getVoltage();
        servoAngle = (getContinuousVoltage() / MAX_VOLTAGE) * 360.0;
        if (!isBusy) {
            updateOdometry();
        }
    }

    public double getContVoltage() {
        return continuousVoltage;
    }

    public double getServoAngle() { //Voltage to Servo Angle
        return servoAngle;
    }

    public double getPodAngle() { //Servo Angle to Pod Angle
        return getServoAngle() / GEAR_RATIO;
    }

    public double getServoVoltage() {
        return voltage;
    }

    //File Saving/Access Methods
    //These methods handle the saving of the rollover value to disk, so that in the event of a power failure or the robot being turned off, the robot will remember where the pods are to zero them
    private File getFile() {
        return AppUtil.getInstance().getSettingsFile(name + "_rollover.txt");
    }
    public int loadRollover() {
        try {
            Scanner scanner = new Scanner(getFile());
            int value = scanner.nextInt();
            scanner.close();

            //this exists because if the pod was jostled a little bit when powered off it could cause it to roll over
            if (value == -1 && voltage < MAX_VOLTAGE / 8) {
                value = 0;
            } else if (value == 0 && voltage > MAX_VOLTAGE - MAX_VOLTAGE / 8){
                value = -1;
            }
            return value;
        } catch (Exception e) {
            return 0; //default if file missing
        }
    }

    public void saveRollover(int rollover) {
        try {
            FileWriter writer = new FileWriter(getFile()); //overwrites
            writer.write(String.valueOf(rollover));
            savedRollover = rollover;
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void readLogFile() {
        rollover = loadRollover();
        savedRollover = rollover;
    }

    public void updateSavedRollover() {
        //This statement determines if the current rollover should be saved
        if (saveRollover) {
            //We don't want to have too many writes to disk, as not only can it wear down the disk but also slow the program.
            if (rollover != savedRollover) {
                saveRollover(rollover);
                saveCount++;
            }
        }
    }
    //End File Saving/Access Methods

    //This method handles the voltage wraparound, which keeps track of the number of times the servo's analog output has 'rolled over' from 0V-3.3V or 3.3V-0V
    public double getContinuousVoltage() {
        double current = getServoVoltage();
        double delta = current - lastVoltage;

        if (Math.abs(lastDelta) > MAX_VOLTAGE * 0.2 && Math.abs(delta) > MAX_VOLTAGE * 0.2 && Math.signum(lastDelta) == Math.signum(delta)) { delta = delta + lastDelta; }

        //Handle rollover
        if (delta > MAX_VOLTAGE * 0.5) {
            if (lastDelta <= MAX_VOLTAGE * 0.5) {
                rollover -= 1;
            }
        } else if (delta < -MAX_VOLTAGE * 0.5) {
            if (lastDelta >= -MAX_VOLTAGE * 0.5) {
                rollover += 1;
            }
        }

        continuousVoltage = current + (MAX_VOLTAGE * rollover);
        updateSavedRollover();

        lastVoltage = current;
        lastDelta = delta;
        return continuousVoltage;
    }

    public void setTargetAngle(double podAngle) {
        servoTarget = (podAngle * GEAR_RATIO);
    }

    //Troubleshooting Getters
    public int getRollover() {
        return rollover;
    }
    public double getError() {
        return error;
    }
    //End Troubleshooting Getters

    //Begin Odometry Methods
    public void updateOdometry() {
        int currentEncoder = driveMotor.getCurrentPosition();
        int deltaTicks = currentEncoder - lastEncoder;
        double podAngle = getPodAngle();
        double deltaDistance = ticksToDistance(deltaTicks);
        deltaX += deltaDistance * Math.sin(Math.toRadians(podAngle)); //Accumulates b/c the delta might not be polled every loop
        deltaY += deltaDistance * Math.cos(Math.toRadians(podAngle));

        lastEncoder = currentEncoder;
    }
    public double ticksToDistance(int ticks) { //Returns distance in feet
        double circumference = (WHEEL_DIAMETER / 304.8) * Math.PI;
        return (ticks / ENCODER_RESOLUTION) * circumference / DRIVE_GEAR_RATIO;
    }
    public double getDeltaX() {
        double temp = deltaX;
        deltaX = 0;
        return temp;
    }
    public double getDeltaY() {
        double temp = deltaY;
        deltaY = 0;
        return temp;
    }
    //End Odometry Methods

    //Begin Control Methods
    public void updateTurning() {
        double current = getServoAngle();
        current = current % (360 * ABSOLUTE_GEAR_RATIO); //normalize to one full pod rotation in degrees
        error = servoTarget - current;

        //Handles situations where if the pod has to rotate over 180 degrees, it can go angle - 360 the other way (e.g. -90 instead of 270)
        if (Math.abs(error) > 200 * ABSOLUTE_GEAR_RATIO) {
            if (error > 0) {
                error -= 360 * ABSOLUTE_GEAR_RATIO;
            } else {
                error += 360 * ABSOLUTE_GEAR_RATIO;
            }
        }

        //Handles situations where if the pod needs to rotate more than 90 degrees, it can rotate angle - 180 and reverse the motor speed (e.g. -45 instead of 135)
        direction = 1;
        if (Math.abs(error) > 95 * ABSOLUTE_GEAR_RATIO) {
            if (error > 0) {
                error -= 180 * ABSOLUTE_GEAR_RATIO;
            } else {
                error += 180 * ABSOLUTE_GEAR_RATIO;
            }
            direction = direction * -1;
        }

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            power = Math.signum(error);
        } else if (Math.abs(error) > 2){
            power = CORRECTION_COEFFICIENT * error;
        } else {
            power = 0.0;
        }

        if (Math.abs(error) > 30) {
            isBusy = true;
        } else {
            isBusy = false;
        }

        if (TeleOP.isNoInput && TeleOP.resetPods) {
            if (getContVoltage() > 1.3) {
                power = -1;
            } else if (getContVoltage() < -1.3) {
                power = 1;
            } else {
                power = -CORRECTION_COEFFICIENT * getServoAngle();
            }
        }

        turnServo.setPower(power * DIRECTION);
        podRPM = power * (SERVO_RPM / GEAR_RATIO);
    }

    public void setDrivePower(double power) {
        if (TeleOP.updateOdometry) {
            //Check if we are accelerating (magnitude is increasing in the same direction)
            boolean isAcceleratingForward = (power > 0 && lastMotorPower >= 0 && (power - lastMotorPower > MAX_ACCELERATION)); //* TeleOP.deltaTime));
            boolean isAcceleratingReverse = (power < 0 && lastMotorPower <= 0 && (lastMotorPower - power > MAX_ACCELERATION)); //* TeleOP.deltaTime));

            if (isAcceleratingForward) {
                power = lastMotorPower + MAX_ACCELERATION * TeleOP.deltaTime;
            } else if (isAcceleratingReverse) {
                power = lastMotorPower - MAX_ACCELERATION * TeleOP.deltaTime;
            }
            power = Math.max(-1.0, Math.min(1.0, power));
        }
        //Apply power and accurately track the actual output
        if (isBusy) {
            driveMotor.setPower((podRPM / MAX_RPM) * -motorDirection);
            lastMotorPower = 0;
        } else {
            driveMotor.setPower(power * direction + (podRPM / MAX_RPM) * -motorDirection);
            lastMotorPower = power;
        }
    }
}