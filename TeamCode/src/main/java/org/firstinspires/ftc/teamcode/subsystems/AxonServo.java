package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class AxonServo {
    private CRServo CRservo;
    private AnalogInput analog;
    private String name;
    private final ElapsedTime runtime = new ElapsedTime();
    public AxonServo(String Name, CRServo Servo, AnalogInput analog) {
        this.CRservo = Servo;
        this.analog = analog;
        this.name = Name;
    }

    //Variable declarations
    //Hardware Information Variables
    private static final double MAX_VOLTAGE = 3.273; //The maximum voltage the analog output reaches before rolling over
    private double SERVO_RPM = 71.43; //Max Servo RPM (86.96 for 6V, 71.43 for 4.8V)
    private int DIRECTION = -1; //Servo rotation direction with positive power, -1 for CCW and 1 for CW
    //End Hardware Information Variables

    //Tuning Variables - these may need to be tuned depending on how much load is on the servo
    private double ERROR_THRESHOLD = 80.0; //Servo angle threshold (in degrees) where power scales down linearly by CORRECTION_COEFFICIENT * error
    private double CORRECTION_COEFFICIENT = 0.008; //How fast power scales down after passing ERROR_THRESHOLD, higher numbers = more power
    //End Tuning Variables

    int rollover = 0; //Holds the number of times the servo has crossed the rollover threshold
    double servoAngle = 0; //Holds the angle of the servo in degrees

    //Priors
    double lastVoltage = 0; //Holds the voltage from the last loop
    double lastDelta = 0; //Holds the change in voltage from the last loop
    double lastTime = 0; //Holds the time from the last loop
    //End Priors

    double RPM = 0; //Holds the RPM of the servo, used for setting RPM
    double offset = 0; //Added to power in setRPM to bring the RPM closer to the target
    double savedRollover = 0; //Holds the rollover currently saved to disk
    boolean saveRollover = true;
    boolean isBusy = false;

    //Initializer methods
    //These handle reading off of disk and allow the setting of attributes
    public void initializeServo(boolean SaveRollover, double Threshold, double Correction, int Direction, boolean PowerModule) { //It is recommended that if you intend to operate the servo beyond one rotation and keeping track of it matters that you enable saving to the disk
        if (SaveRollover) {
            readLogFile();
            saveRollover = true;
        } else {
            saveRollover = false;
        }
        if (PowerModule) { SERVO_RPM = 86.96; }
        else { SERVO_RPM = 71.43; }

        ERROR_THRESHOLD = Threshold;
        CORRECTION_COEFFICIENT = Correction;
        lastVoltage = analog.getVoltage();
        DIRECTION = Direction;
    }
    public void initializeServo(boolean SaveRollover) { //Running this method will keep the default parameters
        if (SaveRollover) {
            readLogFile();
            saveRollover = true;
        } else {
            saveRollover = false;
        }
        lastVoltage = analog.getVoltage();
    }

    //Getters & metering methods
    public double findAngle() { //Run once per loop
        servoAngle = (getContinuousVoltage() / MAX_VOLTAGE) * 360.0;
        return servoAngle;
    }
    public double getAngle() { return servoAngle; }
    public double getVoltage() { return analog.getVoltage(); }
    public boolean isBusy() { return isBusy; }
    //End Getters & metering methods

    //File Saving/Access Methods
    //These methods handle the saving of the rollover value to disk, so that in the event of a power failure or the robot being turned off, the robot will remember where the servos are to zero them
    private File getFile() {
        return AppUtil.getInstance().getSettingsFile(name + "_rollover.txt");
    }
    public int loadRollover() {
        try {
            Scanner scanner = new Scanner(getFile());
            int value = scanner.nextInt();
            scanner.close();
            double voltage = getVoltage();

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
            FileWriter writer = new FileWriter(getFile()); // overwrites
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
    //End File Saving Code

    //This method handles the voltage wraparound, which keeps track of the number of times the servo's analog output has 'rolled over' from 0V-3.3V or 3.3V-0V
    public double getContinuousVoltage() {
        double current = getVoltage();
        double delta = current - lastVoltage;
        double time = runtime.seconds();
        double deltaTime = time - lastTime;

        RPM = (delta / MAX_VOLTAGE) / (deltaTime / 60);
        if (Math.abs(lastDelta) > MAX_VOLTAGE * 0.2 && Math.abs(delta) > MAX_VOLTAGE * 0.2 && Math.signum(lastDelta) == Math.signum(delta)) { delta = delta + lastDelta; }

        //Handle rollover
        if (delta > MAX_VOLTAGE / 2) {
            if (lastDelta <= MAX_VOLTAGE / 2) {
                rollover -= 1;
            }
        } else if (delta < -MAX_VOLTAGE / 2) {
            if (lastDelta >= -MAX_VOLTAGE / 2) {
                rollover += 1;
            }
        }

        double continuousVoltage = current + (MAX_VOLTAGE * rollover);

        if (saveRollover && rollover != savedRollover) { saveRollover(rollover); }  //This statement determines if the current rollover should be saved

        lastTime = time;
        lastDelta = delta;
        lastVoltage = current;
        return continuousVoltage;
    }

    //Servo control methods
    public void runToPosition(double angle) { //Accepts inputs of any angle, and the servo will move to that position at max speed
        double error = angle - findAngle();

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            isBusy = true;
            power = Math.signum(error); //When power is positive axon servo rotates counterclockwise
        } else if (Math.abs(error) > 2){
            isBusy = true;
            power = CORRECTION_COEFFICIENT * error;
        } else {
            isBusy = false;
            power = 0.0;
        }
        CRservo.setPower(power * DIRECTION);
    }
    public void runToPosition(double angle, double speed) { //Accepts inputs of any angle, and the servo will move to that position at set speed
        double error = angle - findAngle();

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            isBusy = true;
            power = Math.signum(error);
        } else if (Math.abs(error) > 2){
            isBusy = true;
            power = CORRECTION_COEFFICIENT * error;
        } else {
            isBusy = false;
            power = 0.0;
        }
        power = Math.min(power, speed);
        power = Math.max(power, -speed);

        CRservo.setPower(power * DIRECTION);
    }
    public void setPower(double power) {
        CRservo.setPower(power);
    }
    public void setRPM(double targetRPM) {
        double power = targetRPM / SERVO_RPM + offset * Math.signum(targetRPM);
        double error = targetRPM - RPM;
        if (error > 5) { offset += 0.05; }
        if (error < -5) { offset -= 0.05; }
        power = Math.min(power, 1);
        power = Math.max(power, -1);
        CRservo.setPower(power * DIRECTION);
    }
    //End servo control methods
}