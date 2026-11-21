package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SwerveDriveTest {
    @Rule public TemporaryFolder settings = new TemporaryFolder();
    private static final double EPSILON = 1e-8;
    private final boolean previousOdometry = TeleOP.updateOdometry;
    private final boolean previousIdle = TeleOP.isNoInput;
    private final boolean previousSave = SwervePod.saveRollover;

    @Before public void enableDriving() {
        TeleOP.updateOdometry = true;
        TeleOP.isNoInput = false;
        SwervePod.saveRollover = false;
    }

    @After public void restoreSettings() {
        TeleOP.updateOdometry = previousOdometry;
        TeleOP.isNoInput = previousIdle;
        SwervePod.saveRollover = previousSave;
    }

    private static <T> T device(Class<T> type, InvocationHandler handler) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, handler));
    }

    private class Wheel {
        double power;
        double voltage;
        int ticks;
        final SwervePod pod;

        Wheel() {
            DcMotorEx motor = device(DcMotorEx.class, (proxy, method, args) -> {
                switch (method.getName()) {
                    case "getCurrentPosition": return ticks;
                    case "getDirection": return DcMotor.Direction.FORWARD;
                    case "setPower": power = (double) args[0]; return null;
                    default: throw new AssertionError(method);
                }
            });
            CRServo servo = device(CRServo.class, (proxy, method, args) -> {
                if (method.getName().equals("setPower")) return null;
                throw new AssertionError(method);
            });
            AnalogInputController sensor = device(AnalogInputController.class, (proxy, method, args) -> {
                if (method.getName().equals("getAnalogInputVoltage")) return voltage;
                throw new AssertionError(method);
            });
            pod = new SwervePod("wheel", servo, motor, new AnalogInput(sensor, 0)) {
                @Override File getFile() { return new File(settings.getRoot(), "rollover.txt"); }
            };
        }
    }

    @Test public void startupRestoresUsingLiveVoltageAndCountsNextWrapOnce() throws IOException {
        Wheel wheel = new Wheel();
        try (FileWriter saved = new FileWriter(wheel.pod.getFile())) {
            saved.write("-1");
        }
        wheel.voltage = 3.1;
        wheel.pod.readLogFile();
        wheel.pod.update();
        assertEquals(-1, wheel.pod.getRollover());
        wheel.voltage = 0.1;
        wheel.pod.update();
        assertEquals(0, wheel.pod.getRollover());
    }

    @Test public void accelerationUsesElapsedSecondsAndStopIsImmediate() {
        Wheel wheel = new Wheel();
        wheel.pod.setDrivePower(1);
        wheel.pod.updateOutputs(0.02);
        assertEquals(0.02, wheel.power, EPSILON);
        wheel.pod.updateOutputs(0.04);
        assertEquals(0.06, wheel.power, EPSILON);
        wheel.pod.setTargetAngle(180);
        wheel.pod.updateOutputs(0.02);
        assertEquals(-0.02, wheel.power, EPSILON);
        wheel.pod.setDrivePower(0);
        wheel.pod.updateOutputs(0.02);
        assertEquals(0, wheel.power, EPSILON);
    }

    @Test public void reversedWheelGetsCorrectPowerOnFirstCommand() {
        TeleOP.updateOdometry = false;
        Wheel[] wheels = {new Wheel(), new Wheel(), new Wheel(), new Wheel()};
        SwerveDrive drive = new SwerveDrive(wheels[0].pod, wheels[1].pod, wheels[2].pod, wheels[3].pod);
        drive.drive(180, 0.75, 0);
        drive.update();
        for (Wheel wheel : wheels) assertEquals(-0.75, wheel.power, EPSILON);
        drive.drive(0, 0, 0);
        drive.update();
        for (Wheel wheel : wheels) assertEquals(0, wheel.power, EPSILON);
    }

    @Test public void odometrySeparatesFieldTranslationFromClockwiseRotation() {
        Wheel fl = new Wheel();
        Wheel fr = new Wheel();
        Wheel rl = new Wheel();
        Wheel rr = new Wheel();
        Odometry pose = new Odometry(fl.pod, fr.pod, rl.pod, rr.pod, null, null);
        pose.headingMethod = 0;
        pose.setStartingPosition(0, 0, 90);
        // One foot forward plus 0.2 radians of rotation about the center.
        fl.pod.deltaX = TeleOP.LENGTH / 24.0 * 0.2;
        fl.pod.deltaY = 1 + TeleOP.WIDTH / 24.0 * 0.2;
        rr.pod.deltaX = -fl.pod.deltaX;
        rr.pod.deltaY = 1 - TeleOP.WIDTH / 24.0 * 0.2;
        fr.pod.deltaX = 10; // This encoder is disabled.
        pose.update();
        pose.update(); // Movement must only be consumed once.
        assertEquals(1, pose.getSDX(), EPSILON);
        assertEquals(0, pose.getSDY(), EPSILON);
        assertEquals(90 + Math.toDegrees(0.2), pose.getSDHeading(), EPSILON);
    }

    @Test public void encoderMotionWhileSteeringDoesNotLeakIntoNextMeasurement() {
        Wheel wheel = new Wheel();
        wheel.pod.setTargetAngle(90);
        wheel.pod.updateOutputs(0.02);
        assertTrue(wheel.pod.isBusy);
        wheel.ticks = 100;
        wheel.pod.update();
        wheel.pod.setTargetAngle(0);
        wheel.pod.updateOutputs(0.02);
        wheel.ticks = 103;
        wheel.pod.update();
        assertEquals(wheel.pod.ticksToDistance(3), wheel.pod.getDeltaY(), EPSILON);
    }
}
