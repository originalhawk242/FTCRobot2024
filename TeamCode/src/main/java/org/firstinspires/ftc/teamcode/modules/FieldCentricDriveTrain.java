package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;

/**
 * This module is for the field-centric drive train, which moves the robot.
 * Note: As this drive train module is field-centric, the robot decides what
 *      forward is based on a preset forward (commonly the direction the drivers
 *      are facing).
 *      For a robot-centric drive train, use the DriveTrain module.
 */

public class FieldCentricDriveTrain extends DriveTrain {

    public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    private double headingOffset = 0;

    /**
     * The IMU
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final IMU getIMU() {
        return hardwareDevices.requireLoadedDevice(IMU.class, IMU_NAME);
    }

    private static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    public static IMU.Parameters getImuParameters() {
        return IMU_PARAMETERS;
    }

    public static final String IMU_NAME = "imu";

    public FieldCentricDriveTrain(OpMode registrar) {
        super(registrar);
        hardwareDevices.tryLoadDevice(IMU.class, IMU_NAME);

        hardwareDevices.executeIfAllAreAvailable(() -> {
            getIMU().initialize(getImuParameters());
            resetRotation();
            getTelemetry().addLine("[Field Centric Drive Train] Found IMU");
        }, () -> getTelemetry().addLine("[Field Centric Drive Train] Couldn't find IMU!"));
    }

    public void resetRotation() {
        headingOffset = getRobotPose().getHeading(ANGLE_UNIT);
    }

    @Override
    public void setVelocity(double strafe, double forward, double rotation) {
        if (!odometry.isConnected()) {
            getTelemetry().addLine("! OTOS DISCONNECTED !");
            super.setVelocity(strafe, forward, rotation);
            return;
        }

        hardwareDevices.executeIfAllAreAvailable(() -> {
            double botHeading = getRobotPose().getHeading(ANGLE_UNIT) - headingOffset;

            // Rotate the movement direction counter to the robot's rotation
            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
            getTelemetry().addData("[Field Centric Drive Train] current x velocity", rotX);
            getTelemetry().addData("[Field Centric Drive Train] current y velocity", rotY);
            getTelemetry().addData("[Field Centric Drive Train] bot heading value", botHeading);
            rotX = rotX;  // Counteract imperfect strafing

            super.setVelocity(rotX, rotY, rotation);
        });
    }
}
