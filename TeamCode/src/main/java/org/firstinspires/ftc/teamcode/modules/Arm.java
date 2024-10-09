package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.Module;

/**
 * This module controls the arm, which rotates the intake mechanism around the robot
 */
@Config
public class Arm extends Module {
    private final ConditionalHardwareDeviceGroup motors;
    public static final String LEFT_ARM_MOTOR_NAME = "Left Arm Motor";
    public static final String RIGHT_ARM_MOTOR_NAME = "Right Arm Motor";
    /**
     * Encoder resolution for the 5203 117 RPM DC Motors used by the arm
     */
    private static final double ARM_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);
    private final PIDFController controller;

    /**
     * The coefficients for the arm's PIDF controller
     */
    @Config
    public static class ArmConfig {
        public static double P_COEF = 0.003;
        public static double I_COEF = 0;
        public static double D_COEF = 0;
        public static double F_COEF = 0;
        public static double TOLERANCE = 2;
    }

    /*
     * Preset arm rotations for certain events during play
     */
    public static double ARM_ROTATION_INTAKE = -25.5;
    public static double ARM_ROTATION_MOVING = -35;
    public static double ARM_ROTATION_SCORING = -105;

    /**
     * Configures a motor to be used to control the arm
     * @param m either the left or right arm motor
     */
    private static void configureMotor(DcMotor m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initializes the arm with the current rotation as '0 degrees'
     * @param registrar The op mode creating this arm
     */
    public Arm(OpMode registrar) {
        super(registrar);
        motors = new ConditionalHardwareDeviceGroup(registrar.hardwareMap);

        motors.tryLoadDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
        motors.tryLoadDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);

        motors.executeIfAllAreAvailable(() -> {
            final PIDFDcMotor leftMotor = motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            configureMotor(leftMotor);
            final PIDFDcMotor rightMotor = motors.requireLoadedDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            configureMotor(rightMotor);
        }, () -> getTelemetry().addLine("Failed to load arm motors!"));

        controller = new PIDFController(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF, ArmConfig.F_COEF);

        controller.setPIDF(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF, ArmConfig.F_COEF);
        controller.setTolerance(ArmConfig.TOLERANCE);
    }

    /**
     * Sets the target position for the PIDF controller
     * @param targetPosition the target position, in ticks
     */
    private void setTargetPosition(int targetPosition) {
        controller.setSetPoint(targetPosition);
    }

    /**
     * Sets the target rotation for the arm
     * @param rotation The desired rotation in degrees
     */
    public void setTargetRotation(double rotation) {
        // 360 degrees maps to 1 rotation, with a 5:1 gear ratio
        setTargetPosition((int)(rotation * ARM_ENCODER_RESOLUTION * 5 / 360));
    }
    public void updateMotorPowers() {
        motors.executeIfAllAreAvailable(() -> {
            controller.setPIDF(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF, ArmConfig.F_COEF);
            controller.setTolerance(ArmConfig.TOLERANCE);
            DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
            DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
            double power = controller.calculate(leftMotor.getCurrentPosition()); // use one encoder for safety and apply the same power to both motors
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        });
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!motors.areAllDevicesAvailable()) { return; }
        final DcMotor leftMotor = motors.requireLoadedDevice(DcMotor.class, LEFT_ARM_MOTOR_NAME);
        final DcMotor rightMotor = motors.requireLoadedDevice(DcMotor.class, RIGHT_ARM_MOTOR_NAME);

        telemetry.addData("Current left motor position", leftMotor.getCurrentPosition());
        telemetry.addData("Current right motor position", rightMotor.getCurrentPosition());
        telemetry.addData("Target motor position", controller.getSetPoint());
    }
}
