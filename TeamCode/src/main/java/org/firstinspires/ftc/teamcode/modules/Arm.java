package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class Arm extends Module {
    private final ConditionalHardwareDeviceGroup motors;
    public static final String LEFT_ARM_MOTOR_NAME = "Left Arm Motor";
    public static final String RIGHT_ARM_MOTOR_NAME = "Right Arm Motor";
    /**
     * Encoder resolution for the 5203 117 RPM DC Motors used by the arm
     */
    private static final double ARM_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    @Config
    public static class ArmConfig {
        public static double P_COEF = 0.01;
        public static double I_COEF = 0;
        public static double D_COEF = 0;
        public static double F_COEF = 0;
        public static double TOLERANCE = 2;
    }

    /**
     * Configures a motor to be used to control the arm
     * @param m either the left or right arm motor
     */
    private static void configureMotorPID(PIDFDcMotor m) {
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m.setPIDF(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF, ArmConfig.F_COEF);
        m.setTolerance(ArmConfig.TOLERANCE);
    }
    public Arm(OpMode registrar) {
        super(registrar);
        motors = new ConditionalHardwareDeviceGroup(registrar.hardwareMap);

        motors.tryLoadDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
        motors.tryLoadDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);

        motors.executeIfAllAreAvailable(() -> {
            final PIDFDcMotor leftMotor = motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            configureMotorPID(leftMotor);
            final PIDFDcMotor rightMotor = motors.requireLoadedDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            configureMotorPID(rightMotor);
        }, () -> getTelemetry().addLine("Failed to load arm motors!"));
    }

    private void setTargetPosition(int targetPosition) {
        motors.executeIfAllAreAvailable(() -> {
            final PIDFDcMotor leftMotor = motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            final PIDFDcMotor rightMotor = motors.requireLoadedDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            leftMotor.setSetPoint(targetPosition);
            rightMotor.setSetPoint(targetPosition);
        });
    }

    /**
     * Sets the target rotation for the arm
     * @param rotation The desired rotation in degrees
     */
    public void setTargetRotation(double rotation) {
        setTargetPosition((int)(rotation * ARM_ENCODER_RESOLUTION / 360));
    }
    /**
     * Updates the motor powers using the provided PIDF coefficients
     */
    private static void updateMotorPower(PIDFDcMotor motor) {
        motor.setPIDF(ArmConfig.P_COEF, ArmConfig.I_COEF, ArmConfig.D_COEF, ArmConfig.F_COEF);
        motor.setTolerance(ArmConfig.TOLERANCE);

        motor.setPower(motor.calculate());
    }
    public void updateMotorPowers() {
        motors.executeIfAllAreAvailable(() -> {
            updateMotorPower(motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME));
        });
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!motors.areAllDevicesAvailable()) { return; }
        final PIDFDcMotor leftMotor = motors.requireLoadedDevice(PIDFDcMotor.class, LEFT_ARM_MOTOR_NAME);
        final PIDFDcMotor rightMotor = motors.requireLoadedDevice(PIDFDcMotor.class, RIGHT_ARM_MOTOR_NAME);

        telemetry.addData("Current left motor position", leftMotor.getCurrentPosition());
        telemetry.addData("Target left motor position", leftMotor.getSetPoint());
        telemetry.addData("Current right motor position", rightMotor.getCurrentPosition());
        telemetry.addData("Target right motor position", rightMotor.getSetPoint());
    }
}
