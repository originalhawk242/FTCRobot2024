package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class LinearSlide extends Module {
    private final ConditionalHardwareDevice<PIDFDcMotor> motor;
    public static final String SLIDE_MOTOR_NAME = "Slide Motor";
    /**
     * Encoder resolution for the 5203 312 RPM DC Motors used by the arm
     */
    private static final double SLIDE_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    @Config
    public static class SlideConfig {
        public static double P_COEF = 0.01;
        public static double I_COEF = 0;
        public static double D_COEF = 0;
        public static double F_COEF = 0;
        public static double TOLERANCE = 2;
    }

    public LinearSlide(OpMode registrar) {
        super(registrar);
        motor = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, PIDFDcMotor.class, SLIDE_MOTOR_NAME);

        motor.runIfAvailable(m -> {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            m.setTolerance(SlideConfig.TOLERANCE);
        });
    }

    private void setTargetPosition(int targetPosition) {
        motor.runIfAvailable(m -> { m.setSetPoint(targetPosition); });
    }

    /**
     * Sets the target height of the robot
     * @param height The desired height; values range from 0 for fully retracted to 1 for fully extended
     */
    public void setTargetHeight(double height) {
        setTargetPosition((int)(height * SLIDE_ENCODER_RESOLUTION));
    }

    /**
     * Updates the motor power using the provided PIDF coefficients
     */
    public void updateMotorPower() {
        motor.runIfAvailable(slide -> {
            slide.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            slide.setTolerance(SlideConfig.TOLERANCE);

            slide.setPower(slide.calculate());
        });
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!motor.isAvailable()) { return; }
        PIDFDcMotor slide = motor.requireDevice();

        telemetry.addData("Current slide position", slide.getCurrentPosition());
        telemetry.addData("Target slide position", slide.getSetPoint());
    }
}
