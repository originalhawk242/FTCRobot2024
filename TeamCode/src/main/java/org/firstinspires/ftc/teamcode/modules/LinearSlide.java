package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.PIDFDcMotor;
import org.firstinspires.ftc.teamcode.modules.core.Module;

/**
 * This module controls the linear slide, which moves the intake closer and farther
 * away from the robot.
 */

@Config
public class LinearSlide extends Module {
    private final ConditionalHardwareDevice<PIDFDcMotor> motor;
    public static final String SLIDE_MOTOR_NAME = "Slide Motor";
    /**
     * Encoder resolution for the 5203 312 RPM DC Motors used by the arm
     */
    private static final double SLIDE_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);
    private static final int SLIDE_MAX_EXTENSION_TICKS = 2500;

    @Config
    public static class SlideConfig {
        public static double P_COEF = 0.0033;
        public static double I_COEF = 0;
        public static double D_COEF = 0.0001;
        public static double F_COEF = 0;
        public static double TOLERANCE = 2;
    }

    public static double SLIDE_HEIGHT_INTAKE = 0.9163;
    public static double SLIDE_HEIGHT_MOVING = 0.0120;
    public static double SLIDE_HEIGHT_SCORING = 1.0000;
    public static double SLIDE_HEIGHT_HANG_LVL1 = 0.5000;
    public static double SLIDE_HEIGHT_HANG_LVL2 = 0.0100;

    public LinearSlide(OpMode registrar) {
        this(registrar, true);
    }
    public LinearSlide(OpMode registrar, boolean resetPosition) {
        super(registrar);
        motor = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, PIDFDcMotor.class, SLIDE_MOTOR_NAME);

        motor.runIfAvailable(m -> {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            if (resetPosition) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            m.setTolerance(SlideConfig.TOLERANCE);
        });
    }

    /**
     * Checks if this module is connected to the hardware it requires
     *
     * @return false if the module cannot change the state of the hardware, true otherwise
     */
    @Override
    public boolean isConnected() {
        return motor.isAvailable();
    }

    /**
     * Ensures that the module is in a safe state for other modules to operate.
     * Between calling this method and calling any other method on this module that modifies
     * hardware devices, the module is guaranteed to not damage itself or anything else when
     * other modules modify hardware state
     */
    @Override
    public void ensureSafety() {
        moveSlideTo(SLIDE_HEIGHT_MOVING);
    }

    private void setTargetPosition(int targetPosition) {
        motor.runIfAvailable(m -> { m.setSetPoint(targetPosition); });
    }

    /**
     * Sets the target height of the robot
     * @param height The desired height; values range from 0 for fully retracted to 1 for fully extended
     */
    public void setTargetHeight(double height) {
        if (height < 0 || height > 1) {
            return;
        }
        setTargetPosition((int)(height * SLIDE_MAX_EXTENSION_TICKS));
    }

    public void moveSlideTo(double height) {
        setTargetHeight(height);
        motor.runIfAvailable(PIDFDcMotor::waitUntilPointReached);
    }

    /**
     * Updates the motor power using the provided PIDF coefficients
     */
    public void updateMotorPower() {
        motor.runIfAvailable(slide -> {
            slide.setPIDF(SlideConfig.P_COEF, SlideConfig.I_COEF, SlideConfig.D_COEF, SlideConfig.F_COEF);
            slide.setTolerance(SlideConfig.TOLERANCE);

            slide.applyMotorPIDF();
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
