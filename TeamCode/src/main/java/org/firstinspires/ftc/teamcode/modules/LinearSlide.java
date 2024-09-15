package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;
import org.firstinspires.ftc.teamcode.opmode.TeleOpMain;

public class LinearSlide extends Module {
    private final ConditionalHardwareDevice<DcMotor> slideMotor;
    public static final String SLIDE_MOTOR_NAME = "Slide Motor";

    private final PIDFController slidePID;

    public LinearSlide(OpMode registrar) {
        super(registrar);
        slideMotor = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, DcMotor.class, SLIDE_MOTOR_NAME);
        slideMotor.runIfAvailable(motor -> {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        });
        slidePID = new PIDFController(0, 0, 0, 0);
    }

    public void setPIDCoefficients(double kP, double kI, double kD, double kF) {
        slidePID.setPIDF(kP, kI, kD, kF);
    }
    public void setPIDTolerance(double tolerance) {
        slidePID.setTolerance(tolerance);
    }

    public void setTargetPosition(int targetPosition) {
        slidePID.setSetPoint(targetPosition);
    }
    public void updateMotorPower() {
        slideMotor.runIfAvailable(slide -> slide.setPower(slidePID.calculate()));
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if (!slideMotor.isAvailable()) { return; }
        DcMotor slide = slideMotor.requireDevice();

        telemetry.addData("Current slide position", slide.getCurrentPosition());
        telemetry.addData("Target slide position", slidePID.getSetPoint());
    }
}
