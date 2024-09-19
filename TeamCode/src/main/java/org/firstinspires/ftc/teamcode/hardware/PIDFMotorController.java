package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class PIDFMotorController extends PIDFController {
    private final ConditionalHardwareDevice<DcMotor> motor;

    public PIDFMotorController(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction motorDirection, double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf, 0, 0);
        this.motor = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, DcMotor.class, motorName);

        if (!motor.isAvailable()) { return; }
        DcMotor m = motor.requireDevice();

        m.setDirection(motorDirection);

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public PIDFMotorController(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction motorDirection) {
        this(hardwareMap, motorName, motorDirection, 0, 0, 0, 0);
    }

    public void updateMotorPower() {
        motor.runIfAvailable(m -> {
            m.setPower(calculate(m.getCurrentPosition()));
        });
    }
}
