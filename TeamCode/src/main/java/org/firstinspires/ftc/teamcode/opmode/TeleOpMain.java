package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.*;

@TeleOp
public class TeleOpMain extends OpMode {

    private DriveTrain driveTrain;

    private ConditionalHardwareDevice<DcMotor> slideMotor;
    private PIDFController slidePID;

    private ConditionalHardwareDevice<Servo> slideServo;

    @Override
    public void init() {
        driveTrain = new DriveTrain(this);

        slideMotor = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, DcMotor.class, "Slide Motor");
        slideMotor.runIfAvailable(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
        slidePID = new PIDFController(0,0,0,0);

        slideServo = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, Servo.class, "Slide Servo");
    }

    @Override
    public void loop() {
        driveTrain.setVelocity(-gamepad1.left_stick_x * 0.5, gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x * 0.5);

        slideMotor.runIfAvailable(motor -> {
            slidePID.setSetPoint(gamepad1.left_trigger);
            motor.setPower(slidePID.calculate(motor.getCurrentPosition()));
        });

        slideServo.runIfAvailable(servo -> {
            servo.setPosition(gamepad2.right_trigger);
        });
    }

}
