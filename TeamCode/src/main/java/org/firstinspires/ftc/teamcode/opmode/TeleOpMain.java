package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.*;

@TeleOp
public class TeleOpMain extends OpMode {

    private FieldCentricDriveTrain driveTrain;

    private LinearSlide slide;

    private ConditionalHardwareDevice<Servo> slideServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new FieldCentricDriveTrain(this);

        slide = new LinearSlide(this);

        slideServo = ConditionalHardwareDevice.tryGetHardwareDevice(hardwareMap, Servo.class, "Slide Servo");
    }

    @Override
    public void init_loop() {
        driveTrain.log();
        slide.log();
    }

    @Override
    public void loop() {

        if (gamepad1.back) {
            driveTrain.resetRotation();
        }

        driveTrain.setVelocity(gamepad1.left_stick_x * 0.5, -gamepad1.left_stick_y * 0.5, -gamepad1.right_stick_x * 0.5);

        slide.setTargetPosition((int)(gamepad1.left_trigger * 1800));
        slide.updateMotorPower();

        //0.35 seems to be closed claw position
        slideServo.runIfAvailable(servo -> {
            servo.setPosition(0.35 - (gamepad1.right_trigger / 2.857));
        });

        driveTrain.log();
        slide.log();
        telemetry.addData("Gamepad1 Right Trigger: ", gamepad1.right_trigger);
    }

}
