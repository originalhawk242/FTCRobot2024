package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;

@TeleOp
public class TeleOpMain extends OpMode {

    private FieldCentricDriveTrain driveTrain;

    private LinearSlide slide;

    private Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new FieldCentricDriveTrain(this);

        slide = new LinearSlide(this);

        intake = new Intake(this);
    }

    @Override
    public void init_loop() {
        driveTrain.log();
        slide.log();
        intake.log();
    }

    @Override
    public void loop() {

        if (gamepad1.back) {
            driveTrain.resetRotation();
        }

        driveTrain.setVelocity(gamepad1.left_stick_x * 0.5, -gamepad1.left_stick_y * 0.5, -gamepad1.right_stick_x * 0.5);

        slide.setTargetPosition((int)(gamepad1.left_trigger * 1800));
        slide.updateMotorPower();

        if (gamepad2.a) {
            intake.closeIntake();
        } else if (gamepad2.b) {
            intake.openIntake();
        }

        driveTrain.log();
        slide.log();
        intake.log();
        telemetry.addData("Gamepad1 Right Trigger: ", gamepad1.right_trigger);
    }

}
