package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.*;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends OpMode {

    private DriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain = new DriveTrain(this);
    }

    @Override
    public void loop() {
        driveTrain.setVelocity(-gamepad1.left_stick_x * 0.5, gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x * 0.5);
    }

}
