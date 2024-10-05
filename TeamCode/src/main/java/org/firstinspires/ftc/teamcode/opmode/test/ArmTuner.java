package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;

@Config
@Autonomous
public class ArmTuner extends OpMode {
    private Arm arm;
    private LinearSlide slide;
    public static double TARGET_ROTATION = 0;


    @Override
    public void init() {
        arm = new Arm(this);
        slide = new LinearSlide(this);
    }

    @Override
    public void init_loop() {
        arm.log();
        slide.log();
    }

    @Override
    public void start() {
        slide.setTargetHeight(0);
    }

    @Override
    public void loop() {
        arm.setTargetRotation(TARGET_ROTATION);
    }
}
