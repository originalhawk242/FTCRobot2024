package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(this);
        slide = new LinearSlide(this);
    }

    @Override
    public void init_loop() {
        arm.log();
    }

    @Override
    public void start() {
        slide.setTargetHeight(0);
    }

    @Override
    public void loop() {
        arm.setTargetRotation(TARGET_ROTATION);
        arm.updateMotorPowers();
        slide.updateMotorPower();
        arm.log();
    }
}
