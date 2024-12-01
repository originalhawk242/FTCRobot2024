package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.vision.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.SamplePipeline;


@Autonomous
public class VisionAuto extends LinearOpMode {
    Camera camera = new Camera(this);


    @Override
    public void runOpMode() throws InterruptedException {
        camera.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");
        waitForStart();


        SamplePipeline.getLocation();
        while (opModeIsActive()) {
        }
    }
}

