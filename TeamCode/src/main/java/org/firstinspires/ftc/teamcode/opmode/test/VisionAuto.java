package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.vision.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.SamplePipeline;
import org.firstinspires.ftc.teamcode.modules.vision.YellowSample;


@Autonomous
public class VisionAuto extends LinearOpMode {
    Camera camera = new Camera(this);


    @Override
    public void runOpMode() throws InterruptedException {
        camera.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");
        waitForStart();

        YellowSample.processFrame();

        String sampleLocation = YellowSample.getLocation();
        if (sampleLocation.equals("1")) {
            telemetry.addLine("sample: left");
        } else if (sampleLocation.equals("2")) {
            telemetry.addLine("sample: right");
        } else {
            telemetry.addLine("no sample detected")
        }
        while (opModeIsActive()) {
        }
    }
}

