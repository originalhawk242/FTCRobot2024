package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");
        waitForStart();

        while (opModeIsActive()) {
            int sampleLocation = YellowSample.getLocation();

            if (sampleLocation == 1) {
                telemetry.addLine("sample: left");
            } else if (sampleLocation == 2) {
                telemetry.addLine("sample: right");
            } else {
                telemetry.addLine("no sample detected");
            }

            telemetry.addLine(camera.getYellowStatus());
            telemetry.update();
        }
    }
}

