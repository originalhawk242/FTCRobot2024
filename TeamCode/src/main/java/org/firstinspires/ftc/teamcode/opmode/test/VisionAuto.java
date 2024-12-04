package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.vision.Camera;
import org.firstinspires.ftc.teamcode.modules.vision.YellowSample;


@Autonomous
public class VisionAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final Camera camera = new Camera(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Distance to sample: " + YellowSample.getDistance());
            telemetry.addLine(camera.getYellowStatus());
            telemetry.update();
        }
    }
}

