package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.vision.Camera;

@Autonomous
public class VisionTest extends LinearOpMode {
    Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException {
        camera.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine(camera.getPipeline1Output());
            telemetry.update();
        }
    }
}
