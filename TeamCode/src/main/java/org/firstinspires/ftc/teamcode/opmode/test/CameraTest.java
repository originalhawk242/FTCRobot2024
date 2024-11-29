package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.vision.Camera;

public class CameraTest {
    @TeleOp
    public class VisionTeleop extends LinearOpMode {
        Camera camera = new Camera(this);

        @Override
        public void runOpMode() throws InterruptedException {
            camera.switchToFirstPipeline();
            telemetry.addLine("Status: Initialized");
            waitForStart();

            while (opModeIsActive()) {
            }
        }
    }
}
