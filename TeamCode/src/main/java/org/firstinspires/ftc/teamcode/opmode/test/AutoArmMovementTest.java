package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

@Autonomous
public class AutoArmMovementTest extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        final ModuleManager moduleManager = new ModuleManager(this);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);

        waitForStart();

        arm.setTargetRotation(45);
        slide.setTargetHeight(0.25);

        do {
            if (isStopRequested()) {
                return;
            }
            arm.updateMotorPower();
            slide.updateMotorPower();
            telemetry.update();
        } while (arm.isMoving() && slide.isMoving());

        telemetry.addLine("Movement complete");
        telemetry.update();

        arm.deactivate();

        while (opModeIsActive()) {
            Thread.yield();
        }
    }
}
