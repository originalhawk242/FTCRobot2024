package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class ImmediateParkAuto extends LinearOpMode {
    private static final double DRIVE_ENCODER_RESOLUTION = ((((1+(46/17))) * (1+(46/11))) * 28);

    public static double MOVE_TO_PARK_DURATION_SECONDS = 2;

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
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        TeleOpMain.resetSlidePosition = false;

        waitForStart();
        // get arm out of way
        slide.setTargetHeight(0);
        slide.updateMotorPower();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPowers();
        Thread.sleep(TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);

        driveTrain.setVelocity(0.5, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time(TimeUnit.SECONDS) < MOVE_TO_PARK_DURATION_SECONDS) {
            if (isStopRequested()) {
                return;
            }
            slide.updateMotorPower();
        }

        driveTrain.setVelocity(0, 0, 0);

        arm.activate();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPowers();
        Thread.sleep(3L * TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        intake.moveWristTo(Intake.WRIST_POSITION_START);
        while (opModeIsActive()) {
            slide.updateMotorPower();
        }
    }
}
