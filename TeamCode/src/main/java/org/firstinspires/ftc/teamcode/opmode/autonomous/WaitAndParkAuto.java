package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class WaitAndParkAuto extends AutonomousBase {

    public static double WAIT_BEFORE_PARK_SECONDS = 2;
    public static double MOVE_TO_PARK_DURATION_SECONDS = 4;

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
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        TeleOpMain.resetSlidePosition = false;

        waitForStart();

        resetArmPosition();

        final ElapsedTime timer = new ElapsedTime();


        timer.reset();
        while (timer.time(TimeUnit.SECONDS) < WAIT_BEFORE_PARK_SECONDS) {
            if (isStopRequested()) {
                return;
            }
            slide.updateMotorPowers();
        }

        driveTrain.setVelocity(0.5, 0, 0);
        timer.reset();
        while (timer.time(TimeUnit.SECONDS) < MOVE_TO_PARK_DURATION_SECONDS) {
            if (isStopRequested()) {
                return;
            }
            slide.updateMotorPowers();
        }

        driveTrain.setVelocity(0, 0, 0);

        arm.activate();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPowers();
        intake.moveWristTo(Intake.WRIST_POSITION_START);
        Thread.sleep(3L * TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        waitForEnd();
    }
}
