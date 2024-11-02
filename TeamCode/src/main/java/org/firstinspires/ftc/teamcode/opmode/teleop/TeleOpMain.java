package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.*;

@Config
@TeleOp
public class TeleOpMain extends OpMode {

    public static boolean resetSlidePosition = true;

    /**
     * The time at start where the arm is moving upward at constant speed.
     * Needs to be nonzero to ensure that the wrist isn't pushing against the ground
     * when it rotates.
     */
    public static int INITIAL_JUMP_TIME_MILLIS = 40;
    public static double SLOWER_TURN_SPEED_MULTIPLIER = 0.25;

    private FieldCentricDriveTrain driveTrain;

    private LinearSlide slide;

    private Arm arm;

    private Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new FieldCentricDriveTrain(this);

        slide = new LinearSlide(this, resetSlidePosition);

        arm = new Arm(this);

        intake = new Intake(this);
    }

    @Override
    public void init_loop() {
        driveTrain.log();
        slide.log();
        arm.log();
        intake.log();
    }

    @Override
    public void start() {
//        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
//        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
//        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPowers();
        try {
            Thread.sleep(INITIAL_JUMP_TIME_MILLIS);
        } catch (InterruptedException ignored) {}
        deactivateArm();

        driveTrain.resetRotation();
    }

    @Override
    public void loop() {
        if (gamepad1.guide || gamepad2.guide || gamepad1.ps || gamepad2.ps) {
            terminateOpModeNow();
        }

        if (gamepad1.back && gamepad1.start) {
            driveTrain.resetRotation();
        }

        final double strafe = gamepad1.left_stick_x;
        final double forward = -gamepad1.left_stick_y;
        final double rotate = -gamepad1.right_stick_x;
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            driveTrain.setVelocity(strafe, forward, rotate * SLOWER_TURN_SPEED_MULTIPLIER);
        }
        else {
            driveTrain.setVelocity(strafe, forward, rotate);
        }

//        if (gamepad2.y) {
//            intake.turn();
//        }
        intake.holdWristRotation();
        if (gamepad2.left_bumper) {
            intake.grab();
        } else if (gamepad2.right_bumper) {
            intake.eject();
        } else {
            intake.settle();
        }

        boolean activateArm = true;
        if (gamepad2.a) {
            slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
            arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
            intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        }
        else if (gamepad2.x) {
            slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_SCORING);
            arm.setTargetRotation(Arm.ARM_ROTATION_SCORING);
            intake.moveWristTo(Intake.WRIST_POSITION_SCORING);
        }
        else if (gamepad2.b) {
            slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_INTAKE);
            arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
            intake.moveWristTo(Intake.WRIST_POSITION_INTAKE);
        }
        else if (gamepad2.dpad_up) {
            slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_HANG_LVL1);
            arm.setTargetRotation(Arm.ARM_ROTATION_HANG_LVL1_SETUP);
            intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);
        }
        else {
            activateArm = false;
        }

        slide.updateMotorPower();
        arm.updateMotorPowers();
        if (gamepad2.y) {
            deactivateArm();
        }
        else if (gamepad2.dpad_down) {
            dropArmUnsafe();
        }
        else if (activateArm || arm.monitorPositionSwitch()) {
            activateArm();
        }

        driveTrain.log();
        slide.log();
        arm.log();
        intake.log();
        telemetry.addData("Gamepad1 Right Trigger: ", gamepad1.right_trigger);
    }


    @Override
    public void stop() {
        resetSlidePosition = true;
    }

    private void activateArm() {
        if (arm.isActive()) {
            return;
        }
        arm.activate();
        intake.setWristActive(true);
    }
    private void dropArmUnsafe() {
        if (!arm.isActive()) {
            return;
        }
        arm.deactivate();
        intake.setWristActive(false);
    }
    private void deactivateArm() {
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);
        dropArmUnsafe();
    }
}
