package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.action.PID_ToPoint;
import org.firstinspires.ftc.teamcode.hardware.UpdateableMotorPower;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

@Config
@Autonomous
public class BasketAutonomous extends LinearOpMode {
    //config variables for positions
    public static double X1 = -33;
    public static double Y1 = 12;
    public static double H1 = 135;

    public static double X2 = -4;
    public static double Y2 = 40;
    public static double H2 = -90;

    public static double X3 = 0;
    public static double Y3 = 40;
    public static double H3 = -90;

    // position for move 1
    public final Pose2D move1 = new Pose2D(PID_ToPoint.TRANSLATE_UNIT, X1, Y1, PID_ToPoint.ROTATE_UNIT, H1);

    // position for move 2
    public final Pose2D move2 = new Pose2D(PID_ToPoint.TRANSLATE_UNIT, X2, Y2, PID_ToPoint.ROTATE_UNIT, H2);

    // positions for move 3
    public final Pose2D move3 = new Pose2D(PID_ToPoint.TRANSLATE_UNIT, X3, Y3, PID_ToPoint.ROTATE_UNIT, H3);

    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        final ModuleManager moduleManager = new ModuleManager(this);
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        TeleOpMain.resetSlidePosition = false;

        PID_ToPoint movementPID = new PID_ToPoint(driveTrain, this);
        movementPID.setUpdateableMechanisms(new UpdateableMotorPower[]{arm, slide});

        waitForStart();

        // get arm out of way
        slide.setTargetHeight(0);
        slide.updateMotorPower();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPower();
        Thread.sleep(TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);

        while(!arm.monitorPositionSwitch()){
            slide.updateMotorPower();
            Thread.sleep(10);
        }
        arm.activate();
        arm.setTargetRotation(Arm.ARM_ROTATION_SCORING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_SCORING);
        intake.moveWristTo(Intake.WRIST_POSITION_SCORING);

        movementPID.move(move1);

        intake.eject();
        timer.reset();
        while(timer.time() < 0.5){
            slide.updateMotorPower();
            arm.updateMotorPower();
        }
        intake.settle();

        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_HANG_LVL1);
        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);

        movementPID.move(move2);

        arm.setTargetRotation(Arm.ARM_ROTATION_HANG_LVL1_SETUP);

        movementPID.move(move3);

        arm.deactivate();

        while(!isStopRequested()){
            arm.updateMotorPower();
            slide.updateMotorPower();
        }

    }
}
