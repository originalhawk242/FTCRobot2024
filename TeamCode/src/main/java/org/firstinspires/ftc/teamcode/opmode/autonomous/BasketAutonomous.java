package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.action.PIDToPoint;
import org.firstinspires.ftc.teamcode.hardware.MotorPowerUpdater;
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
    public static double X1 = -27;
    public static double Y1 = 9;
    public static double H1 = -45;

    public static double X2 = -26;
    public static double Y2 = 10;
    public static double H2 = 25;

    public static double X3 = -26;
    public static double Y3 = 16;
    public static double H3 = 25;

    // position for move 1
    public final Pose2D move1 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X1, Y1, PIDToPoint.ROTATE_UNIT, H1);

    // position for move 2
    public final Pose2D move2 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X2, Y2, PIDToPoint.ROTATE_UNIT, H2);

    // positions for move 3
    public final Pose2D move3 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X3, Y3, PIDToPoint.ROTATE_UNIT, H3);

    private static ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        final ModuleManager moduleManager = new ModuleManager(this);
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        TeleOpMain.resetSlidePosition = false;

        PIDToPoint movementPID = new PIDToPoint(driveTrain, this);
        movementPID.setUpdatableMechanisms(new MotorPowerUpdater[]{arm, slide});

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
            Thread.sleep(5);
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
            Thread.sleep(5);
        }
        intake.settle();

        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);

        timer.reset();
        while(timer.time() < 0.75){
            arm.updateMotorPower();
            slide.updateMotorPower();
        }

        arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_INTAKE);
        intake.moveWristTo(Intake.WRIST_POSITION_INTAKE);

        movementPID.move(move2);

        intake.grab();

        movementPID.move(move3);

        intake.settle();


        while(!isStopRequested()){
            arm.updateMotorPower();
            slide.updateMotorPower();
        }

    }
}
