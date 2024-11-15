package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.action.PIDToPoint;
import org.firstinspires.ftc.teamcode.hardware.MotorPowerUpdater;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpMain;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class BasketAutonomous extends LinearOpMode {
    //config variables for positions
    public static double SAFE_MOVE_DISTANCE_X_AND_Y = 6;

    public static double X1 = -25;
    public static double Y1 = 13;
    public static double H1 = 135;

    public static double X2 = -26;
    public static double Y2 = 10;
    public static double H2 = 25;

    public static double X3 = -26;
    public static double Y3 = 16;
    public static double H3 = 25;

    public static double INTAKE1_X = 6;
    public static double INTAKE1_Y = 12;
    public static double INTAKE1_HEADING = 45;

    public Pose2D intake1 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, INTAKE1_X, INTAKE1_Y, PIDToPoint.ROTATE_UNIT, INTAKE1_HEADING);

    //public static double X4 = ;
    //public static double Y4 = ;
    //public static double H4 = ;

    // position for move 1
    public final Pose2D preload = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X1, Y1, PIDToPoint.ROTATE_UNIT, H1);

    // position for move 2
    public final Pose2D move2 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X2, Y2, PIDToPoint.ROTATE_UNIT, H2);

    // positions for move 3
    public final Pose2D move3 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X3, Y3, PIDToPoint.ROTATE_UNIT, H3);

    private static final ElapsedTime timer = new ElapsedTime();

    final ModuleManager moduleManager = new ModuleManager(this);

    @Override
    public void runOpMode() throws InterruptedException {
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        TeleOpMain.resetSlidePosition = false;

        PIDToPoint movementPID = new PIDToPoint(driveTrain, this);
        movementPID.setUpdatableMechanisms(new MotorPowerUpdater[]{arm, slide});

        waitForStart();

        /* reset arm position */

        // get arm out of way
        slide.setTargetHeight(0);
        slide.updateMotorPower();
        arm.setTargetRotationAbsolute(20);
        arm.updateMotorPower();
        Thread.sleep(TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        intake.moveWristTo(Intake.WRIST_POSITION_DEACTIVATED);

        while (!arm.monitorPositionSwitch()) {
            slide.updateMotorPower();
        }
        arm.activate();

        /* score preload */
        arm.setTargetRotation(Arm.ARM_ROTATION_SCORING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_SCORING);
        intake.moveWristTo(Intake.WRIST_POSITION_SCORING);

        movementPID.move(preload);

        intake.eject();
        waitForTime(500); //milliseconds
        intake.settle();

        // move a bit back from the basket so that the arm can safely move down
        movementPID.move(new Pose2D(
                DistanceUnit.INCH,
                preload.getX(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
                preload.getY(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
                AngleUnit.DEGREES,
                preload.getHeading(AngleUnit.DEGREES)
        ));

        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        waitForTime(500);

        /* Intake the 1st sample */
        movementPID.move(intake1);
        intake.grab();
        arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_INTAKE);
        intake.moveWristTo(Intake.WRIST_POSITION_INTAKE);
        waitForTime(1000);
        movementPID.move(new Pose2D(PIDToPoint.TRANSLATE_UNIT, INTAKE1_X - 1, INTAKE1_Y + 1, PIDToPoint.ROTATE_UNIT, INTAKE1_HEADING));
        intake.settle();
        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);

        waitForTime(1000); // remove when auto is finished

        /* cleanup */
        // we're done with autonomous -- reset the everything for teleop
        driveTrain.setVelocity(0, 0, 0);

        // we're using setrotation and updatemotorpower here not to
        // actually move the arm to a point, but to give the arm constant upward
        // motion so the intake has time to move to its start position
        arm.activate();
        if (arm.getCurrentRotationAbsolute() < 20) {
            arm.setTargetRotationAbsolute(20);
        }
        intake.moveWristTo(Intake.WRIST_POSITION_START);
        arm.updateMotorPower();
        slide.setTargetHeight(0);
        slide.updateMotorPower();
        Thread.sleep(3L * TeleOpMain.INITIAL_JUMP_TIME_MILLIS);
        arm.deactivate();

        // keep everything stable until auto ends
        while (opModeIsActive()) {
            slide.updateMotorPower();
        }

        /*
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

        */
    }
    protected final void waitForTime (long timeToWait) throws InterruptedException {
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        ElapsedTime run = new ElapsedTime();
        run.reset();
        while (run.time(TimeUnit.MILLISECONDS) < timeToWait){
            if (isStopRequested()){
                throw new InterruptedException();
            }
            slide.updateMotorPower();
            arm.updateMotorPower();
        }
    }
}
