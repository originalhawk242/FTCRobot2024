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

    public static double SCORING_X = -25;
    public static double SCORING_Y = 13;
    public static double SCORING_HEADING = 135;

    public static double PARKING_X = 0;
    public static double PARKING_Y = 0;
    public static double PARKING_HEADING = 0;

    @Deprecated
    public static double X2 = -26;
    @Deprecated
    public static double Y2 = 10;
    @Deprecated
    public static double H2 = 25;

    @Deprecated
    public static double X3 = -26;
    @Deprecated
    public static double Y3 = 16;
    @Deprecated
    public static double H3 = 25;

    public static double INTAKE1_X = 6;
    public static double INTAKE1_Y = 12;
    public static double INTAKE1_HEADING = 45;

    public static double INTAKE2_X = 0;
    public static double INTAKE2_Y = 12;
    public static double INTAKE2_HEADING = 45;

    public static double INTAKE3_X = -6;
    public static double INTAKE3_Y = 12;
    public static double INTAKE3_HEADING = 45;

    /*
     * ! IMPORTANT !
     * These poses are instance fields so that they get updated with the above values whenever
     * autonomous is rerun.  This way, the fields above can be updated in FTC Dashboard with the
     * correct positions during tuning.
     */

    public final Pose2D intake1 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, INTAKE1_X, INTAKE1_Y, PIDToPoint.ROTATE_UNIT, INTAKE1_HEADING);

    public final Pose2D intake2 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, INTAKE2_X, INTAKE2_Y, PIDToPoint.ROTATE_UNIT, INTAKE2_HEADING);

    public final Pose2D intake3 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, INTAKE3_X, INTAKE3_Y, PIDToPoint.ROTATE_UNIT, INTAKE3_HEADING);

    public final Pose2D scoring = new Pose2D(PIDToPoint.TRANSLATE_UNIT, SCORING_X, SCORING_Y, PIDToPoint.ROTATE_UNIT, SCORING_HEADING);

    public final Pose2D parking = new Pose2D(PIDToPoint.TRANSLATE_UNIT, PARKING_X, PARKING_Y, PIDToPoint.ROTATE_UNIT, PARKING_HEADING);

    @Deprecated
    public final Pose2D move2 = new Pose2D(PIDToPoint.TRANSLATE_UNIT, X2, Y2, PIDToPoint.ROTATE_UNIT, H2);

    @Deprecated
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
        scoreHighBasket(); //score preload

        /* Intake & score the 1st sample */
        movementPID.move(intake1);
        intakeSample();
        scoreHighBasket();

        /* Intake & score the 2nd sample */
        movementPID.move(intake2);
        intakeSample();
        scoreHighBasket();

        // TODO make movement not stall when robot hits wall
        /* Intake & score the 3rd sample */
        movementPID.move(intake3);
        intakeSample();
        scoreHighBasket();

        /* park */
        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        movementPID.move(parking);

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

    }

    @Deprecated
    private void intakeSample(PIDToPoint movementPID, Intake intake, Arm arm, LinearSlide slide) throws InterruptedException {
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

    protected void scoreHighBasket() throws InterruptedException {
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        PIDToPoint movementPID = new PIDToPoint(driveTrain, this);

        arm.setTargetRotation(Arm.ARM_ROTATION_SCORING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_SCORING);
        intake.moveWristTo(Intake.WRIST_POSITION_SCORING);

        movementPID.move(scoring);

        intake.eject();
        waitForTime(500); //milliseconds
        intake.settle();

        // move a bit back from the basket so that the arm can safely move down
        movementPID.move(new Pose2D(
                DistanceUnit.INCH,
                scoring.getX(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
                scoring.getY(DistanceUnit.INCH) + SAFE_MOVE_DISTANCE_X_AND_Y,
                AngleUnit.DEGREES,
                scoring.getHeading(AngleUnit.DEGREES)
        ));

        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
        waitForTime(500);
    }

    protected void intakeSample() throws InterruptedException {
        final Arm arm = moduleManager.getModule(Arm.class);
        final LinearSlide slide = moduleManager.getModule(LinearSlide.class);
        final Intake intake = moduleManager.getModule(Intake.class);
        final FieldCentricDriveTrain driveTrain = moduleManager.getModule(FieldCentricDriveTrain.class);
        PIDToPoint movementPID = new PIDToPoint(driveTrain, this);

        intake.grab();
        arm.setTargetRotation(Arm.ARM_ROTATION_INTAKE);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_INTAKE);
        intake.moveWristTo(Intake.WRIST_POSITION_INTAKE);
        waitForTime(1000); // wait a bit for the arm & slide to move in place

        // move forward so that the sample is caught
        final Pose2D curPose = driveTrain.getRobotPose();
        final double curAngleTrig = curPose.getHeading(AngleUnit.RADIANS) + (Math.PI / 2);
        movementPID.move(new Pose2D(
                DistanceUnit.INCH,
                curPose.getX(DistanceUnit.INCH) + Math.cos(curAngleTrig),
                curPose.getY(DistanceUnit.INCH) + Math.sin(curAngleTrig),
                PIDToPoint.ROTATE_UNIT,
                curPose.getHeading(PIDToPoint.ROTATE_UNIT)
        ));

        intake.settle();
        arm.setTargetRotation(Arm.ARM_ROTATION_MOVING);
        slide.setTargetHeight(LinearSlide.SLIDE_HEIGHT_MOVING);
        intake.moveWristTo(Intake.WRIST_POSITION_MOVING);
    }
}