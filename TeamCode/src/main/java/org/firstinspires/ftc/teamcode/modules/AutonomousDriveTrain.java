package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;

@Config
public class AutonomousDriveTrain extends FieldCentricDriveTrain implements MotorPowerUpdater {
    private MotorPowerUpdater[] updatableMechanisms = new MotorPowerUpdater[0];

    private Pose2D targetPose;
    public static final DistanceUnit TRANSLATE_UNIT = DistanceUnit.INCH;
    public static final AngleUnit ROTATE_UNIT = AngleUnit.DEGREES;

    //PIDs for control
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController hController;

    // Timer for information
    ElapsedTime timer = new ElapsedTime();

    // PID Coefficients for translation
    public static double TRANSLATE_P = 0.15;
    public static double TRANSLATE_I = 0.001;
    public static double TRANSLATE_D = 0.01;

    // PID Coefficients for rotation
    public static double ROTATE_P = 0.06;
    public static double ROTATE_I = 0;
    public static double ROTATE_D = 0.0;

    public static double TRANSLATE_TOLERANCE = 0.5;
    public static double ROTATE_TOLERANCE = 2;

    /**
     * Creates a AutonomousDriveTrain object using the provided opmode
     * Can be used for the entirety of an OpMode by changing the targetPosition
     * @param registrar the OpMode initializing this module
     */
    public AutonomousDriveTrain(OpMode registrar) {
        super(registrar);

        // initialize PID controllers
        xController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        yController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        hController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);

        xController.setTolerance(TRANSLATE_TOLERANCE);
        yController.setTolerance(TRANSLATE_TOLERANCE);
        hController.setTolerance(ROTATE_TOLERANCE);
    }

    @Override
    public void updateMotorPowers() {
        if (xController.atSetPoint() && yController.atSetPoint() && hController.atSetPoint()) {
            setVelocity(0, 0, 0);
            return;
        }

        final Pose2D currentRobotPose = getRobotPose();

        // updates PID controllers with current robot position (for the given axis)
        final double xPower = xController.calculate(currentRobotPose.getX(TRANSLATE_UNIT));
        final double yPower = yController.calculate(currentRobotPose.getY(TRANSLATE_UNIT));
        final double hPower = hController.calculate(currentRobotPose.getHeading(ROTATE_UNIT));

        // set driveTrain velocity based on PID controller output
        setVelocity(xPower, yPower, hPower);

        // update all updatableMechanisms
        for(MotorPowerUpdater mechanism : updatableMechanisms){
            mechanism.updateMotorPowers();
        }
    }

    /**
     * Checks if the motors need to be updated
     *
     * @return true if {@link #updateMotorPowers()} needs to be called, false otherwise
     */
    @Override
    public boolean isUpdateNecessary() {
        // use 'and' instead of 'or' since we are only at our target if x, y, and heading have been reached
        if (xController.atSetPoint() && yController.atSetPoint() && hController.atSetPoint()) {
            // we are at our target
            // since this method will return false, updateMotorPowers() won't be run, so we have to
            // stop the robot ourselves
            setVelocity(0, 0, 0);
            return false; // no update necessary
        }
        return true; // we haven't reached our target, so the PID loop should be called again
    }

    /**
     * moves the robot to the target position
     * @throws InterruptedException The opmode has stopped
     * @deprecated This method has been superseded by functionality in {@link org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase AutonomousBase}
     */
    @Deprecated
    public void move() throws InterruptedException {
        // used the end of the move to report the duration of the move
        timer.reset();

        // avoids a NullPointerException
        if(targetPose == null) {
            return;
        }

        // loop for driveTrain position PID, etc.
        while(!xController.atSetPoint() || !yController.atSetPoint() || !hController.atSetPoint()) {
            /* TODO this is a jank patch, eventually this loop should be refactored into separate
             *  methods in this class and the auto classes
             */
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }

            updateMotorPowers();

            // update all telemetry
            getTelemetry().update();
        }

        // set the driveTrain velocity to 0 so that the robot doesn't move after the method ends
        setVelocity(0,0,0);

        // adds the time taken to move() to telemetry
        getTelemetry().addData("move() runtime: ", timer.time());
        getTelemetry().update();
    }

    /**
     * sets the target position and moves the robot to the target position
     * @param target the target robot position to be reached at the end of the move()
     * @throws InterruptedException The opmode has stopped
     * @deprecated This method has been superseded by functionality in {@link org.firstinspires.ftc.teamcode.opmode.autonomous.AutonomousBase AutonomousBase}
     */
    @Deprecated
    public void move(Pose2D target) throws InterruptedException {
        setTargetPose(target);
        move();
    }

    /**
     * a secondary method of changing the targetPosition without calling move()
     * @param target the Pose2D of the targetPosition of the robot for the next move()
     */
    public void setTargetPose(Pose2D target){
        targetPose = target;

        xController.setSetPoint(targetPose.getX(TRANSLATE_UNIT));
        yController.setSetPoint(targetPose.getY(TRANSLATE_UNIT));
        hController.setSetPoint(targetPose.getHeading(ROTATE_UNIT));
    }

    /**
     * sets mechanisms that need to be updated during the move()
     * @param mechanisms an array of mechanisms that need to have their control loops run during move()
     */
    public void setUpdatableMechanisms(MotorPowerUpdater[] mechanisms){
        updatableMechanisms = mechanisms;
    }
}
