package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;
import org.firstinspires.ftc.teamcode.hardware.UpdateableMotorPower;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

@Config
public class PID_ToPoint {
    private LinearOpMode program;
    private FieldCentricDriveTrain driveTrain;
    private UpdateableMotorPower[] updateableMechanisms = new UpdateableMotorPower[0];

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

    public static double TRANSLATE_TOLERANCE = 0.3;
    public static double ROTATE_TOLERANCE = 2;

    /**
     * Constructor for a PID_ToPoint object
     * Can be used for the entirety of an OpMode by changing the targetPosition
     * @param inputDriveTrain the FieldCentricDriveTrain that is meant to be moved
     * @param opMode the LinearOpMode the object is being used in (needed for isStopCalled() and telemetry)
     */
    public PID_ToPoint(FieldCentricDriveTrain inputDriveTrain, LinearOpMode opMode){
        // assign object to driveTrain
        driveTrain = inputDriveTrain;

        // assign opMode to program
        program = opMode;

        // initialize PID controllers
        xController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        yController = new PIDController(TRANSLATE_P, TRANSLATE_I,TRANSLATE_D);
        hController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);

        xController.setTolerance(TRANSLATE_TOLERANCE);
        yController.setTolerance(TRANSLATE_TOLERANCE);
        hController.setTolerance(ROTATE_TOLERANCE);
    }

    /**
     * moves the robot to the targetPosition
     */
    public void move(){
        // used the end of the move to
        timer.reset();

        // avoids a NullPointerException
        if(targetPose == null){
            return;
        }

        // loop for driveTrain position PID, etc.
        while(!xController.atSetPoint() || !yController.atSetPoint() || !hController.atSetPoint()){
            Pose2D currentRobotPose = driveTrain.getRobotPose();

            // updates PID controllers with current robot position (for the given axis)
            double xPower = xController.calculate(currentRobotPose.getX(TRANSLATE_UNIT));
            double yPower = yController.calculate(currentRobotPose.getY(TRANSLATE_UNIT));
            double hPower = hController.calculate(currentRobotPose.getHeading(ROTATE_UNIT));

            // sets driveTrain velocity based on PID controller output
            driveTrain.setVelocity(xPower, yPower, hPower);

            // update all updateableMechanisms
            for(UpdateableMotorPower mechanism : updateableMechanisms){
                mechanism.updateMotorPower();
            }

            // if the OpMode wants to stop, stop
            if(program.isStopRequested()){
                return;
            }

            // update all telemetry
            program.telemetry.update();
        }

        // set the driveTrain velocity to 0 so that the robot doesn't move after the method ends
        driveTrain.setVelocity(0,0,0);

        // adds the time taken to move() to telemetry
        program.telemetry.addData("move() runtime: ", timer.time());
        program.telemetry.update();
    }

    /**
     * sets the target position and moves the robot to the target position
     * @param target the target robot position to be reached at the end of the move()
     */
    public void move(Pose2D target){
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
     * @param updateables an UpdateableMotorPower[] of mechanisms that need to have their control loops run during move()
     */
    public void setUpdateableMechanisms(UpdateableMotorPower[] updateables){
        updateableMechanisms = updateables;
    }
}
