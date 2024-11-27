package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.core.Module;

@Config
public class Odometry extends Module {
    public static double ANGULAR_SCALAR = 1.0058;
    public static double LINEAR_SCALAR = 1.0655;
    private final boolean USE_OTOS = true;

    public static final DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static final AngleUnit angleUnit = AngleUnit.DEGREES;

    private double x;
    private double y;
    private double h;

    private final SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D previousPosition = new SparkFunOTOS.Pose2D(0,0,0);

    public Odometry(OpMode registrar){
        super(registrar);

        otos = registrar.hardwareMap.get(SparkFunOTOS.class, "otos");

        // sets units for distance and heading
        otos.setLinearUnit(distanceUnit);
        otos.setAngularUnit(angleUnit);

        // offsets the Pose2D that the OTOS provides to account for non-centered positioning on the robot
        otos.setOffset(new SparkFunOTOS.Pose2D(4.5,2,0));

        // to be edited based on calibration/testing
        otos.setLinearScalar(LINEAR_SCALAR);
        otos.setAngularScalar(ANGULAR_SCALAR);

        // calibrates the imu, takes 612ms
        // robot must be stationary at this time
        otos.calibrateImu();

        //resets the position
        otos.resetTracking();
    }

    /**
     * Returns the calculated Pose
     * @return the output of the updateOdometry method, which updates the Pose and returns a Pose2D
     */
    public Pose2D getPose(){
        return updateOdometry();
    }

    /**
     * Updates the robot position and returns it as a Pose2D
     * @return a Pose2D of the current robot position
     */
    private Pose2D updateOdometry(){
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        x += otosPose.x - previousPosition.x;
        y += otosPose.y - previousPosition.y;
        h += otosPose.h - previousPosition.h;

        final Pose2D realPose = new Pose2D(
                distanceUnit,
                x,
                y,
                angleUnit,
                h
                );

        previousPosition = otosPose;

        return realPose;
    }

    /**
     * can be used to set a specific robot position, overriding whatever the sensors think
     * @param x the x-value of the position being set (can be in whatever reference frame you choose)
     * @param y the y-value of the position being set (can be in whatever reference frame you choose)
     * @param h the h-value of the position being set (can be in whatever reference frame you choose)
     */
    public void setOdometryPose(double x, double y, double h){
        otos.setOffset(new SparkFunOTOS.Pose2D(x, y, h));

    }

    @Override
    public void ensureSafety() {

    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void log() {
        getTelemetry().addData("Robot Position", "x: %f, y: %f, h: %f",
                x,
                y,
                h
        );
    }
}
