package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;

public class Odometry {
    private final boolean USE_OTOS = true;

    public static final DistanceUnit distanceUnit = DistanceUnit.CM;
    public static final AngleUnit angleUnit = AngleUnit.RADIANS;

    private double x;
    private double y;
    private double h;

    SparkFunOTOS otos;

    public Odometry(HardwareMap hardwareMap){
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // sets units for distance and heading
        otos.setLinearUnit(distanceUnit);
        otos.setAngularUnit(angleUnit);

        // offsets the Pose2D that the OTOS provides to account for non-centered positioning on the robot
        otos.setOffset(new SparkFunOTOS.Pose2D(0,0,0));

        // to be edited based on calibration/testing
        otos.setLinearScalar(1.0655);
        otos.setAngularScalar(1.0058);

        // calibrates the imu, takes 612ms
        // robot must be stationary at this time
        otos.calibrateImu();

        //resets the position
        otos.resetTracking();
    }

    public Pose2D getPose(){
        return updateOdometry();
    }

    private Pose2D updateOdometry(){
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        x = otosPose.x;
        y = otosPose.y;
        h = otosPose.h;

        Pose2D realPose = new Pose2D(
                distanceUnit,
                x,
                y,
                angleUnit,
                h
                );

        return realPose;
    }

    public void setOdometryPose(double x, double y, double h){
        otos.setOffset(new SparkFunOTOS.Pose2D(x, y, h));

    }
}
