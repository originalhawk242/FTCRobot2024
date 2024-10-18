package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modules.*;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * An Autonomous OpMode
 * Currently, this OpMode simply moves the robot in a square.
 * The following path is for later implementation.
 * This OpMode is for the following Autonomous path:
 * The robot starts on the right side of the field,
 * Then moves to pick up samples on the right side of the field,
 * Then deposits them in the sample bucket
 */

/*
 * Abbreviation of class name is as follows:
 * {Side} Start To {Side} Samples To {Target}
 * For example: Right Start To Right Samples To Bucket
 * This means the robot starts on the right side of the arena,
 * goes and grabs the samples of the right side of the arena,
 * and then goes to the bucket
 * {Side} can be Right (R) or Left (L)
 * {Target} can be Bucket or Chamber
 */

@Autonomous(name="Right Side To Right Samples To Bucket")
public class RToRToBucket extends OpMode {

    private MecanumDrive rrMecanumDrive;
    private Pose2d beginPose;

    private LinearSlide slide;

    private Arm arm;

    private Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        beginPose = new Pose2d(0, 0, 0);

        rrMecanumDrive = new MecanumDrive(hardwareMap, beginPose);

        slide = new LinearSlide(this);

        arm = new Arm(this);

        intake = new Intake(this);
    }

    @Override
    public void loop() {
        Actions.runBlocking(
                rrMecanumDrive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, 10))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(10, 10))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(10, 0))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(0, 0))
                        .waitSeconds(1)
                        .build()
        );
    }
}
