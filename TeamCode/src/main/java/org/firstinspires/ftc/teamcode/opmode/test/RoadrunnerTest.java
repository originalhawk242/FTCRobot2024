package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class RoadrunnerTest extends OpMode {

    private MecanumDrive rrMecanumDrive;
    private MecanumDrive.TurnAction turn;

    private Pose2d beginPose;

    @Override
    public void init() {
        beginPose = new Pose2d(0, 0, 0);
        rrMecanumDrive = new MecanumDrive(hardwareMap, beginPose);
    }

    @Override
    public void loop() {
        Actions.runBlocking(
                rrMecanumDrive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(10, 0))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(0, 0))
                        .waitSeconds(2)
                        .build());
    }
}
