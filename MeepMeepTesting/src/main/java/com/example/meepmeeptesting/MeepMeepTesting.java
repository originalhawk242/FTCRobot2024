package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(6, -36),Math.toRadians(90))
                .waitSeconds(2)
                        .lineToY(-40)
                //.splineTo(new Vector2d(-6, -40), Math.toRadians(0))
                //.splineTo(new Vector2d(-60, -60), Math.toRadians(225))
                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6, -40, Math.toRadians(90)))
                .splineTo(new Vector2d(-6, -50), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}