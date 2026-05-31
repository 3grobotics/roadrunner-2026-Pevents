package com.example.meepmeep2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.PI, Math.PI, 16)
                .build();
        Pose2d initialPose = new Pose2d(-49, 51, Math.toRadians(121));
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(7, 34, Math.toRadians(90)), Math.toRadians(90))
                // .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(7, 45, Math.toRadians(90)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}