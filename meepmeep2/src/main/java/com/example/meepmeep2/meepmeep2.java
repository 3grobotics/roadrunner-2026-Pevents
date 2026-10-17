package com.example.meepmeep2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d initialPose = new Pose2d(-37, -40, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 16)
                .setDimensions(16, 16)

                .setStartPose(initialPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)


                // first volley driving away from zone (first, second and third artifact shot)


                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-37, 0, Math.toRadians(90)), Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(37, 0, Math.toRadians(0)), Math.toRadians(0))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(37, -45, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-37, -40, Math.toRadians(90)), Math.toRadians(180))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_BIOBUZZ_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}