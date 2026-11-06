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
        Pose2d initialPose = new Pose2d(62, -16, Math.toRadians(270));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 16)
                .setDimensions(16, 16)

                .setStartPose(initialPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)


                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(55, -37, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(55, -60, Math.toRadians(0)), Math.toRadians(315))



                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(61, -62, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(57, -62, Math.toRadians(0)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(61, -62, Math.toRadians(0)), Math.toRadians(0))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(57, -62, Math.toRadians(0)), Math.toRadians(180))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62, -16, Math.toRadians(180)), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_BIOBUZZ_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}