package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d initialPose = new Pose2d(-20, 18, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16, 16)

                .setStartPose(initialPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)

                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(0, 24, Math.toRadians(90)), Math.toRadians(90))
                //        .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, 47, Math.toRadians(90)), Math.toRadians(90))
                //.waitSeconds(.5)

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, 56, Math.toRadians(100)), Math.toRadians(90)/*, baseVelConstraint3*/)
                //.waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, 57, Math.toRadians(120)), Math.toRadians(45))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}