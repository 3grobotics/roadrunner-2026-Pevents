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
        Pose2d initialPose = new Pose2d(-10, -24, Math.toRadians(-90));
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
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(270)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(35, -55, Math.toRadians(270)), Math.toRadians(270))

                //.afterDisp(10, robot.stopFire())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -28, Math.toRadians(0)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20,-24), Math.toRadians(180))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())


                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60.12616976039617,  -36.806707907849415, Math.toRadians(0)), Math.toRadians(180)/*, adaptiveBrakeneoooom*/)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}