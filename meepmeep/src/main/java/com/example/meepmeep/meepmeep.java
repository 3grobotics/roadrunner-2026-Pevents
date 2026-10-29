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
        Pose2d initialPose = new Pose2d(62, -39, Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16, 16)

                .setStartPose(initialPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)

                .waitSeconds(4)
                // flywheel up
                //fire
                //intake off

                //intake and flowerdown in the middle of the path

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -52, Math.toRadians(270)), Math.toRadians(270))

                .waitSeconds(1)


                // stop intake
                // flower up in the middle of the path

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-45, -52, Math.toRadians(180)), Math.toRadians(180))

                //.waitSeconds(.1)

                //flower down in middle of path

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-45, -24, Math.toRadians(180)), Math.toRadians(180))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52, -24, Math.toRadians(180)), Math.toRadians(180))

                //fire

                .waitSeconds(1)

                //stop fire
                // flower up mid path

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-52, -57, Math.toRadians(180)), Math.toRadians(270))



                .build());

        /*flower to wall dump pickup
        .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, -52, Math.toRadians(315)), Math.toRadians(0))
         .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(59, -40, Math.toRadians(70)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(59, -12, Math.toRadians(70)), Math.toRadians(90))*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_BIOBUZZ_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}