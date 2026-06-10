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
        Pose2d initialPose = new Pose2d(-50, 49, Math.toRadians(65));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16, 16)

                .setStartPose(initialPose)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)


                // first volley driving away from zone (first, second and third artifact shot)
                .waitSeconds(1)
                //.afterDisp(0, robot.turret1())
                //.afterDisp(0, robot.raiseVelocity())

                //.afterDisp(10, robot.fire())
                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(-25, 25, Math.toRadians(37)), Math.toRadians(315))

                // intake first spike (fourth, fifth and sixth artifact pickup)
                //.afterDisp(0, robot.lowerVelocity())
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(9, 30, Math.toRadians(90)), Math.toRadians(90))
                //.waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, 45, Math.toRadians(90)), Math.toRadians(90)/*, slow_as_hell*/)
                /*   //      .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(8, 47, Math.toRadians(120)), Math.toRadians(115))
                //.waitSeconds(.5)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, 55, Math.toRadians(90)), Math.toRadians(90)) */


                // drive to zone for first spike mark shot (fourth, fifth and sixth artifact shot)
                //.afterDisp(5, robot.stopFire())
                //.afterDisp(5, robot.turret2())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(5, 36, Math.toRadians(90)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                // intake second spike mark (seventh, eighth and ninth artifact pickup)
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(45)), Math.toRadians(30))

                // back to zone after flush (seventh, eighth and ninth artifact shot)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                // intake second spike mark (seventh, eighth and ninth artifact pickup)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(27, 18, Math.toRadians(90)), Math.toRadians(0))
                //.afterDisp(0, robot.intake())

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(34, 48, Math.toRadians(45)), Math.toRadians(30))

                // back to zone after flush (seventh, eighth and ninth artifact shot)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}