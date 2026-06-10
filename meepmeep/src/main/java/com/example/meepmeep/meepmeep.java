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
        Pose2d initialPose = new Pose2d(-53, -53, Math.toRadians(-37.5));
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
                .setTangent(Math.toRadians(-315))
                .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(-37)), Math.toRadians(-315))

                // intake first spike (fourth, fifth and sixth artifact pickup)
                //.afterDisp(0, robot.lowerVelocity())
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToSplineHeading(new Pose2d(9, -30, Math.toRadians(-90)), Math.toRadians(-90))
                //.waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(9, -45, Math.toRadians(-90)), Math.toRadians(-90)/*, slow_as_hell*/)
                /*   //      .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(8, 47, Math.toRadians(-120)), Math.toRadians(-115))
                //.waitSeconds(.5)

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(6, 55, Math.toRadians(-90)), Math.toRadians(-90)) */


                // drive to zone for first spike mark shot (fourth, fifth and sixth artifact shot)
                ////.afterDisp(5, robot.stopFire())
                //.afterDisp(5, robot.turret2())
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(5, -36, Math.toRadians(-90)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(-90)), Math.toRadians(-225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                //cc here

                // first gate pickup (thirteenth, fourteenth and fifteenth artifact pickup)
                // heading (deg): 124.12017218615657
                //x: 8.69645632773257
                //y: 55.082679522912116
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToLinearHeading(new Pose2d(0, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(6.69645632773257, -58.082679522912116, Math.toRadians(-120.12017218615657)), Math.toRadians(-90))
                /*.setTangent(Math.toRadians(-90))
                .waitSeconds(.00001)
                .splineToSplineHeading(new Pose2d(15, 60, Math.toRadians(-135)), Math.toRadians(-45))*/
                .waitSeconds(1)


                // drive to zone for first gate pickup shot (fourth, fifth and sixth artifact shot)
                //.afterDisp(24, robot.stopFire())
                //.afterDisp(5, robot.turret2())
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(5, -36, Math.toRadians(-90)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(-90)), Math.toRadians(-225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                // second gate pickup (thirteenth, fourteenth and fifteenth artifact pickup)
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToLinearHeading(new Pose2d(0, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(6.69645632773257, -58.082679522912116, Math.toRadians(-120.12017218615657)), Math.toRadians(-90))
                /*.setTangent(Math.toRadians(-90))
                .waitSeconds(.00001)
                .splineToSplineHeading(new Pose2d(15, 60, Math.toRadians(-135)), Math.toRadians(-45))*/
                .waitSeconds(1)


                // drive to zone for second gate pickup shot (fourth, fifth and sixth artifact shot)
                //.afterDisp(24, robot.stopFire())
                //.afterDisp(5, robot.turret2())
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(5, -36, Math.toRadians(-90)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(-90)), Math.toRadians(-225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                // third gate pickup (thirteenth, fourteenth and fifteenth artifact pickup)
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToLinearHeading(new Pose2d(0, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(6.69645632773257, -58.082679522912116, Math.toRadians(-120.12017218615657)), Math.toRadians(-90))
                /*.setTangent(Math.toRadians(-90))
                .waitSeconds(.00001)
                .splineToSplineHeading(new Pose2d(15, 60, Math.toRadians(-135)), Math.toRadians(-45))*/
                .waitSeconds(1)


                // drive to zone for third gate pickup shot (fourth, fifth and sixth artifact shot)
                //.afterDisp(24, robot.stopFire())
                //.afterDisp(5, robot.turret2())
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(5, -36, Math.toRadians(-90)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(-90)), Math.toRadians(-225))
                //.stopAndAdd(robot.fire())
                .waitSeconds(1)
                //.stopAndAdd(robot.stopFire())

                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-45, -10, Math.toRadians(-90)), Math.toRadians(-180)/*, adaptiveBrakeneoooom*/)

/*
                    // intake second spike mark (seventh, eighth and ninth artifact pickup)
                    //.afterDisp(0, robot.intake())
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(-45)), Math.toRadians(-30))

                    // back to zone after flush (seventh, eighth and ninth artifact shot)
                    .setTangent(Math.toRadians(-270))
                    .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-90)), Math.toRadians(-225))
                    //.stopAndAdd(robot.fire())
                    .waitSeconds(1)
                    //.stopAndAdd(robot.stopFire())

               /* // intake third spike mark (tenth, eleventh and twelfth artifact pickup)
                //.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-45))
                .splineToLineeading(new Pose2d(15, 49, Math.toRadians(-0)), Math.toRadians(-0))
                .setTangent(Math.toRadians(-0))
                .splineToSplineHeading(new Pose2d(38, 49, Math.toRadians(-0)), Math.toRadians(-0)/*, baseVelConstraint2)
                //.stopAndAdd(robot.stopFire())
*/

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}