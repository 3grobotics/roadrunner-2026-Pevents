package org.firstinspires.ftc.teamcode.roadrunner.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "drive test", group = "competition")
public class driveTest extends LinearOpMode {
    DcMotorEx             frontRight    ;
    DcMotorEx             backRight     ;
    DcMotorEx             frontLeft     ;
    DcMotorEx             backLeft      ;
    DcMotorEx             topIntake     ;
    DcMotorEx             middleIntake  ;
    DcMotorEx             bottomIntake  ;
    DcMotorEx             leftFlywheel  ;
    DcMotorEx             rightFlywheel ;
    Servo                 turret1       ;
    Servo                 turret2       ;
    Servo                 gate          ;
    Servo                 flower        ;
    GoBildaPinpointDriver pip           ;


    double tx = 18; // Target X
    double ty = -13;  // Target Y
    double tiltedTx = -18; // Target X
    double tiltedTy = -13;  // Target Y



    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;
    double riggedHypot;

    double angleToGoal;
    double robotHeading;
    double calculatedTurretRad;
    double finalServoDegrees;
    double offset = 10;





    public flywheelSub fly;

    public class Robot {

        private MecanumDrive driver;

        public Robot(MecanumDrive driver) {
            this.driver = driver;
        }

        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                topIntake.setPower(1);
                middleIntake.setPower(1);
                bottomIntake.setPower(1);
                gate.setPosition(.6);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class fire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                topIntake.setPower(1);
                middleIntake.setPower(1);
                bottomIntake.setPower(1);
                gate.setPosition(.5);
                return false;
            }
        }
        public Action fire() { return new fire(); }

        private class stopFire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                topIntake.setPower(0);
                middleIntake.setPower(0);
                bottomIntake.setPower(0);
                gate.setPosition(.6);
                return false;
            }
        }
        public Action stopFire() { return new stopFire(); }

        private class flowerUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flower.setPosition(.5);
                return false;
            }
        }
        public Action flowerUp() { return new flowerUp(); }

        private class flowerDown implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flower.setPosition(0);
                return false;
            }
        }
        public Action flowerDown() { return new flowerDown(); }

        private class flowerDown2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flower.setPosition(0);
                return false;
            }
        }
        public Action flowerDown2() { return new flowerDown2(); }

        private class flywheelUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                if (opModeIsActive()){



                    // 2. Use RoadRunner's offset-adjusted field coordinates
                    //robotX = pip.getPosX(DistanceUnit.INCH);
                    //robotY = pip.getPosY(DistanceUnit.INCH);

                    robotX = driver.localizer.getPose().position.x;
                    robotY = driver.localizer.getPose().position.y;

                    if(robotX > 0){
                        xl = tx - robotX;
                        yl = ty - robotY;
                    } else{
                        xl = tiltedTx - robotX;
                        yl = tiltedTy - robotY;
                    }

                    hypot = Math.sqrt((xl * xl) + (yl * yl));

                    fly.runFlywheel();
                    fly.hypot = hypot + offset;
                    fly.loop();

                    angleToGoal = Math.atan2(yl, xl);
                    robotHeading = pip.getHeading(AngleUnit.RADIANS);

                    calculatedTurretRad = angleToGoal - robotHeading;

                    while (calculatedTurretRad > Math.PI) {
                        calculatedTurretRad -= 2 * Math.PI;
                    }
                    while (calculatedTurretRad < -Math.PI) {
                        calculatedTurretRad += 2 * Math.PI;
                    }

                    finalServoDegrees = Math.toDegrees(calculatedTurretRad) + (543.636 / 2);

                    //turret1.setPosition(finalServoDegrees / 543.636);
                    //turret2.setPosition(finalServoDegrees / 543.636);



                    pip.update();
                    return true;
                }
                else return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }

        private class flywheel1st implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                riggedHypot = 54.603;
                return false;
            }
        }
        public Action flywheel1st() { return new flywheel1st(); }

        private class firstShotTurret implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.46);
                turret2.setPosition(.46);
                return false;
            }
        }
        public Action firstShotTurret() { return new firstShotTurret(); }

        private class lowerVelocity implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = -4;
                return false;
            }
        }
        public Action lowerVelocity() { return new lowerVelocity(); }

        private class lowerVelocity2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = -2;
                return false;
            }
        }
        public Action lowerVelocity2() { return new lowerVelocity2(); }

        private class lowerVelocity3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = 4;
                return false;
            }
        }
        public Action lowerVelocity3() { return new lowerVelocity3(); }

        private class secondShotTurret implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.26);
                turret2.setPosition(.26);
                return false;
            }
        }
        public Action secondShotTurret() { return new secondShotTurret(); }

        private class thirdShotTurret implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.19);
                turret2.setPosition(.19);
                return false;
            }
        }
        public Action thirdShotTurret() { return new thirdShotTurret(); }


    }

    @Override
    public void runOpMode() throws InterruptedException {


        fly = new flywheelSub(hardwareMap);
        topIntake    = hardwareMap.get(DcMotorEx.class,  "topIntake");

        middleIntake   = hardwareMap.get(DcMotorEx.class,  "middleIntake");

        bottomIntake   = hardwareMap.get(DcMotorEx.class,  "bottomIntake");


        rightFlywheel = hardwareMap.get(DcMotorEx.class,  "rightFlywheel");
        //leftFlywheel  = hardwareMap.get(DcMotorEx.class,  "leftFlywheel");
        gate = hardwareMap.get(Servo.class,  "gate");
        flower = hardwareMap.get(Servo.class,  "flower");

        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);

        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        pip.setPosition(new Pose2D(DistanceUnit.INCH, 62.8, -40, AngleUnit.DEGREES, 180));


        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(62.8, -40, Math.toRadians(180));

        // MecanumDrive takes control of the Pinpoint here and sets the -53, 53 offset internally
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Robot robot = new Robot(drive);

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)

                .stopAndAdd(robot.firstShotTurret())
                .stopAndAdd(robot.lowerVelocity())
                .waitSeconds(3)
                // flywheel up
                //fire
                .stopAndAdd(robot.fire())
                .waitSeconds(.4)
                .stopAndAdd(robot.stopFire())
                //intake off

                //intake and flowerdown in the middle of the path

                .afterDisp(15, robot.flowerDown())
                .afterDisp(20, robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -58, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(22, -58, Math.toRadians(270)), Math.toRadians(280))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(26, -58, Math.toRadians(270)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -58, Math.toRadians(270)), Math.toRadians(180))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(24, -55, Math.toRadians(270)), Math.toRadians(90))

                .waitSeconds(1)
                .stopAndAdd(robot.secondShotTurret())


                // stop intake
                // flower up in the middle of the path

                .afterDisp(5, robot.flowerUp())
                .afterDisp(10, robot.stopFire())
                .afterDisp(5, robot.lowerVelocity2())

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, -40, Math.toRadians(180)), Math.toRadians(180))


                .waitSeconds(1)

                .stopAndAdd(robot.fire())

                .waitSeconds(1)

                .stopAndAdd(robot.stopFire())
                .stopAndAdd(robot.thirdShotTurret())



                //.waitSeconds(.1)

                //flower down in middle of path
                .afterDisp(15, robot.flowerDown())
                .afterDisp(20, robot.intake())
                .afterDisp(20,robot.lowerVelocity3())

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50, -24, Math.toRadians(180)), Math.toRadians(180))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -24, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -22, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-57, -26, Math.toRadians(180)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())


                //stop fire
                // flower up mid path

                .afterDisp(5, robot.flowerUp())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-52, -57, Math.toRadians(180)), Math.toRadians(270))



                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(

                // The Parallel action runs your path, background mechanisms, and live telemetry feed together!
                new ParallelAction(
                        TEST,
                        robot.flywheelUp(),
                        (telemetryPacket) -> {

                            telemetry.addData("Distance from goal", hypot);
                            telemetry.addData("Fake distance from goal", riggedHypot);
                            telemetry.addData("angle to goal", angleToGoal);
                            telemetry.addData("heading", robotHeading);
                            telemetry.update();

                            return true;
                        }
                )
        ));
        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}