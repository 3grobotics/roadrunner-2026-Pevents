package org.firstinspires.ftc.teamcode.roadrunner.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

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
    Limelight3A           limelight     ;

    double tx = 18;        // +X Side Target X
    double ty = -13;       // +X Side Target Y
    double tiltedTx = -18; // -X Side Target X
    double tiltedTy = -13; // -X Side Target Y

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

    boolean hiveNotTipped = false;
    int lastSeenTagId = -1;

    public flywheelSub fly;

    public class Robot {

        private MecanumDrive driver;

        public Robot(MecanumDrive driver) {
            this.driver = driver;
        }

        // 1. Check if Limelight JSON sees any of the Hive AprilTags (34-41)
        private class checkHive implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                    hiveNotTipped = true;
                    lastSeenTagId = result.getFiducialResults().get(0).getFiducialId();
                } else {
                    hiveNotTipped = false;
                    lastSeenTagId = -1;
                }
                return false;
            }
        }
        public Action checkHive() { return new checkHive(); }

        // 2. Run pathA if hiveNotTipped is true, or pathB if false
        private class branchHivePath implements Action {
            private final Action pathA;
            private final Action pathB;
            private Action chosenPath = null;

            public branchHivePath(Action pathA, Action pathB) {
                this.pathA = pathA;
                this.pathB = pathB;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (chosenPath == null) {
                    chosenPath = hiveNotTipped ? pathA : pathB;
                }
                return chosenPath.run(p);
            }
        }
        public Action branchHivePath(Action pathA, Action pathB) {
            return new branchHivePath(pathA, pathB);
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
                if (opModeIsActive()) {
                    Pose2d currentPose = driver.localizer.getPose();
                    robotX = currentPose.position.x;
                    robotY = currentPose.position.y;

                    if (robotX > 0) {
                        xl = tx - robotX;
                        yl = ty - robotY;
                    } else {
                        xl = tiltedTx - robotX;
                        yl = tiltedTy - robotY;
                    }

                    hypot = Math.sqrt((xl * xl) + (yl * yl));

                    fly.runFlywheel();
                    fly.hypot = hypot + offset;
                    fly.loop();

                    angleToGoal = Math.atan2(yl, xl);
                    robotHeading = currentPose.heading.toDouble();

                    calculatedTurretRad = angleToGoal - robotHeading;

                    while (calculatedTurretRad > Math.PI) {
                        calculatedTurretRad -= 2 * Math.PI;
                    }
                    while (calculatedTurretRad < -Math.PI) {
                        calculatedTurretRad += 2 * Math.PI;
                    }

                    finalServoDegrees = Math.toDegrees(calculatedTurretRad) + (543.636 / 2);

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

        private class firstShotTurret2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
                return false;
            }
        }
        public Action firstShotTurret2() { return new firstShotTurret2(); }

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
                offset = 6;
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

        private class secondShotTurretb implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
                return false;
            }
        }
        public Action secondShotTurretb() { return new secondShotTurretb(); }

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

        fly           = new flywheelSub(hardwareMap);
        topIntake     = hardwareMap.get(DcMotorEx.class, "topIntake");
        middleIntake  = hardwareMap.get(DcMotorEx.class, "middleIntake");
        bottomIntake  = hardwareMap.get(DcMotorEx.class, "bottomIntake");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        gate          = hardwareMap.get(Servo.class, "gate");
        flower        = hardwareMap.get(Servo.class, "flower");
        pip           = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret1       = hardwareMap.get(Servo.class, "turret1");
        turret2       = hardwareMap.get(Servo.class, "turret2");
        limelight     = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);

        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.setPosition(new Pose2D(DistanceUnit.INCH, 62, -16, AngleUnit.DEGREES, 270));

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(62, -16, Math.toRadians(270));
        Pose2d splitPose   = new Pose2d(62, -16, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot(drive);

        // PATH A (hiveNotTipped == true):
        // 1. Goes back to +X shooting pose (62.8, -40) and shoots the 1st flower to make the hive tip
        // 2. Drives across to the 2nd flower on the -X side (-50, -24), intakes it, shoots, and parks
        Action failure = drive.actionBuilder(splitPose)
                .stopAndAdd(robot.firstShotTurret2())
                .afterDisp(5, robot.flowerUp())
                .afterDisp(10, robot.stopFire())
                .afterDisp(5, robot.lowerVelocity())

                // Drive back to +X side and shoot the 1st flower to tip the hive
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(62.8, -16, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())
                .stopAndAdd(robot.thirdShotTurret())

                // Now drive across to the 2nd flower on the -X side, intake, shoot, and park
                .afterDisp(15, robot.flowerDown())
                .afterDisp(20, robot.intake())
                .afterDisp(20, robot.lowerVelocity3())

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-45, -40, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50, -24, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -24, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -22, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-57, -26, Math.toRadians(180)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())

                .afterDisp(5, robot.flowerUp())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-52, -57, Math.toRadians(180)), Math.toRadians(270))
                .build();

        // PATH B (hiveNotTipped == false):
        // Hive ALREADY tipped -> Cross directly to (-55, -40) to shoot the 1st flower,
        // then get the 2nd flower, shoot, and park
        Action success = drive.actionBuilder(splitPose)
                .stopAndAdd(robot.secondShotTurretb())
                .afterDisp(5, robot.flowerUp())
                .afterDisp(10, robot.stopFire())
                .afterDisp(5, robot.lowerVelocity2())

                .setTangent(Math.toRadians(170))
                .splineToLinearHeading(new Pose2d(-20, -12, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -24, Math.toRadians(180)), Math.toRadians(190))

                .waitSeconds(1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())
                .stopAndAdd(robot.thirdShotTurret())

                .afterDisp(15, robot.flowerDown())
                .afterDisp(20, robot.intake())
                .afterDisp(20, robot.lowerVelocity3())

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-48, -24, Math.toRadians(180)), Math.toRadians(0))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -24, Math.toRadians(180)), Math.toRadians(180))

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(270)), Math.toRadians(0))

                // SCAN
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-62.8, -12, Math.toRadians(0)), Math.toRadians(225))
                // SCAN


                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-41, -40, Math.toRadians(0)), Math.toRadians(270))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, -58, Math.toRadians(270)), Math.toRadians(280))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(26, -58, Math.toRadians(270)), Math.toRadians(0))


                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -58, Math.toRadians(270)), Math.toRadians(180))

                .build();

        // Main Path
        Action TEST = drive.actionBuilder(initialPose)
                .stopAndAdd(robot.firstShotTurret2())
                .stopAndAdd(robot.lowerVelocity())
                .waitSeconds(3)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())



                .afterDisp(10, robot.intake())
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


                .afterDisp(5, robot.stopFire())
                .afterDisp(5, robot.flowerDown())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(splitPose, Math.toRadians(0))

                // 1. Lower the flower so it doesn't block the Limelight
                // 2. Wait 0.4s for the flower to drop AND the shot to hit/tip the hive
                .waitSeconds(.5)
                // 3. Check Limelight while the view is clear
                .stopAndAdd(robot.checkHive())
                // 4. Put the flower back up before driving!
                .stopAndAdd(robot.flowerUp())
                .waitSeconds(.5)


                // Branch based on whether checkHive() saw a tag!
                .stopAndAdd(robot.branchHivePath(failure, success))
                .build();

        while (!isStarted() && !isStopRequested()) {
            LLResult initResult = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> tags = (initResult != null) ? initResult.getFiducialResults() : null;
            boolean seeingHiveNow = (tags != null && !tags.isEmpty());

            telemetry.addData("INIT - Seeing Hive Tag", seeingHiveNow);
            if (seeingHiveNow) {
                telemetry.addData("Detected Hive Tag ID", tags.get(0).getFiducialId());
            }
            telemetry.update();
        }

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        TEST,
                        robot.flywheelUp(),
                        (telemetryPacket) -> {
                            telemetry.addData("Hive Not Tipped (Post-1st Shot)", hiveNotTipped);
                            telemetry.addData("Last Seen Tag ID", lastSeenTagId);
                            telemetry.addData("Distance from goal", hypot);
                            telemetry.addData("Fake distance from goal", riggedHypot);
                            telemetry.addData("angle to goal", angleToGoal);
                            telemetry.addData("heading", robotHeading);
                            telemetry.update();

                            return true;
                        }
                )
        ));

        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}