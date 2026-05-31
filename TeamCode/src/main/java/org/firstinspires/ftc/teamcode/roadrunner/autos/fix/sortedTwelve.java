package org.firstinspires.ftc.teamcode.roadrunner.autos.fix;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "sorted twelve", group = "competition")
public class sortedTwelve extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, indexer;
    public Servo hood, turret1, turret2, gate, swingarm, sickle;
    public GoBildaPinpointDriver pip;
    double vTarget;
    double hpos;
    double tx = -72; // Target X
    double ty = 72;  // Target Y
    double t = 1;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double offset = 20;
    public flywheelSub fly; // <-- Added the new subsystem here



    public class Robot {
        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(-1);
                indexer.setPower(-1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class outtake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                indexer.setPower(1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false;
            }
        }
        public Action outtake() { return new outtake(); }

        private class fire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                gate.setPosition(.97);
                indexer.setPower(-1);
                intake.setPower(-1);
                swingarm.setPosition(.95);
                return false;
            }
        }
        public Action fire() { return new fire(); }

        private class stopFire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                indexer.setPower(0);
                sickle.setPosition(.85);
                gate.setPosition(0.65);
                return false;
            }
        }
        public Action stopFire() { return new stopFire(); }

        private class flywheelUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                if (opModeIsActive()){
                    vy                         = pip.getVelY(DistanceUnit.INCH);
                    vx                         = pip.getVelX(DistanceUnit.INCH);

                    tx                         = -72 - (vx * t);
                    ty                         =  72 - (vy * t);

                    pip.update();

                    robotX                     = pip.getPosX(DistanceUnit.INCH);
                    robotY                     = pip.getPosY(DistanceUnit.INCH);

                    xl                         = tx - robotX;
                    yl                         = ty - robotY;
                    hypot                      = Math.sqrt((xl * xl) + (yl * yl));

                    if ( hypot < 89){
                        t                      =  0.004 * hypot + 0.468;
                    } else if ( hypot > 89 && hypot < 121)  {
                        t                      =  0.833;
                    } else if ( hypot > 121) {
                        t                      = -0.004 * hypot + 1.317;
                    }

                    // =========================================================================
                    //  TURRET APPLICATION
                    // =========================================================================
                    double angleToGoal = Math.atan2(yl, xl);
                    double robotHeading = pip.getHeading(AngleUnit.RADIANS);

                    double targetTurretRad = angleToGoal - robotHeading;

                    while (targetTurretRad > Math.PI) targetTurretRad  -= 2 * Math.PI;
                    while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

                    double finalServoDegrees = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);


                    turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
                    turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));





                    fly.runFlywheel();
                    fly.hypot = hypot; // Pass fresh data first
                    fly.loop();        // Calculate power


                    if (hypot < 75) {
                        hpos = -0.005 * hypot + 0.96;
                    } else if(hypot > 75 && hypot < 104){
                        hpos = -0.019 * hypot + 1.968;
                    } else if(hypot > 104){
                        hpos = 0;
                    }

                    // Clip base hood pos
                    hpos = Range.clip(hpos, 0, .7);
                    hood.setPosition(hpos);
                    return true;}
                else return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }

        private class flywheelUpPre implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {

                hood.setPosition(.5);
                turret1.setPosition(.485);
                turret2.setPosition(.485);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel1.setVelocity((double) (2000 * 28) / 60);
                flywheel2.setVelocity((double) (2000 * 28) / 60);
                return false;
            }
        }
        public Action flywheelUpPre() { return new flywheelUpPre(); }


        private class lowerVelocity implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = 5;
                return false;
            }
        }
        public Action lowerVelocity() { return new lowerVelocity(); }
    }
    
    @Override
    public void runOpMode() throws InterruptedException {


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        swingarm = hardwareMap.get(Servo.class, "swingArm");
        sickle = hardwareMap.get(Servo.class, "sickle");
        gate = hardwareMap.get(Servo.class, "gate");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        hood = hardwareMap.get(Servo.class, "hood");
        turret1    = hardwareMap.get(Servo.class, "turret");
        turret2    = hardwareMap.get(Servo.class, "turret2");
        hood.setDirection(Servo.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        fly = new flywheelSub(hardwareMap); // <-- Initialized the new subsystem

        /* ---- S Selection (Toggle) ---- */
        int S = 1; // 1 = Red, -1 = Blue
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) S = 1;  // B for RED
            if (gamepad1.x) S = -1; // X for BLUE

            telemetry.addLine("--- S SELECTION ---");
            telemetry.addData("SELECTED S", S == 1 ? "RED (B)" : "BLUE (X)");
            telemetry.update();
        }

        /* ---- Adaptive Braking Constraint ---- */
        VelConstraint adaptiveBrake = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 90;
            double slowVel = 45;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            } else return cruiseVel;
        };

        VelConstraint adaptiveBrakeSlow = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45;
            double slowVel = 20;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            } else return cruiseVel;
        };

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(-50, 49, Math.toRadians(65));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Pinpoint Initialization
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        double mmPerTick = 0.00197895600191183 * 25.4;
        pip.setOffsets( mmPerTick * -985.8987870798854, mmPerTick * -3119.0758671134577, DistanceUnit.MM);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH,
                initialPose.position.x, initialPose.position.y,
                AngleUnit.RADIANS,
                initialPose.heading.toDouble()
        ));
                Robot robot = new Robot();

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)


                // first volley driving away from zone (first, second and third artifact shot)
                .waitSeconds(.5)
                .afterDisp(20, robot.fire())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-31, 31, Math.toRadians(20)), Math.toRadians(315))

                // intake first spike (fourth, fifth and sixth artifact pickup)
                .afterDisp(0, robot.intake())
                .strafeTo(new Vector2d(-15, 50))
                .stopAndAdd(robot.stopFire())

                // drive to zone for fist spike mark shot (fourth, fifth and sixth artifact shot)
                .strafeTo(new Vector2d(-5, 18))
                .waitSeconds(.5)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                // intake second spike mark (seventh, eighth and ninth artifact pickup)
                .afterDisp(0, robot.intake())
                .strafeTo(new Vector2d(10, 50))
                .stopAndAdd(robot.stopFire())

                // flush (flushes the first to the sixth artifacts)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, 55, Math.toRadians(20)), Math.toRadians(90))

                // back to zone after flush (seventh, eighth and ninth artifact shot)
                .strafeTo(new Vector2d(-10, 18))
                .waitSeconds(.5)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                // intake third spike mark (tenth, eleventh and twelfth artifact pickup)
                .afterDisp(0, robot.intake())
                .strafeTo(new Vector2d(33, 50))
                .stopAndAdd(robot.stopFire())


                // back to zone after third spike mark (tenth, eleventh and twelfth artifact shot)
                .strafeTo(new Vector2d(-10, 18))
                .waitSeconds(.5)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                // first gate pickup (thirteenth, fourteenth and fifteenth artifact pickup)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(5, 55, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(15, 60, Math.toRadians(135)), Math.toRadians(45))
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                // back to zone after first gate pickup (thirteenth, fourteenth and fifteenth artifact pickup)
                .strafeTo(new Vector2d(-15, 18))
                .waitSeconds(.5)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                // second gate pickup (sixteenth, seventeenth and eighteenth artifact pickup)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(5, 55, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(15, 60, Math.toRadians(135)), Math.toRadians(45))
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                // back to zone after second gate pickup and park (sixteenth, seventeenth and eighteenth artifact shot)
                .strafeTo(new Vector2d(-35, 16))
                .waitSeconds(.5)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        TEST,
                        robot.flywheelUp()
                )

        ));
    }
}