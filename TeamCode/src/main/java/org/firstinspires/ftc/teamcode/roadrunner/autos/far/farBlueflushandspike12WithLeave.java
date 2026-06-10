package org.firstinspires.ftc.teamcode.roadrunner.autos.far;

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

import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "far blue flush and spike 12 with leave  ", group = "far")
public class farBlueflushandspike12WithLeave extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, indexer;
    public Servo hood, turret1, turret2, gate, swingarm, sickle;
    public GoBildaPinpointDriver pip;

    double hpos;
    double tx = -72; // Target X
    double ty = -72;  // Target Y
    double t = 0;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double offset = 0;
    public flywheelSub fly;
    private VoltageSensor controlHubVoltageSensor;
    double currentVoltage;


    public class Robot {

        private MecanumDrive driver;

        public Robot(MecanumDrive driver) {
            this.driver = driver;
        }

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
                indexer.setPower(-.7);
                intake.setPower( -.7);
                swingarm.setPosition(.95);
                return false;
            }
        }
        public Action fire() { return new fire(); }

        private class firehard implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                gate.setPosition(.97);
                indexer.setPower(-1);
                intake.setPower( -1);
                swingarm.setPosition(.95);
                return false;
            }
        }
        public Action firehard() { return new firehard(); }

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
                    // 1. Grab the live velocity and position directly from RoadRunner
                    PoseVelocity2d currentVel = driver.updatePoseEstimate();
                    vx = currentVel.linearVel.x;
                    vy = currentVel.linearVel.y;

                    tx =    -72 - (vx * t);
                    ty =  -72 - (vy * t);

                    // 2. Use RoadRunner's offset-adjusted field coordinates
                    robotX = driver.localizer.getPose().position.x;
                    robotY = driver.localizer.getPose().position.y;

                    xl = tx - robotX;
                    yl = ty - robotY;
                    hypot = Math.sqrt((xl * xl) + (yl * yl));

                    if ( hypot < 89){
                        t                      =  0.004 * hypot + 0.468;
                    } else if ( hypot > 89 && hypot < 121)  {
                        t                      =  0.833;
                    } else if ( hypot > 121) {
                        t                      = -0.004 * hypot + 1.317;
                    }
                    currentVoltage = controlHubVoltageSensor.getVoltage();
                    fly.runFlywheel();
                    fly.hypot =   offset + hypot; // Pass fresh data first
                    fly.voltage = currentVoltage;
                    fly.loop();        // Calculate power

                    if (hypot < 75) {
                        hpos = -0.005 * hypot + 0.96;
                    } else if(hypot > 75 && hypot < 100){
                        hpos = -0.014 * hypot + 1.608;
                    } else if(hypot > 100){
                        hpos = 0;
                    }

                    // Clip base hood pos
                    hpos = Range.clip(hpos, 0, .7);
                    hood.setPosition(hpos);
                    return true;
                }
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

        private class raiseVelocity implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = 0;
                return false;
            }
        }
        public Action raiseVelocity() { return new raiseVelocity(); }

        private class lowerVelocity implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = -15;
                return false;
            }
        }
        public Action lowerVelocity() { return new lowerVelocity(); }

        private class turret1 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.8);
                turret2.setPosition(.8);
                return false;
            }
        }
        public Action turret1() { return new turret1(); }

        private class turret2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.79);
                turret2.setPosition(.79);
                return false;
            }
        }
        public Action turret2() { return new turret2(); }

        private class turret3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.6);
                turret2.setPosition(.6);
                return false;
            }
        }
        public Action turret3() { return new turret3(); }

        private class turret4 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.4);
                turret2.setPosition(.4);
                return false;
            }
        }
        public Action turret4() { return new turret4(); }
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
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        fly = new flywheelSub(hardwareMap);


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
            return 10000;
        };

        VelConstraint adaptiveBrakeSlow = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45;
            double slowVel = 20;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel;
            } else return cruiseVel;
        };
        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < 15) {
                return 15;
            } else {
                return 50.0;
            }
        };

        VelConstraint slow_as_hell = (robotPose, _path, _disp) -> 20;


        VelConstraint baseVelConstraint2 = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < 38) {
                return 15;
            } else {
                return 50.0;
            }
        };

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(60, -16.5, Math.toRadians(-90));
        Pose2d intakePose = new Pose2d(60, -60, Math.toRadians(-90));
        Pose2d shotPose = new Pose2d(60, -24, Math.toRadians(-90));


        // MecanumDrive takes control of the Pinpoint here and sets the -53, 53 offset internally
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // We can keep this mapping just in case, but RoadRunner owns the hardware math now!
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        Robot robot = new Robot(drive);

        Action TEST = drive.actionBuilder(initialPose)

                .waitSeconds(1)
                .stopAndAdd(robot.lowerVelocity())
                .stopAndAdd( robot.turret1())
                .waitSeconds(.1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1.5)
                .stopAndAdd(robot.firehard())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())
                

                // pickup
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading( new Pose2d(30, -40, Math.toRadians(-90)), Math.toRadians(180),adaptiveBrake)

                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(30, -60, Math.toRadians(-90)), Math.toRadians(270), adaptiveBrake)

                .afterDisp(20, robot.stopFire())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(shotPose, Math.toRadians(45),adaptiveBrake)
                //.waitSeconds(1)
                .stopAndAdd( robot.turret2())
                .waitSeconds(.1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1.5)
                .stopAndAdd(robot.firehard())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                // pickup
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(intakePose, Math.toRadians(-90), adaptiveBrake)
                .waitSeconds(.5)


                // drive back
                .afterDisp(20, robot.stopFire())
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(shotPose, Math.toRadians(90),adaptiveBrake)
                //.waitSeconds(1)
                .stopAndAdd( robot.turret2())
                .waitSeconds(.1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1.5)
                .stopAndAdd(robot.firehard())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                // pickup
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(intakePose, Math.toRadians(-90),adaptiveBrake)
                .waitSeconds(.5)
                /* VV this picks up from a flush better VV
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(120)), Math.toRadians(90))
                .waitSeconds(.5)*/

                /* VV this keeps artifacts in place better VV
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(45)), Math.toRadians(45)) */

                // drive back
                .afterDisp(20, robot.stopFire())
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(shotPose, Math.toRadians(90),adaptiveBrake)
                //.waitSeconds(1)
                .stopAndAdd( robot.turret2())
                .waitSeconds(.1)
                .stopAndAdd(robot.fire())
                .waitSeconds(1.5)
                .stopAndAdd(robot.firehard())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(60, -50, Math.toRadians(-90)), Math.toRadians(270),adaptiveBrake)


                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(

                // The Parallel action runs your path, background mechanisms, and live telemetry feed together!
                new ParallelAction(
                        TEST,
                        robot.flywheelUp(),
                        (telemetryPacket) -> {

                            // Grab the synchronized pose from RoadRunner
                            Pose2d currentPose = drive.localizer.getPose();

                            // 1. Push perfectly aligned data to the Dashboard
                            telemetryPacket.put("X Pose", currentPose.position.x);
                            telemetryPacket.put("Y Pose", currentPose.position.y);
                            telemetryPacket.put("Heading", Math.toDegrees(currentPose.heading.toDouble()));
                            telemetryPacket.put("Distance from goal", hypot);
                            telemetryPacket.put("ABC velocity offset", offset);

                            // 2. Push aligned data to Driver Station
                            telemetry.addData("X Pose", currentPose.position.x);
                            telemetry.addData("Y Pose", currentPose.position.y);
                            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
                            telemetry.addData("Distance from goal", hypot);
                            telemetry.addData("vel offset", offset);
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