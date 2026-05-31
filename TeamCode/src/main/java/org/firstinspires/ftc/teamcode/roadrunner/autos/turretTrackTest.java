package org.firstinspires.ftc.teamcode.roadrunner.autos;

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
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.teleop;

@Autonomous(name = "turret track test", group = "competition")
public class turretTrackTest extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, gecko;
    public Servo hood, turret1, turret2;
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


    public class Robot {
        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                gecko.setPower(1);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class fire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                gecko.setPower(-1);
                return false;
            }
        }
        public Action fire() { return new fire(); }

        private class stopFire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                gecko.setPower(0);
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


        turret1    = hardwareMap.get(Servo.class, "turret");
        turret2    = hardwareMap.get(Servo.class, "turret2");

        /* ---- S Selection (Toggle) ---- */
        int S = 1; // 1 = Red, -1 = Blue
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) S = 1;  // B for RED
            if (gamepad1.x) S = -1; // X for BLUE

            telemetry.addLine("--- S SELECTION ---");
            telemetry.addData("SELECTED S", S == 1 ? "RED (B)" : "BLUE (X)");
            telemetry.update();
        }



        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Pinpoint Initialization
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        double mmPerTick = 0.00197895600191183 * 25.4;
        // TODO: Use tunable parameters instead of hardcoded values if possible.
        pip.setOffsets( mmPerTick * -985.8987870798854, mmPerTick * -3119.0758671134577, DistanceUnit.MM);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH,
                initialPose.position.x, initialPose.position.y,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                initialPose.heading.toDouble()
        ));

        Robot robot = new Robot();

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        TEST,
                        robot.flywheelUp()
                )

        ));


        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}