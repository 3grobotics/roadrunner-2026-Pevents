package org.firstinspires.ftc.teamcode.roadrunner.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "abuse", group = "competition")
public class abuse extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, gecko;
    public Servo hood, turret1, turret2;
    public GoBildaPinpointDriver pip;

    // Flag to control when tracking stops
    public boolean isTracking = true;

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
                if (opModeIsActive() && isTracking){
                    vy                         = pip.getVelY(DistanceUnit.INCH);
                    vx                         = pip.getVelX(DistanceUnit.INCH);

                    tx                         = -72 - (vx * t);
                    ty                         =  72 - (vy * t);



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


                    turret1.setPosition((finalServoDegrees) / 314.6112145);
                    turret2.setPosition((finalServoDegrees) / 314.6112145);
                    pip.update();
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
        Pose2d initialPose = new Pose2d(62, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Pinpoint Initialization
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        double mmPerTick = 0.00197895600191183 * 25.4;
        pip.setOffsets( mmPerTick * -985.8987870798854, mmPerTick * -3119.0758671134577, DistanceUnit.MM);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.setPosition(new Pose2D(DistanceUnit.INCH, 62, -62, AngleUnit.DEGREES, 90));

        Robot robot = new Robot();

                    // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(135))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(24, 24, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, 24, Math.toRadians(270)), Math.toRadians(180))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(270))

                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(24, 24, Math.toRadians(315)), Math.toRadians(45))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(47, 0, Math.toRadians(225)), Math.toRadians(315))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(23, -23, Math.toRadians(135)), Math.toRadians(225))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(45)), Math.toRadians(135))


                .waitSeconds(.1)


                // Assuming your start pose looks something like this:
                // new Pose2d(0, 0, Math.toRadians(90))

                .setTangent(Math.toRadians(0)) // <--- ADD THIS HERE
                .splineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0, 24, Math.toRadians(270)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(0)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(0))



                .setTangent(Math.toRadians(90)) 				// Leave 0,0 traveling UP

                // --- Revolution 1 ---
                .splineTo(new Vector2d(-0.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -1.5), Math.toRadians(0))
                .splineTo(new Vector2d(2.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 3.0), Math.toRadians(180))

                // --- Revolution 2 ---
                .splineTo(new Vector2d(-3.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -4.5), Math.toRadians(0))
                .splineTo(new Vector2d(5.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 6.0), Math.toRadians(180))

                // --- Revolution 3 ---
                .splineTo(new Vector2d(-6.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -7.5), Math.toRadians(0))
                .splineTo(new Vector2d(8.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 9.0), Math.toRadians(180))

                // --- Revolution 4 ---
                .splineTo(new Vector2d(-9.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -10.5), Math.toRadians(0))
                .splineTo(new Vector2d(11.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 12.0), Math.toRadians(180))

                // --- Revolution 5 ---
                .splineTo(new Vector2d(-12.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -13.5), Math.toRadians(0))
                .splineTo(new Vector2d(14.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 15.0), Math.toRadians(180))

                // --- Revolution 6 ---
                .splineTo(new Vector2d(-15.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -16.5), Math.toRadians(0))
                .splineTo(new Vector2d(17.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 18.0), Math.toRadians(180))

                // --- Revolution 7 ---
                .splineTo(new Vector2d(-18.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -19.5), Math.toRadians(0))
                .splineTo(new Vector2d(20.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 21.0), Math.toRadians(180))

                // --- Revolution 8 ---
                .splineTo(new Vector2d(-21.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -22.5), Math.toRadians(0))
                .splineTo(new Vector2d(23.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 24.0), Math.toRadians(180))

                // --- Revolution 9 ---
                .splineTo(new Vector2d(-24.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -25.5), Math.toRadians(0))
                .splineTo(new Vector2d(26.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 27.0), Math.toRadians(180))

                // --- Revolution 10 (Caps at 30" radius) ---
                .splineTo(new Vector2d(-27.75, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -28.5), Math.toRadians(0))
                .splineTo(new Vector2d(29.25, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 30.0), Math.toRadians(180))

                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(62, -62, Math.toRadians(90)), Math.toRadians(300))

                .build();

        // Quick action to stop the turret tracker
        Action stopTracking = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isTracking = false;
                return false;
            }
        };

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        robot.flywheelUp(),
                        new SequentialAction(
                                TEST,
                                stopTracking
                        )
                )
        );

        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}