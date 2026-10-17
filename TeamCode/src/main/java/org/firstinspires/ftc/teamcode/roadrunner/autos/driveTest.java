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

import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "drive test", group = "competition")
public class driveTest extends LinearOpMode {


    public class Robot {

        private MecanumDrive driver;

        public Robot(MecanumDrive driver) {
            this.driver = driver;
        }





    }

    @Override
    public void runOpMode() throws InterruptedException {





        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(62, -39, Math.toRadians(180));

        // MecanumDrive takes control of the Pinpoint here and sets the -53, 53 offset internally
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Robot robot = new Robot(drive);

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, -52, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(1)

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-45, -52, Math.toRadians(180)), Math.toRadians(180))

                //.waitSeconds(.1)

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52, -24, Math.toRadians(180)), Math.toRadians(90))

                //.waitSeconds(.1)

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52, -57, Math.toRadians(180)), Math.toRadians(270))

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(

                // The Parallel action runs your path, background mechanisms, and live telemetry feed together!
                new ParallelAction(
                        TEST
                )
        ));
        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}