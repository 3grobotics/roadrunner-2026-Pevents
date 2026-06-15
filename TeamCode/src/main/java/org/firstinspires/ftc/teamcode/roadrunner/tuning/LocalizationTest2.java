package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// --- NEW IMPORTS FOR AMP READING ---
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.util.List;
// -----------------------------------

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
@TeleOp
public class LocalizationTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = new Pose2d(-49, -53, Math.toRadians(-34.7));

        // --- MAP THE HUBS ONCE AT INITIALIZATION ---
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                TelemetryPacket packet = new TelemetryPacket();

                // --- HUB AMP READING LOGIC ---
                for (LynxModule hub : allHubs) {
                    double hubAmps = hub.getCurrent(CurrentUnit.AMPS);
                    if (hub.isParent()) {
                        telemetry.addData("Control Hub Amps", hubAmps);
                        packet.put("Control Hub Amps", hubAmps);
                    } else {
                        telemetry.addData("Expansion Hub Amps", hubAmps);
                        packet.put("Expansion Hub Amps", hubAmps);
                    }
                }
                // -----------------------------

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                TelemetryPacket packet = new TelemetryPacket();

                // --- HUB AMP READING LOGIC ---
                for (LynxModule hub : allHubs) {
                    double hubAmps = hub.getCurrent(CurrentUnit.AMPS);
                    if (hub.isParent()) {
                        telemetry.addData("Control Hub Amps", hubAmps);
                        packet.put("Control Hub Amps", hubAmps);
                    } else {
                        telemetry.addData("Expansion Hub Amps", hubAmps);
                        packet.put("Expansion Hub Amps", hubAmps);
                    }
                }
                // -----------------------------

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}