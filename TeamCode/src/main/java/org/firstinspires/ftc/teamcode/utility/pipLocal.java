package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;

@TeleOp(name="pinpoint localization test", group="linear equations test")

public class pipLocal extends LinearOpMode {
    private FtcDashboard dashboard;

    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();
        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = 0.00197895600191183 * 25.4;
        // TODO: Use tunable parameters instead of hardcoded values if possible.
        pip.setOffsets( mmPerTick * -985.8987870798854, mmPerTick * -3119.0758671134577, DistanceUnit.MM);
        pip.setBulkReadScope(defaultRegisters);
        pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pip.update();
            Pose2d pose = new Pose2d(pip.getPosX(DistanceUnit.INCH), pip.getPosY(DistanceUnit.INCH), Math.toRadians(pip.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#D100FF");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            dashboard.getTelemetry().addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            dashboard.getTelemetry().addData("heading", pip.getHeading(AngleUnit.DEGREES));
            dashboard.getTelemetry().addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            dashboard.getTelemetry().update();
            telemetry.update();

        }
    }
}
