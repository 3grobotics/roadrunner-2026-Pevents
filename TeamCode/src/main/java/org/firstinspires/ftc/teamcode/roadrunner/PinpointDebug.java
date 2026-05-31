package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint Push Test", group = "Debug")
public class PinpointDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Broadcast all telemetry to BOTH the Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Match your physical pod setup
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pip.update();

            telemetry.addLine("--- ROADRUNNER RULES ---");
            telemetry.addLine("1. Push FORWARD -> X MUST increase (Positive)");
            telemetry.addLine("2. Push LEFT -> Y MUST increase (Positive)");
            telemetry.addLine("3. Spin COUNTER-CLOCKWISE -> Heading MUST increase");
            telemetry.addLine("------------------------");

            telemetry.addData("Current X", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("Current Y", pip.getPosY(DistanceUnit.INCH));
            telemetry.addData("Current Heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}