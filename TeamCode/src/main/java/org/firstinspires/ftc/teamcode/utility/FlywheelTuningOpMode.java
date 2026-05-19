package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
@TeleOp(name="Flywheel Dashboard Tuner", group="Tuning")
public class FlywheelTuningOpMode extends OpMode {

    // --- Hardware ---
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    //-------------------------------------------------------------------------
    // FTC Dashboard Live Variables
    //-------------------------------------------------------------------------
    public double kP = .00033;
    public double kI = .05;
    public double kD = 0.000;
    public double kF = 0.0003;

    public static double targetRPM = 2000; // Target velocity to test

    // FTCLib PIDFController
    public PIDFController flywheelController = new PIDFController(kP, kI, kD, kF);

    // Dashboard instance
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Map hardware
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);


        // Get the FTC Dashboard instance
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized. Open FTC Dashboard to graph RPM.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Ensure the controller is updated from dashboard values
        flywheelController.setPIDF(kP, kI, kD, kF); // We use manual feedforward logic for kF if preferred, or include it
        // Calculate true actual RPM
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;
        double ticksPerSec = (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;
        double currentRPM = (flywheel1.getVelocity() * 60) / 37.333;


        // 2) PID calculation (target & k’s are expected to be updated elsewhere)
        double pid = flywheelController.calculate(currentRPM, targetRPM);


        // 4) Apply power (clip at ±1)
        double power = Math.max(-1, Math.min(1, pid));
        flywheel1.setPower(power); // preserve original scaling
        flywheel2.setPower(power); // preserve original scaling

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", currentRPM);
        telemetry.addData("Motor Power", power);
        telemetry.update();

        dashboard.getTelemetry().addData("Target RPM", targetRPM);
        dashboard.getTelemetry().addData("Actual RPM", currentRPM);
        dashboard.getTelemetry().addData("Motor Power", power);
        dashboard.getTelemetry().addData("raw Motor Power", power);
        dashboard.getTelemetry().addData("1 Motor vel", flywheel1.getVelocity());
        dashboard.getTelemetry().addData("2 Motor vel", flywheel2.getVelocity());
        dashboard.getTelemetry().addData("RPM flywheel 1", ((flywheel1.getVelocity() * 60) / 37.333));
        dashboard.getTelemetry().addData("RPM flywheel 2", ((flywheel2.getVelocity() * 60) / 37.333));
        dashboard.getTelemetry().update();
    }

    @Override
    public void stop() {
        flywheel1.setPower(0.0);
        flywheel2.setPower(0.0);
    }
}