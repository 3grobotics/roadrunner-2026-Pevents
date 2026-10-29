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
    private DcMotorEx rightFlywheel;
    //private DcMotorEx leftFlywheel;

    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    //-------------------------------------------------------------------------
    // FTC Dashboard Live Variables
    //-------------------------------------------------------------------------

    // Right Turret PIDF Gains
    public static double rightKp = .00033;
    public static double rightKi = .05;
    public static double rightKd = 0.000;
    public static double rightKf = 0.0003;

    // Left Turret PIDF Gains
    public static double leftKp = .00033;
    public static double leftKi = .05;
    public static double leftKd = 0.000;
    public static double leftKf = 0.0003;

    // Shared Target
    public static double targetRPM = 2000; // Target velocity to test

    // FTCLib PIDFControllers - ONE FOR EACH TURRET
    public PIDFController rightController = new PIDFController(rightKp, rightKi, rightKd, rightKf);
    public PIDFController leftController  = new PIDFController(leftKp, leftKi, leftKd, leftKf);

    // Dashboard instance
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Map hardware
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        //leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");

        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        //leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        // Get the FTC Dashboard instance
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized. Open FTC Dashboard to graph RPM.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1) Ensure the controllers are updated from their specific independent live dashboard values
        rightController.setPIDF(rightKp, rightKi, rightKd, rightKf);
        leftController.setPIDF(leftKp, leftKi, leftKd, leftKf);

        // 2) Calculate true actual RPM independently using effective TPR
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

        double rightRPM = (rightFlywheel.getVelocity() * 60.0) / effectiveTPR;
        //double leftRPM  = (leftFlywheel.getVelocity() * 60.0)  / effectiveTPR;

        // 3) Independent PID calculations based on shared target
        double rightPid = rightController.calculate(rightRPM, targetRPM);
        //double leftPid  = leftController.calculate(leftRPM, targetRPM);

        // 4) Apply power (clip at ±1)
        double rightPower = Math.max(-1, Math.min(1, rightPid));
        //double leftPower  = Math.max(-1, Math.min(1, leftPid));

        rightFlywheel.setPower(rightPower);
        //leftFlywheel.setPower(leftPower);

        // --- Driver Station Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Right RPM", rightRPM);
        //telemetry.addData("Left RPM", leftRPM);
        telemetry.addData("Right Power", rightPower);
        //telemetry.addData("Left Power", leftPower);
        telemetry.update();

        // --- FTC Dashboard Telemetry (Graphing) ---
        dashboard.getTelemetry().addData("Target RPM", targetRPM);
        dashboard.getTelemetry().addData("Right RPM", rightRPM);
        //dashboard.getTelemetry().addData("Left RPM", leftRPM);

        dashboard.getTelemetry().addData("Right Power", rightPower);
        //dashboard.getTelemetry().addData("Left Power", leftPower);

        dashboard.getTelemetry().addData("Raw Right Vel", rightFlywheel.getVelocity());
        //dashboard.getTelemetry().addData("Raw Left Vel", leftFlywheel.getVelocity());
        dashboard.getTelemetry().update();
    }

    @Override
    public void stop() {
        rightFlywheel.setPower(0.0);
        //leftFlywheel.setPower(0.0);
    }
}