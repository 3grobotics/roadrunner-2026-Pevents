package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="Flywheel Dashboard Tuner BangBang", group="Tuning")
public class FlywheelTuningOpModeBANGBANG extends OpMode {

    // --- Hardware ---
    private DcMotorEx rightFlywheel;
    private DcMotorEx leftFlywheel;
    DcMotorEx             middleIntake  ;
    DcMotorEx             bottomIntake  ;
    Servo gate          ;

    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    //-------------------------------------------------------------------------
    // FTC Dashboard Live Variables (Bang-Bang + FF Tuning)
    //-------------------------------------------------------------------------
    public static double bangBangTolerance = 50.0; // RPM buffer band
    public static double shotKf = 0.0003;          // Feedforward cruise gain

    // Shared Target
    public static double targetRPM = 2000; // Target velocity to test

    // Dashboard instance
    private FtcDashboard dashboard;

    double effectiveTPR;
    double rightRPM;
    double leftRPM;
    double rightPower;
    double leftPower;

    @Override
    public void init() {
        // Map hardware
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");

        middleIntake   = hardwareMap.get(DcMotorEx.class,  "middleIntake");
        bottomIntake   = hardwareMap.get(DcMotorEx.class,  "bottomIntake");
        gate = hardwareMap.get(Servo.class,  "gate");

        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        // Get the FTC Dashboard instance
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized. Open FTC Dashboard to graph Bang-Bang + FF RPM.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1) Calculate true actual RPM independently using effective TPR
        effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

        rightRPM = (rightFlywheel.getVelocity() * 60.0) / effectiveTPR;
        leftRPM  = (leftFlywheel.getVelocity() * 60.0)  / effectiveTPR;

        // 2) Calculate Feedforward baseline cruise power
        double feedforwardPower = targetRPM * shotKf;

        // 3) Independent Bang-Bang + FF calculations based on shared targetRPM
        if (targetRPM > 0) {
            // Right Flywheel Bang-Bang logic
            if (rightRPM < targetRPM - bangBangTolerance) {
                rightPower = 1.0; // Full power to accelerate quickly
            } else {
                rightPower = Range.clip(feedforwardPower, 0.0, 1.0); // Maintain cruise power
            }

            // Left Flywheel Bang-Bang logic
            if (leftRPM < targetRPM - bangBangTolerance) {
                leftPower = 1.0;
            } else {
                leftPower = Range.clip(feedforwardPower, 0.0, 1.0);
            }
        } else {
            rightPower = 0.0;
            leftPower = 0.0;
        }

        rightFlywheel.setPower(rightPower);
        //leftFlywheel.setPower(leftPower);

        // --- Intake & Gate Controls ---
        double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intakeCmd = -1.0;
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intakeCmd = 1.0;
        }

        middleIntake.setPower(intakeCmd);
        bottomIntake.setPower(intakeCmd);

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            gate.setPosition(.5);
        } else {
            gate.setPosition(.6);
        }

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