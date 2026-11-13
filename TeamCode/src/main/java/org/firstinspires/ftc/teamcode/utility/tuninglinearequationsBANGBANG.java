package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

@TeleOp(name="the opmode for tuning linear equations with the bangbang", group="linear equations test")
public class tuninglinearequationsBANGBANG extends LinearOpMode {

    // --- Hardware Components (Matching mainTestingTeleop) ---
    Servo turret1, turret2;
    DcMotorEx             frontLeft     ;
    DcMotorEx             backLeft      ;
    DcMotorEx             topIntake     ;
    DcMotorEx             middleIntake  ;
    DcMotorEx             bottomIntake  ;
    DcMotorEx             leftFlywheel  ;
    DcMotorEx             rightFlywheel ;
    Servo                 gate          ;
    Servo                 light         ;
    Servo                 claw          ;

    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1;
    double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

    // --- FTC Dashboard Live Variables (Bang-Bang + FF Tuning) ---
    public static double bangBangTolerance = 1; // RPM buffer
    public static double bangBangTolerance2 = 19; // RPM buffer

    public static double shotKf = 0.00025;          // Feedforward gain


    double flywheelPower;
    double currentSpeedRight;
    public volatile double rightTarget = 0;

    private FtcDashboard dashboard;
    boolean triggerPressed;
    double fireIntakeCmd;

    private ElapsedTime intakeTimer;
    int tiltState   = 0;

    boolean tPressed = false;

    enum TiltState {
        BLUE, BLUE_TILTED, RED, RED_TILTED
    }

    int tiltStateIndex = 0;
    TiltState currentTiltState = TiltState.BLUE;

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        claw = hardwareMap.get(Servo.class, "claw");

        middleIntake   = hardwareMap.get(DcMotorEx.class,  "middleIntake");
        bottomIntake   = hardwareMap.get(DcMotorEx.class,  "bottomIntake");

        rightFlywheel = hardwareMap.get(DcMotorEx.class,  "rightFlywheel");
        leftFlywheel  = hardwareMap.get(DcMotorEx.class,  "leftFlywheel");
        gate = hardwareMap.get(Servo.class,  "gate");
        light = hardwareMap.get(Servo.class,  "light");

        // --- Servo & Motor Configurations ---
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);

        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();

        double spt = 0.5;
        double clawPos = 1;
        double armPos = .5;
        double v = 0; // Target RPM
        double target_y = 0;
        double target_x = 0;

        boolean prevll = false;
        boolean prevrr = false;
        boolean prevaa = false;
        boolean prevyy = false;
        boolean prevoo = false;
        boolean prevxx = false;
        boolean prevsX = false;
        boolean prevsB = false;

        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double mmPerTick = 0.00197895600191183 * 25.4;
        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.resetPosAndIMU();

        intakeTimer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            // --- Turret Control ---
            {
                boolean ll = gamepad1.dpad_left;
                boolean rr = gamepad1.dpad_right;

                if (ll && !prevll && !rr) {
                    spt = Range.clip(spt - 0.01, 0, 1);
                }
                if (rr && !prevrr && !ll) {
                    spt = Range.clip(spt + 0.01, 0, 1);
                }

                prevll = ll;
                prevrr = rr;

                turret1.setPosition(spt);
                turret2.setPosition(spt);
            }

            // --- Flywheel Bang-Bang + FF Control ---
            {
                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;
                boolean oo = gamepad1.options;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 25, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 25, -7000, 7000.0);
                }
                if (oo && !prevoo && !aa && !yy) {
                    v = Range.clip(v + 2975, -7000, 7000.0);
                }


                prevaa = aa;
                prevyy = yy;
                prevoo = oo;

                currentSpeedRight = (rightFlywheel.getVelocity() * 60.0) / effectiveTPR;

                // Calculate Feedforward baseline power based on target RPM
                double feedforwardPower = v * shotKf;

                // Bang-Bang logic with FF baseline
                if (v > 0) {
                    if (currentSpeedRight < v - bangBangTolerance) {
                        flywheelPower = 1.0; // Full power to accelerate quickly
                        light.setPosition(.5);
                    } else {
                        flywheelPower = 0.0;
                        //flywheelPower = Range.clip(feedforwardPower, 0.0, 1.0); // Cruise power
                        light.setPosition(.3);
                    }
                } else {
                    flywheelPower = 0.0;
                }

                rightFlywheel.setPower(flywheelPower);
                leftFlywheel.setPower(flywheelPower);
            }

            /* intake stuff */ {
                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = -1.0;
                } else if ((gamepad1.right_bumper || gamepad2.right_bumper)) {
                    intakeCmd = 1.0;
                }

                middleIntake.setPower(intakeCmd);
                bottomIntake.setPower(intakeCmd);

                if ((gamepad1.right_bumper || gamepad2.right_bumper) && (Math.abs(currentSpeedRight - v) <= bangBangTolerance2)) {
                    gate.setPosition(.5);
                } else {
                    gate.setPosition(.6);
                }
            }

            if (gamepad1.touchpad && !tPressed) {
                tiltStateIndex = (tiltStateIndex + 1) % 4;
                currentTiltState = TiltState.values()[tiltStateIndex];
                tPressed = true;
            } else if (!gamepad1.touchpad) {
                tPressed = false;
            }

            switch (currentTiltState) {
                case BLUE:
                    target_x = 18;
                    target_y = 13;
                    break;
                case BLUE_TILTED:
                    target_x = -18;
                    target_y = 13;
                    break;
                case RED:
                    target_x = 18;
                    target_y = -13;
                    break;
                case RED_TILTED:
                    target_x = -18;
                    target_y = -13;
                    break;
            }

            pip.update();

            double xl = target_x - pip.getPosX(DistanceUnit.INCH);
            double yl = target_y - pip.getPosY(DistanceUnit.INCH);
            double hypot = Math.sqrt((xl * xl) + (yl * yl));
            double currentRPM = (rightFlywheel.getVelocity() * 60) / effectiveTPR;

            // --- Telemetry ---
            telemetry.addData("tilt state", currentTiltState);
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos turret", spt);
            telemetry.addData("armPos" , armPos);
            telemetry.addData("clawPos", clawPos);
            telemetry.addData("target vels", v);
            telemetry.addData("RPM flywheel right", "%.3f", currentRPM);
            telemetry.addData("difference", "%.3f", currentSpeedRight - v);
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            telemetry.update();

            // --- Dashboard Telemetry ---
            dashboard.getTelemetry().addData("hypot", hypot);
            dashboard.getTelemetry().addData("Target RPM", v);
            dashboard.getTelemetry().addData("Actual RPM", currentRPM);
            dashboard.getTelemetry().addData("Flywheel Power", flywheelPower);
            dashboard.getTelemetry().update();
        }
    }
}