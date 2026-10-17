package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

@TeleOp(name="the opmode for tuning linear equations", group="linear equations test")
public class tuninglinearequations extends LinearOpMode {

    // --- Hardware Components (Matching mainTestingTeleop) ---
    Servo turret1, turret2;
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private DcMotorEx rightFlywheel;
    private DcMotorEx gecko; // Used as indexer

    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1;

    // --- FTC Dashboard Live Variables ---
    public double kP = 0.001;
    public double kI = 0.0001;
    public double kD = 0.00001;
    public double kF = 0.000195;



    public PIDFController flywheelController = new PIDFController(kP, kI, kD, kF);
    private FtcDashboard dashboard;
    boolean triggerPressed;
    double fireIntakeCmd;

    private ElapsedTime intakeTimer;
    int tiltState   = 0;

    boolean tPressed = false;

    // Capitalize the enum to prevent naming conflicts
    enum TiltState {
        BLUE, BLUE_TILTED, RED, RED_TILTED
    }

    int tiltStateIndex = 0; // Use this for your math
    TiltState currentTiltState = TiltState.BLUE; // Use this for your switch statement

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");

        // --- Servo & Motor Configurations ---
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);

        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gecko.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();

        double spt = 0.5;
        double v = 0;
        double target_y = 0;
        double target_x = 0;

        boolean prevll = false;
        boolean prevrr = false;
        boolean prevaa = false;
        boolean prevyy = false;



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
        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        //pip.setBulkReadScope(defaultRegisters);
        //pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
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

            // --- Flywheel PIDF Control ---
            {
                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 50, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 50, -7000, 7000.0);
                }

                prevaa = aa;
                prevyy = yy;

                flywheelController.setPIDF(kP, kI, kD, kF);
                double currentRPM = (rightFlywheel.getVelocity() * 60) / 28;
                double pid = flywheelController.calculate(currentRPM, v);
                double power = Math.max(-1, Math.min(1, pid));
                rightFlywheel.setPower(power);
            }

            /* intake stuff */ {


                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = -1;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = 1;
                }

                fireIntakeCmd = ((Range.scale(intakeCmd, -1.0, 1.0, -3000, 3000) * 56) / 60);

                if (!gamepad1.left_bumper && !gamepad2.left_bumper && !gamepad1.right_bumper && !gamepad2.right_bumper) {
                    leftIntake.setVelocity (((Range.scale(intakeCmd, -1.0, 1.0, -3000, 3000) * 56) / 60));
                    rightIntake.setVelocity(((Range.scale(intakeCmd, -1.0, 1.0, -3000, 3000) * 56) / 60));
                } else if (gamepad1.left_bumper || gamepad2.left_bumper || gamepad1.right_bumper || gamepad2.right_bumper) {
                    leftIntake.setVelocity (((/*fireIntakeCmd + iv*/ 900) * 56) / 60);
                    rightIntake.setVelocity(((/*fireIntakeCmd + iv*/ 900) * 56) / 60);
                }



                /*leftIntake.setPower (Range.clip(intakeCmd, -.5912, .5912));
                rightIntake.setPower(Range.clip(intakeCmd, -.5912, .5912));*/

                /*leftIntake .setPower(intakeCmd);
                rightIntake.setPower(intakeCmd);*/




                triggerPressed = gamepad1.right_trigger >= 0.001;


                if (intakeTimer.seconds() < 0.5) {
                    gecko.setPower(-1);
                } else {
                    if (intakeCmd != 0) {
                        gecko.setPower(-(gamepad1.left_bumper || gamepad1.right_bumper ? -1.0 : 1.0));
                    } else {
                        gecko.setPower(0);
                    }

                }

                if (triggerPressed) {
                    intakeTimer.reset();
                }

            }

            if (gamepad1.touchpad && !tPressed) {
                tiltStateIndex = (tiltStateIndex + 1) % 4;
                currentTiltState = TiltState.values()[tiltStateIndex]; // Convert the number back into a named state
                tPressed = true;
            } else if (!gamepad1.touchpad) {
                tPressed = false;
            }

            switch (currentTiltState) {
                case BLUE:
                    target_x = 25;
                    target_y = 14;
                    break;
                case BLUE_TILTED:
                    target_x = -25;
                    target_y = 14;
                    break;
                case RED:
                    target_x = 25;
                    target_y = -14;
                    break;
                case RED_TILTED:
                    target_x = -25;
                    target_y = -14;
                    break;
            }


            pip.update();

            double xl = target_x - pip.getPosX(DistanceUnit.INCH);
            double yl = target_y - pip.getPosY(DistanceUnit.INCH);
            double hypot = Math.sqrt((xl * xl) + (yl * yl));
            double currentRPM = (rightFlywheel.getVelocity() * 60) / 28;

            // --- Telemetry ---
            telemetry.addData("tilt state", currentTiltState);
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos turret", spt);
            telemetry.addData("target vels", v);
            telemetry.addData("RPM flywheel right", "%.3f", currentRPM);
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            telemetry.update();

            // --- Dashboard Telemetry ---
            dashboard.getTelemetry().addData("hypot", hypot);
            dashboard.getTelemetry().addData("Target RPM", v);
            dashboard.getTelemetry().addData("Actual RPM", currentRPM);
            dashboard.getTelemetry().update();
        }
    }
}