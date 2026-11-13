package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    DcMotorEx             frontLeft     ;
    DcMotorEx             backLeft      ;
    DcMotorEx             topIntake     ;
    DcMotorEx             middleIntake  ;
    DcMotorEx             bottomIntake  ;
    DcMotorEx             leftFlywheel  ;
    DcMotorEx             rightFlywheel ;
    Servo                 gate          ;
    Servo                 arm           ;
    Servo                 claw          ;
    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1;
    double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

    // --- FTC Dashboard Live Variables ---
    public static double shotKp = 0.0011;
    public static double shotKi = 0.035;
    public static double shotKd = 0;
    public static double shotKf = 0.00029;

    public static double idleKp = 0.0006;
    public static double idleKi = 0;
    public static double idleKd = 0;
    public static double idleKf = 0.000263;




    double shotpid;
    double idlepid;
    double shotPower;
    double idlePower;
    double currentSpeedRight;
    public volatile double rightTarget = 0;




    public PIDFController shotSpinController = new PIDFController(shotKp, shotKi, shotKd, shotKf);
    public PIDFController idleSpinController = new PIDFController(idleKp, idleKi, idleKd, idleKf);

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

        //arm = hardwareMap.get(Servo.class, "arm");

        claw = hardwareMap.get(Servo.class, "claw");



        //topIntake    = hardwareMap.get(DcMotorEx.class,  "topIntake");

        middleIntake   = hardwareMap.get(DcMotorEx.class,  "middleIntake");

        bottomIntake   = hardwareMap.get(DcMotorEx.class,  "bottomIntake");


        rightFlywheel = hardwareMap.get(DcMotorEx.class,  "rightFlywheel");
        leftFlywheel  = hardwareMap.get(DcMotorEx.class,  "leftFlywheel");
        gate = hardwareMap.get(Servo.class,  "gate");

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
        double v = 0;
        double target_y = 0;
        double target_x = 0;

        boolean prevll = false;
        boolean prevrr = false;
        boolean prevaa = false;
        boolean prevyy = false;
        boolean prevbb = false;
        boolean prevxx = false;
        boolean prevsX = false;
        boolean prevsB = false;



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

            /* arm and claw {
                boolean xx = gamepad1.x;
                boolean bb = gamepad1.b;

                if (xx && !prevxx && !bb) {
                    armPos = Range.clip(armPos - 0.01, 0, 1);
                }
                if (bb && !prevbb && !xx) {
                    armPos = Range.clip(armPos + 0.01, 0, 1);
                }

                prevxx = xx;
                prevbb = bb;

                arm.setPosition(armPos);


                boolean sX = gamepad1.dpad_up;
                boolean sB = gamepad1.dpad_down;


                if (sX && !prevsX && !sB) {
                    clawPos = Range.clip(clawPos - 0.01, 0, 1);
                }
                if (sB && !prevsB && !sX) {
                    clawPos = Range.clip(clawPos + 0.01, 0, 1);
                }

                prevsX = sX;
                prevsB = sB;

                claw.setPosition(clawPos);
            }
            */


            // --- Flywheel PIDF Control ---
            {
                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 25, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 25, -7000, 7000.0);
                }

                prevaa = aa;
                prevyy = yy;

                currentSpeedRight = (rightFlywheel.getVelocity() * 60.0) / effectiveTPR;


                shotpid = shotSpinController.calculate(currentSpeedRight, v);
                idlepid = idleSpinController.calculate(currentSpeedRight, v);
                //double pidLeft = idleSpinController.calculate(currentSpeedLeft, leftTarget);

                // 7) Apply power (clip at ±1)
                shotPower = Math.max(-1, Math.min(1, shotpid));
                idlePower = Math.max(-1, Math.min(1, idlepid));
                //double powerLeft = Math.max(-1, Math.min(1, pidLeft));

                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    rightFlywheel.setPower(idlePower);
                    leftFlywheel.setPower (idlePower);
                } else {
                    rightFlywheel.setPower(idlePower);
                    leftFlywheel.setPower (idlePower);
                }
            }

            /* intake stuff */ {


                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = -1.0;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = 1.0;
                }

                //topIntake.setPower(intakeCmd);
                middleIntake.setPower(intakeCmd);
                bottomIntake.setPower(intakeCmd);

                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    gate.setPosition(.5);
                } else {
                    gate.setPosition(.6);
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
            double currentRPM = (rightFlywheel.getVelocity() * 60) / (MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO);

            // --- Telemetry ---
            telemetry.addData("tilt state", currentTiltState);
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos turret", spt);
            telemetry.addData("armPos" , armPos);
            telemetry.addData("clawPos", clawPos);
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