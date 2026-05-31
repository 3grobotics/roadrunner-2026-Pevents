package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Timer;
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
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;

@TeleOp(name="the opmode for tuning linear equations", group="linear equations test")
//@Disabled
public class tuninglinearequations extends LinearOpMode {
    public hardwareSubNewBot h;
    public varSub var;
    public Timer swingTimer;
    Servo t, t2, hood;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private IntakeState currentIntakeState = IntakeState.IDLE;
    private ElapsedTime intakeStateTimer = new ElapsedTime();
    private int cycleSubStep = 0; // Used for multi-step cycles like single-note and till-color

    enum IntakeState {
        IDLE,
        MANUAL_TRIGGERS_ACTIVE, // User holding triggers for intake/outtake
        GATE_MANUAL_CONTROL,    // User holding dpad for gate position

        SINGLE_NOTE_CYCLE_INIT,     // Triggered by dpad_left
        SINGLE_NOTE_CYCLE_STEP_1,   // Sickle out, swing down, gate open
        SINGLE_NOTE_CYCLE_STEP_2,   // Intake/Indexer on
        SINGLE_NOTE_CYCLE_STEP_3,   // Sickle to 0.9
        SINGLE_NOTE_CYCLE_STEP_4,   // Gate to 0.85, swing to 0.2, intake off
        SINGLE_NOTE_CYCLE_STEP_5,   // Intake 0.5, Indexer -1
        SINGLE_NOTE_CYCLE_STEP_6,   // Indexer 1
        SINGLE_NOTE_CYCLE_STEP_7,   // Gate to 0.65, Indexer off, Swing to 0.55, Intake -1
        SINGLE_NOTE_CYCLE_STEP_8,   // Intake off, cycle complete

        FIRING,                     // Triggered by right_bumper
        FIRING_COMPLETE,            // Short delay after firing

        INTAKE_TILL_FULL_INIT,      // Triggered by gamepad1.ps
        INTAKE_TILL_FULL_RUNNING,   // Actively intaking until sensors are full


        // Specific flags for which color processor to use during INTAKE_TILL_COLOR_RUNNING
        INTAKE_TILL_PPG, // right_stick_button
        INTAKE_TILL_PGP, // left_stick_button
        INTAKE_TILL_GPP  // left_stick_button && right_stick_button
    }

    // Debounce variables for intake controls (gamepad1)
    private boolean dpadLeftWasPressed = false;
    private boolean psWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean leftStickButtonWasPressed = false;
    private boolean rightStickButtonWasPressed = false; // Changed from original to prevent immediate re-trigger


    // --- Hardware Constants ---
    public static double MOTOR_TICKS_PER_REV = 28.0;
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    //-------------------------------------------------------------------------
    // FTC Dashboard Live Variables
    //-------------------------------------------------------------------------
    public double kP = 0.0001;
    public double kI = 0.001;
    public double kD = 0.00001;
    public double kF = 0.00026;

    // FTCLib PIDFController
    public PIDFController flywheelController = new PIDFController(kP, kI, kD, kF);

    // Dashboard instance
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        var = new varSub();
        swingTimer = new Timer();

        hood = hardwareMap.get(Servo.class, "hood");
        t = hardwareMap.get(Servo.class, "turret");
        t2 = hardwareMap.get(Servo.class, "turret2");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        hood.setDirection(Servo.Direction.REVERSE);
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

        double sp = 0.5;
        double spt = 0.5;
        double v = 0;

        boolean prevxx = false;
        boolean prevbb = false;
        boolean prevaa = false;
        boolean prevyy = false;
        boolean prevll = false;
        boolean prevrr = false;

        double target_y = 72;
        double target_x = -72;
        boolean swingHigh = false;

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
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexer = hardwareMap.get(DcMotor.class, "indexer");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo swingArm = hardwareMap.get(Servo.class, "swingArm");


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
        //cc issue? VV
        hood.setDirection(Servo.Direction.REVERSE);
        t.setDirection(Servo.Direction.REVERSE);
        t2.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {


            /*turret*/
            {
                boolean ll = gamepad1.dpad_left;
                boolean rr = gamepad1.dpad_right;

                if (ll && !prevll && !rr) {
                    spt = Range.clip(spt - 0.01, 0, 1);
                }
                if (rr && !prevrr && !ll) {
                    spt = Range.clip(spt + 0.01, 0, 1);
                }

                // update prev AFTER using them
                prevll = ll;
                prevrr = rr;

                t.setPosition(spt);
                t2.setPosition(spt);
            }

            /*hood*/
            {
                boolean xx = gamepad1.x;
                boolean bb = gamepad1.b;

                if (xx && !prevxx && !bb) {
                    sp = Range.clip(sp - 0.1, 0, .7);
                }
                if (bb && !prevbb && !xx) {
                    sp = Range.clip(sp + 0.1, 0, .7);
                }

                // update prev AFTER using them
                prevxx = xx;
                prevbb = bb;

                hood.setPosition(sp);
            }

            /*flywheel*/

                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 50, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 50, -7000, 7000.0);
                }

                // update prev AFTER using them
                prevaa = aa;
                prevyy = yy;

                // Ensure the controller is updated from dashboard values
                flywheelController.setPIDF(kP, kI, kD, kF); // We use manual feedforward logic for kF if preferred, or include it
                // Calculate true actual RPM
                double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;
                double ticksPerSec = (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;
                double currentRPM = (flywheel1.getVelocity() * 60) / 37.333;


                // 2) PID calculation (target & k’s are expected to be updated elsewhere)
                double pid = flywheelController.calculate(currentRPM, v);


                // 4) Apply power (clip at ±1)
                double power = Math.max(-1, Math.min(1, pid));
                flywheel1.setPower(power); // preserve original scaling
                flywheel2.setPower(power); // preserve original scaling



            // Debounce gamepad1 buttons for intake control
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentPs = gamepad1.ps;
            boolean currentRightBumper = gamepad1.right_bumper;
            boolean currentRightStickButton = gamepad1.right_stick_button;
            boolean currentLeftStickButton = gamepad1.left_stick_button;

            // State transition logic from IDLE or by interrupting certain states
            // Prioritize specific multi-button or complex actions over simpler ones.
            // If currently in an active intake state, new button presses might override or be ignored.
            if (currentIntakeState == IntakeState.IDLE || currentIntakeState == IntakeState.MANUAL_TRIGGERS_ACTIVE || currentIntakeState == IntakeState.GATE_MANUAL_CONTROL) {
                if (currentLeftStickButton && currentRightStickButton && !leftStickButtonWasPressed && !rightStickButtonWasPressed) {
                    // Corrected priority: Most specific combination first
                    currentIntakeState = IntakeState.INTAKE_TILL_GPP;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentRightStickButton && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PPG;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentLeftStickButton && !leftStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PGP;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentPs && !psWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_INIT;
                    intakeStateTimer.reset();
                } else if (currentDpadLeft && !dpadLeftWasPressed) {
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                    intakeStateTimer.reset();
                } else if (currentRightBumper && !rightBumperWasPressed) { // Only transition to FIRING on initial press
                    currentIntakeState = IntakeState.FIRING;
                    // No timer reset here; FIRING is sustained as long as button is held
                } else {
                    // Manual controls (triggers or gate dpad) if no other sequence is active
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    if (Math.abs(intakeCmd) > 0.1) {
                        currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                    } else if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                        currentIntakeState = IntakeState.GATE_MANUAL_CONTROL;
                    } else if (currentIntakeState != IntakeState.IDLE) { // Ensure idle if no input, AND not already in an active non-manual state
                        currentIntakeState = IntakeState.IDLE;
                    }
                }
            }
            // Update debounce variables
            dpadLeftWasPressed = currentDpadLeft;
            psWasPressed = currentPs;
            rightBumperWasPressed = currentRightBumper;
            rightStickButtonWasPressed = currentRightStickButton;
            leftStickButtonWasPressed = currentLeftStickButton;


            // Execute current intake state logic
            switch (currentIntakeState) {
                case IDLE:
                    // Default positions when nothing is active
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.sickle.setPosition(1.0);
                    // h.swingArm.setPosition(1.0); // Allow swingArm to retain its last position when idle unless another state sets it
                    h.gate.setPosition(0.65); // Default closed/intake position
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.65); // Move swing arm for manual intake
                    h.gate.setPosition(.65);    // Set gate for manual intake

                    if (Math.abs(intakeCmd) < 0.1) { // Triggers released
                        currentIntakeState = IntakeState.IDLE; // Transition to IDLE, swingArm stays at .6
                    }
                    break;

                case GATE_MANUAL_CONTROL:
                    if (gamepad1.dpad_right) {
                        h.gate.setPosition(0.8);
                    } else if (gamepad1.dpad_down) {
                        h.gate.setPosition(0.65);
                    } else if (gamepad1.dpad_up) {
                        h.gate.setPosition(1);
                    } else { // No dpad for gate pressed
                        currentIntakeState = IntakeState.IDLE;
                    }
                    // For manual gate control, the swing arm should probably stay put or return to a default position
                    // Currently, it will retain its position from prior states. If you want it retracted here, add:
                    // h.swingArm.setPosition(1.0);
                    break;

                case SINGLE_NOTE_CYCLE_INIT:
                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.55);
                    h.gate.setPosition(0.82);
                    intakeStateTimer.reset();
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_1;
                    break;
                case SINGLE_NOTE_CYCLE_STEP_1:
                    if (intakeStateTimer.milliseconds() > 250) {
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_2;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_2:
                    if (intakeStateTimer.milliseconds() > 500) { // Cumulative time
                        h.sickle.setPosition(0.9);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_3:
                    if (intakeStateTimer.milliseconds() > 750) { // Cumulative time
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_4:
                    if (intakeStateTimer.milliseconds() > 1000) { // Cumulative time
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_5:
                    if (intakeStateTimer.milliseconds() > 1200) { // Cumulative time
                        h.indexer.setPower(1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_6:
                    if (intakeStateTimer.milliseconds() > 1300) { // Cumulative time
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.55);
                        h.intake.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_7:
                    if (intakeStateTimer.milliseconds() > 1400) { // Cumulative time
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case FIRING:
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                    // Firing is active as long as the button is held.
                    if (!gamepad1.right_bumper) { // Button released
                        currentIntakeState = IntakeState.FIRING_COMPLETE;
                        intakeStateTimer.reset(); // Start timer for cleanup
                    }
                    break;
                case FIRING_COMPLETE:
                    // Reset to idle after a short delay to ensure components return
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(1.0); // Retract swing arm after firing
                    h.gate.setPosition(0.65); // Return gate to default closed/intake position
                    if (intakeStateTimer.milliseconds() > 200) { // Short delay for components to return
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case INTAKE_TILL_FULL_INIT:
                    h.gate.setPosition(0.65);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    h.sickle.setPosition(1.0); // Ensure sickle is up for general intake
                    h.swingArm.setPosition(0.6); // Lower swing arm for intake
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_RUNNING;
                    break;
                case INTAKE_TILL_FULL_RUNNING:
                    // Stop if ALL sensors detect something (assuming getState() is true for detection)
                    if (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState()) {
                        h.intake.setPower(0);
                        h.indexer.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    } else {
                        // Continue intaking
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                    }




            break;
        }


            pip.update();

            double robotX = -pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = target_x - pip.getPosX(DistanceUnit.INCH);
            double yl = target_y - pip.getPosY(DistanceUnit.INCH);
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // Standard Telemetry
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos", sp);
            telemetry.addData("servo pos turret", spt);
            telemetry.addData("vels", v);
            telemetry.addData("x leg", xl);
            telemetry.addData("y leg", yl);
            telemetry.addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (flywheel1.getVelocity()));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (flywheel2.getVelocity()));
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));

            // Dashboard Telemetry
            dashboard.getTelemetry().addData("x leg", xl);
            dashboard.getTelemetry().addData("y leg", yl);
            dashboard.getTelemetry().addData("hypot", hypot);
            dashboard.getTelemetry().addData("servo pos", sp);
            dashboard.getTelemetry().addData("vels", v);
            dashboard.getTelemetry().addData("Target RPM", v);
            dashboard.getTelemetry().addData("Actual RPM", currentRPM);
            dashboard.getTelemetry().addData("Motor Power", power);
            dashboard.getTelemetry().addData("1 Motor vel", flywheel1.getVelocity());
            dashboard.getTelemetry().addData("2 Motor vel", flywheel2.getVelocity());
            dashboard.getTelemetry().addData("RPM flywheel 1", ((flywheel1.getVelocity() * 60) / 37.333));
            dashboard.getTelemetry().addData("RPM flywheel 2", ((flywheel2.getVelocity() * 60) / 37.333));
            dashboard.getTelemetry().update();

            telemetry.update();
        }
    }


}