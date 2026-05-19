package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = "ColorCamera 2 shoot while move")
public class teleop extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();
    private Timer endgameTimer;
    private boolean lastResult = false;
    private int falseCount = 0;
    private int successStreak = 0;
    private double probabilityOfStreak = 1.0;
    private boolean lastBumperState = false; // For rising-edge detection

    // 1/6 chance is approximately 0.1667
    private final double SUCCESS_CHANCE = 1.0 / 6.0;
    public hardwareSubNewBot h;
    public varSub v;
    public flywheelSub fly; // <-- Added the new subsystem here
    public Timer swingTimer;

    // --- PTO Control Variables (already a good state machine, kept as is) ---
    private boolean ptoButtonWasPressed = false; // For debouncing the gamepad button
    private boolean ptoIsEngaged = false;           // True if PTO is "on", false if "off"
    private final ElapsedTime ptoDeploymentTimer = new ElapsedTime();
    private PtoDeploymentState currentPtoDeploymentState = PtoDeploymentState.RETRACTED;

    enum PtoDeploymentState {
        RETRACTED,
        DEPLOYING_R_SERVO,
        WAITING_FOR_DEPLOY_DELAY,
        DEPLOYING_L_SERVO,
        ENGAGED,
        RETRACTING_L_SERVO,
        WAITING_FOR_RETRACT_DELAY,
        RETRACTING_R_SERVO
    }

    private static final double PTO_R_RETRACTED_POSITION = 1.0;
    private static final double PTO_L_RETRACTED_POSITION = 1.0;
    private static final double PTO_R_ENGAGED_POSITION = 0.25;
    private static final double PTO_L_ENGAGED_POSITION = 0.25;
    private static final long PTO_DEPLOYMENT_DELAY_MS = 500;
    // --- END PTO Control Variables ---


    // --- NEW Intake Control Variables (State Machine for Intake and related components) ---
    private IntakeState currentIntakeState = IntakeState.IDLE;
    private final ElapsedTime intakeStateTimer = new ElapsedTime();
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

    double hpos;
    double hoodangle;
    double servoAngle;
    double servopos;
    public static double samOffset;

    /**
     * This OpMode illustrates how to use a video source (camera) as a color sensor
     */
    double tx = -72; // Target X
    double ty = 72;  // Target Y
    double t = 1;

    VisionPortal myVisionPortal;
    boolean all = false;

    @Override
    public void runOpMode() {

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
            fly = new flywheelSub(hardwareMap); // <-- Initialized the new subsystem
        }

        /* these are the timers */
        {
            endgameTimer = new Timer();
        }


        /* telemetry stuff */
        {
            telemetry.setMsTransmissionInterval(50);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        }

        // Initialize digital sensor mode (should be done once)
        h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.midDistSensor.setMode(DigitalChannel.Mode.INPUT); // Assuming these exist
        h.frontDistSensor.setMode(DigitalChannel.Mode.INPUT); // Assuming these exist

        // Set initial positions for intake-related servos and motors
        h.sickle.setPosition(1.0);     // Default: up/retracted
        h.gate.setPosition(0.65);      // Default: closed/intake pos
        h.swingArm.setPosition(1.0);   // Default: up/retracted
        h.intake.setPower(0);
        h.indexer.setPower(0);


        // Gamepad debounce variables for turret trim (gamepad2)
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        float motif                         = 111;
        double robotX;
        double robotY;
        double xl;
        double yl;
        double hypot;

        double vx;
        double vy;
        double fl                           ;
        double fr                           ;
        double bl                           ;
        double br                           ;
        double max                          ;

        boolean currentDpadLeft             ;
        boolean currentPs                   ;
        boolean currentRightBumper          ;
        boolean currentRightStickButton     ;
        boolean currentLeftStickButton      ;
        double lift                         ;
        double distanceIn                   ;
        double velocityreal                 ;
        double velDiff                      ;

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            // =========================================================================
            // NEW INTAKE STATE MACHINE LOGIC
            // =========================================================================

            // Debounce gamepad1 buttons for intake control
            currentDpadLeft         = gamepad1.dpad_left;
            currentPs               = gamepad1.ps;
            currentRightBumper      = gamepad1.right_bumper;
            currentRightStickButton = gamepad1.right_stick_button;
            currentLeftStickButton  = gamepad1.left_stick_button;

            if (currentIntakeState == IntakeState.IDLE || currentIntakeState == IntakeState.MANUAL_TRIGGERS_ACTIVE || currentIntakeState == IntakeState.GATE_MANUAL_CONTROL) {
                if (currentLeftStickButton && currentRightStickButton && !leftStickButtonWasPressed && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_GPP;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentRightStickButton && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PPG;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentLeftStickButton && !leftStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PGP;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentPs && !psWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_INIT;
                    intakeStateTimer.reset();
                } else if (currentDpadLeft && !dpadLeftWasPressed) {
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                    intakeStateTimer.reset();
                } else if (currentRightBumper && !rightBumperWasPressed) {
                    currentIntakeState = IntakeState.FIRING;
                } else {
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    if (Math.abs(intakeCmd) > 0.1) {
                        currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                    } else if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                        currentIntakeState = IntakeState.GATE_MANUAL_CONTROL;
                    } else if (currentIntakeState != IntakeState.IDLE) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                }
            }
            dpadLeftWasPressed = currentDpadLeft;
            psWasPressed = currentPs;
            rightBumperWasPressed = currentRightBumper;
            rightStickButtonWasPressed = currentRightStickButton;
            leftStickButtonWasPressed = currentLeftStickButton;


            switch (currentIntakeState) {
                case IDLE:
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.sickle.setPosition(.85);
                    h.gate.setPosition(0.65);
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.65);
                    h.gate.setPosition(.65);

                    if (Math.abs(intakeCmd) < 0.1) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case GATE_MANUAL_CONTROL:
                    if (gamepad1.dpad_right) {
                        h.gate.setPosition(0.8);
                    } else if (gamepad1.dpad_down) {
                        h.gate.setPosition(0.65);
                    } else if (gamepad1.dpad_up) {
                        h.gate.setPosition(1);
                    } else {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case SINGLE_NOTE_CYCLE_INIT:
                    h.sickle.setPosition(0.7);
                    h.swingArm.setPosition(0.55);
                    h.gate.setPosition(0.82);
                    intakeStateTimer.reset();
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_1;
                    break;
                case SINGLE_NOTE_CYCLE_STEP_1:
                    if (intakeStateTimer.milliseconds() > 150) {
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_2;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_2:
                    if (intakeStateTimer.milliseconds() > 300) {
                        h.sickle.setPosition(.75);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_3:
                    if (intakeStateTimer.milliseconds() > 550) {
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_4:
                    if (intakeStateTimer.milliseconds() > 800) {
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_5:
                    if (intakeStateTimer.milliseconds() > 1000) {
                        h.indexer.setPower(1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_6:
                    if (intakeStateTimer.milliseconds() > 1100) {
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.55);
                        h.intake.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_7:
                    if (intakeStateTimer.milliseconds() > 1200) {
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case FIRING:
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                    if (!gamepad1.right_bumper) {
                        currentIntakeState = IntakeState.FIRING_COMPLETE;
                        intakeStateTimer.reset();
                    }
                    break;
                case FIRING_COMPLETE:
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(1.0);
                    h.gate.setPosition(0.65);
                    if (intakeStateTimer.milliseconds() > 200) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;
            }

            // --- PTO Control Logic ---
            if (gamepad1.options && !ptoButtonWasPressed) {
                ptoIsEngaged = !ptoIsEngaged;
                ptoButtonWasPressed = true;

                if (ptoIsEngaged) {
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_R_SERVO;
                    ptoDeploymentTimer.reset();
                } else {
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTING_L_SERVO;
                    ptoDeploymentTimer.reset();
                }
            } else if (!gamepad1.options) {
                ptoButtonWasPressed = false;
            }

            if(gamepad1.ps){
                h.nautR.setPosition(.1);
                h.nautL.setPosition(.1);
            } else if (gamepad1.options){
                h.nautR.setPosition(.6);
                h.nautL.setPosition(.6);
            }

            switch (currentPtoDeploymentState) {
                case RETRACTED:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    h.nautR.setPosition(.1);
                    h.nautL.setPosition(.1);
                    break;
                case DEPLOYING_R_SERVO:
                    h.nautR.setPosition(.6);
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    break;
                case WAITING_FOR_DEPLOY_DELAY:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.nautR.setPosition(.6);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    }
                    break;
                case DEPLOYING_L_SERVO:
                    h.nautL.setPosition(.6);
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.ENGAGED;
                    break;
                case ENGAGED:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    break;
                case RETRACTING_L_SERVO:
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.WAITING_FOR_RETRACT_DELAY;
                    break;
                case WAITING_FOR_RETRACT_DELAY:
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.RETRACTING_R_SERVO;
                    }
                    break;
                case RETRACTING_R_SERVO:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTED;
                    break;
            }


            v.axial = -gamepad1.left_stick_y;
            v.lateral = gamepad1.left_stick_x;
            v.yawCmd = gamepad1.right_stick_x;

            fl = v.axial + v.lateral + v.yawCmd;
            fr = v.axial - v.lateral - v.yawCmd;
            bl = v.axial - v.lateral + v.yawCmd;
            br = v.axial + v.lateral - v.yawCmd;

            max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            h.frontLeft.setPower(fl / max);
            h.frontRight.setPower(fr / max);
            h.backLeft.setPower(bl / max);
            h.backRight.setPower(br / max);

            vy                         = h.pip.getVelY(DistanceUnit.INCH);
            vx                         = h.pip.getVelX(DistanceUnit.INCH);

            tx                         = -72 - (vx * t);
            ty                         =  72 - (vy * t);

            h.pip.update();

            robotX                     = h.pip.getPosX(DistanceUnit.INCH);
            robotY                     = h.pip.getPosY(DistanceUnit.INCH);

            xl                         = tx - robotX;
            yl                         = ty - robotY;
            hypot                      = Math.sqrt((xl * xl) + (yl * yl));

            if ( hypot < 89){
                t                      =  0.004 * hypot + 0.468;
            } else if ( hypot > 89 && hypot < 121)  {
                t                      =  0.833;
            } else if ( hypot > 121) {
                t                      = -0.004 * hypot + 1.317;
            }

            // =========================================================================
            //  TURRET APPLICATION
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = h.pip.getHeading(AngleUnit.RADIANS);

            double targetTurretRad = angleToGoal - robotHeading;

            while (targetTurretRad > Math.PI) targetTurretRad  -= 2 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

            double baseServoDegrees = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);

            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            aa = gamepad2.a;
            bb = gamepad2.b;

            if (left && !prevleft && !right) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (right && !prevright && !left) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevleft = left;
            prevright = right;

            if (aa && !prevaa && !bb) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevaa = aa;
            prevbb = bb;

            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + samOffset;

            if ((gamepad1.ps && !psWasPressed) || gamepad2.ps) {
                if (currentIntakeState != IntakeState.INTAKE_TILL_FULL_INIT &&
                        currentIntakeState != IntakeState.INTAKE_TILL_FULL_RUNNING) {
                    h.turret1.setPosition(0.5);
                    h.turret2.setPosition(0.5);
                }
            } else {
                h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
                h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
            }

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 36, AngleUnit.DEGREES, 180));
                samOffset = 0;
            } else if (gamepad1.dpad_down) {
                h.pip.resetPosAndIMU();
                samOffset = 0;
            }


            // =========================================================================
            //  FLYWHEEL SUBSYSTEM CONTROL
            // =========================================================================

            // 1. Pass the calculated distance into the subsystem
            fly.hypot = hypot;



            /* flywheel turn on/off */

                if (gamepad1.x) {
                    v.var = 1;
                } else if (gamepad1.y) {
                    v.var = 0;
                }

                if (v.var == 1) {
                    fly.runFlywheel();
                } else {
                    fly.power0();
                }

            // 3. Run the PIDF loop and let it manage target RPM and power automatically
            fly.loop();

            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================

            if (hypot < 75) {
                hpos = -0.015 * hypot + 1.654;
            } else if(hypot > 75 && hypot < 96.3){
                hpos = -0.005 * hypot + 0.675;
            } else if(hypot > 96.3 && hypot < 129){
                hpos = -0.015 * hypot + 1.944;
            } else if(hypot > 151){
                hpos = 0;
            }

            // Clip base hood pos
            hpos = Range.clip(hpos, 0, .7);

            // Calculate base hood angle based purely on distance
            hoodangle = 32.857142857142857142857142857143 * hpos + 32;

            // --- USE THE SUBSYSTEM FOR THE HOOD MATH! ---
            // Grab the true velocity directly from the subsystem in degrees per second
            velocityreal = fly.getRotation(flywheelSub.MeasureUnit.DEGREES, flywheelSub.TimeScale.SECONDS);

            // Calculate lag: target RPM to degrees per second minus actual velocity
            velDiff = ((fly.target / 60.0) * 360.0) - velocityreal;

            // This multiplier converts the massive velocity error into a small degree adjustment
            double velCompensationFactor = 0.001; // <-- YOU WILL TUNE THIS NUMBER

            // Apply the adjustment to the base hood angle
            servoAngle = hoodangle + (velDiff * velCompensationFactor);

            // Convert the compensated angle back into a servo position (PEMDAS fixed!)
            servopos = (servoAngle - 32) * 0.03043478260869565217391304347826;

            // Final clip for servo output to protect the physical hardware
            servopos = Range.clip(servopos, 0, .7);
            h.hood.setPosition(servopos);



            if (endgameTimer.getElapsedTimeSeconds() > 90 || gamepad2.options){

                PredominantColorProcessor.Builder frontProcessorBuilder;
                PredominantColorProcessor.Builder backProcessorBuilder;
                VisionPortal.Builder myVisionPortalBuilder;
                PredominantColorProcessor frontPredominantColorProcessor;
                PredominantColorProcessor backPredominantColorProcessor;
                VisionPortal myVisionPortal;
                PredominantColorProcessor.Result frontResult;
                PredominantColorProcessor.Result backResult;


                frontProcessorBuilder = new PredominantColorProcessor.Builder();
                backProcessorBuilder = new PredominantColorProcessor.Builder();

                frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                        80,
                        200,
                        250,
                        400));
                backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                        600,
                        100,
                        800,
                        300));

                frontProcessorBuilder.setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                frontPredominantColorProcessor = frontProcessorBuilder.build();


                backProcessorBuilder.setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                backPredominantColorProcessor = backProcessorBuilder.build();

                myVisionPortalBuilder = new VisionPortal.Builder();
                myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
                myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);

                myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
                myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
                myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


                if (gamepad1.touchpad) {
                    lift = 1;
                } else {
                    lift = 0;
                }
                distanceIn = h.revDist.getDistance(DistanceUnit.INCH);
                if (lift == 1) {
                    if (distanceIn < 15){
                        h.frontLeft.setPower(1);
                        h. backLeft.setPower(1);
                        h.frontRight.setPower(-1);
                        h.backRight.setPower( -1);
                    }
                    if (distanceIn < 16 && distanceIn > 15) {
                        h.frontLeft.setPower(.5);
                        h.backLeft.setPower(.5);
                        h.frontRight.setPower(-.5);
                        h.backRight.setPower(-.5);
                    }
                    if (distanceIn >= 16) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_R_SERVO;
                    }

                }

                if(h.topDistSensor.getState() == true && h.midDistSensor.getState() == true && h.frontDistSensor.getState() == true){
                    all = true;
                } else {
                    all = false;
                }

                if (gamepad2.a == true) {
                    while (opModeIsActive() && !isStopRequested()) {
                        frontResult = frontPredominantColorProcessor.getAnalysis();

                        if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                            break;
                        }

                        h.sickle.setPosition(0.85);
                        h.swingArm.setPosition(0.5);
                        h.gate.setPosition(0.82);
                        sleep(250);
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                        sleep(250);
                        h.sickle.setPosition(0.9);
                        sleep(250);
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        sleep(250);
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        sleep(200);
                        h.indexer.setPower(1);
                        sleep(100);
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.5);
                        h.intake.setPower(-1);
                        sleep(100);
                        h.intake.setPower(0);

                        h.swingArm.setPosition(1);
                        sleep(500);

                    }

                }
                Gamepad.LedEffect motif112Effect = new Gamepad.LedEffect.Builder()
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                Gamepad.LedEffect motif121Effect = new Gamepad.LedEffect.Builder()
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                Gamepad.LedEffect motif211Effect = new Gamepad.LedEffect.Builder()
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                if (gamepad2.left_bumper){

                    if (gamepad2.x){
                        motif = 211;
                        gamepad2.runLedEffect(motif211Effect);
                    } else if (gamepad2.a){
                        motif = 121;
                        gamepad2.runLedEffect(motif121Effect);
                    } else if (gamepad2.b){
                        motif = 112;
                        gamepad2.runLedEffect(motif112Effect);
                    }

                }

                if (gamepad2.x && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 121 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 112 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }


                }

                if (gamepad2.a && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 121 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }

                    if (motif == 112 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }
                }

                if (gamepad2.b  && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }

                    if (motif == 121 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 112 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break;
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }
                }
            }


            telemetry.addData("hpos", hpos);
            // Updated telemetry to use the new subsystem's method directly for clean RPM data
            telemetry.addData("RPM Flywheel Average", "%.3f", fly.getRotation(flywheelSub.MeasureUnit.REVOLUTIONS, flywheelSub.TimeScale.MINUTES));
            telemetry.addData("RPM flywheel 1", "%.3f", ((h.flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((h.flywheel2.getVelocity() * 60) / 37.333));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", h.pip.getPosY(DistanceUnit.INCH));

            telemetry.addData("finalServoDegrees", finalServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name());

            if (currentIntakeState == IntakeState.INTAKE_TILL_PPG || currentIntakeState == IntakeState.INTAKE_TILL_PGP || currentIntakeState == IntakeState.INTAKE_TILL_GPP) {
                telemetry.addData("Cycle Sub-Step", cycleSubStep);
            }

            telemetry.addData("odometry x", h.pip.getEncoderX());
            telemetry.addData("odometry y", h.pip.getEncoderY());
            telemetry.addData("servo pos", servopos);
            telemetry.addData("servo angle", servoAngle);

            telemetry.update();
        }
    }
}