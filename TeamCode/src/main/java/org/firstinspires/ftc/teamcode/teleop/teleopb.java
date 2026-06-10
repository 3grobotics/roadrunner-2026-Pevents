/*
* ''
           .:kNK,
         ,dKWMMWd
       .lXMMMMMMK,
        .,lkKWMMWd.
            .;oONK,
                ':,...           ..,c,
                 .l0X0Oo. ..,:ldOKNWMd
                 '0MMMMX:.cONMMMMMMMWc
                  ,oxdo:.  .;oONMMMMX;
                .l;            'cd0N0'
             .:xXWc               .,;
           ,oKWMMX;
        .cONMMMMM0'
         ,lkXWMMMk.
            .:d0Wd
               .,.
*
* */



package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "TELEOP BLUE COWTOWN  (asuka yay)", group = "Teleop")
public class teleopb extends LinearOpMode {
    public static long webcamExposureMs = 15;
    public static int webcamGain = 250;

    private final ElapsedTime timer = new ElapsedTime();
    private Timer endgameTimer;

    public hardwareSubNewBot h;
    public varSub v;
    public flywheelSub fly;

    // Limelight setup
    private Limelight3A limelight;

    // PTO State Machine
    private boolean ptoButtonWasPressed = false;
    private boolean ptoIsEngaged = false;
    private final ElapsedTime ptoDeploymentTimer = new ElapsedTime();
    private PtoDeploymentState currentPtoDeploymentState = PtoDeploymentState.RETRACTED;

    enum PtoDeploymentState {
        RETRACTED, DEPLOYING_R_SERVO, WAITING_FOR_DEPLOY_DELAY, DEPLOYING_L_SERVO,
        ENGAGED, RETRACTING_L_SERVO, WAITING_FOR_RETRACT_DELAY, RETRACTING_R_SERVO
    }

    private static final double PTO_R_RETRACTED_POSITION = 1.0;
    private static final double PTO_L_RETRACTED_POSITION = 1.0;
    private static final double PTO_R_ENGAGED_POSITION = 0.25;
    private static final double PTO_L_ENGAGED_POSITION = 0.22;
    private static final long PTO_DEPLOYMENT_DELAY_MS = 500;

    // Intake State Machine
    private IntakeState currentIntakeState = IntakeState.IDLE;
    private final ElapsedTime intakeStateTimer = new ElapsedTime();
    private int cycleSubStep = 0;

    // Tracks active sorting routine without blocking the main loop
    private int activeSortingMotif = 0;

    enum IntakeState {
        IDLE, MANUAL_TRIGGERS_ACTIVE, GATE_MANUAL_CONTROL,
        SINGLE_NOTE_CYCLE_INIT, SINGLE_NOTE_CYCLE_STEP_1, SINGLE_NOTE_CYCLE_STEP_2,
        SINGLE_NOTE_CYCLE_STEP_3, SINGLE_NOTE_CYCLE_STEP_4, SINGLE_NOTE_CYCLE_STEP_5,
        SINGLE_NOTE_CYCLE_STEP_6, SINGLE_NOTE_CYCLE_STEP_7, SINGLE_NOTE_CYCLE_STEP_8,
        FIRING, FIRING_COMPLETE, INTAKE_TILL_FULL_INIT, INTAKE_TILL_FULL_RUNNING,
        INTAKE_TILL_PPG, INTAKE_TILL_PGP, INTAKE_TILL_GPP
    }

    // Input debouncing
    private boolean dpadLeftWasPressed = false;
    private boolean psWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean leftStickButtonWasPressed = false;
    private boolean rightStickButtonWasPressed = false;

    double hpos, hoodangle, servoAngle, servopos;
    public static double samOffset;

    // Target pos
    double tx = -72;
    double ty = -72;
    double t = 1;

    VisionPortal myVisionPortal;
    boolean all = false;
    double samOffsetv = 0;
    private FtcDashboard dashboard;
    // Inside your OpMode class definition
    private VoltageSensor controlHubVoltageSensor;

    double currentVoltage;
    double intakeCmd = 0;
    @Override
    public void runOpMode() {
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();

        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        PredominantColorProcessor frontPredominantColorProcessor;
        PredominantColorProcessor middlePredominantColorProcessor;
        PredominantColorProcessor.Result frontResult;
        PredominantColorProcessor.Result middleResult;

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
        middlePredominantColorProcessor = backProcessorBuilder.build();

        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(middlePredominantColorProcessor);

        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Build the VisionPortal so it actually starts the stream
        myVisionPortal = myVisionPortalBuilder.build();
        // VisionPortal natively streams to FTC Dashboard in SDK 9.0+. You can view it by selecting "Webcam 1" from the camera dropdown in the Dashboard UI.

        dashboard = FtcDashboard.getInstance();
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();
        fly = new flywheelSub(hardwareMap);
        endgameTimer = new Timer();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.midDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.frontDistSensor.setMode(DigitalChannel.Mode.INPUT);

        // Initial hardware pos
        h.sickle.setPosition(1.0);
        h.gate.setPosition(0.65);
        h.swingArm.setPosition(1.0);
        h.intake.setPower(0);
        h.indexer.setPower(0);

        // Turret trim debounces
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        boolean prevUp = up;
        boolean prevDown = down;

        float motif = 111;
        double robotX, robotY, xl, yl, hypot;
        double vx, vy, fl, fr, bl, br, max;

        boolean currentDpadLeft, currentPs, currentRightBumper;
        boolean currentRightStickButton, currentLeftStickButton;
        double lift, distanceIn, velocityreal, velDiff;

        // Limelight init loop - Displays tags in telemetry based on a single seen tag
        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    // Grab the very first tag the Limelight sees
                    int tagId = fiducials.get(0).getFiducialId();
                    double targetTx = fiducials.get(0).getTargetXDegrees();

                    if (tagId == 21) {
                        telemetry.addLine("Limelight sees: Tag 21 (gpp)");
                        // Targeting math for 21 goes here if needed
                    } else if (tagId == 22) {
                        telemetry.addLine("Limelight sees: Tag 22 (pgp)");
                        // Targeting math for 22 goes here if needed
                    } else if (tagId == 23) {
                        telemetry.addLine("Limelight sees: Tag 23 (ppg)");
                        // Targeting math for 23 goes here if needed
                    } else {
                        telemetry.addData("Limelight sees unknown Tag: ", tagId);
                    }
                } else {
                    telemetry.addLine("No tags found. Centering turret to 151.5...");
                    // Add your logic to move the turret back to 151.5 here
                }
                telemetry.update();
            }
        }
        h.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
        Pose2d savedPose = PoseStorage.currentPose;
        if (savedPose.position.x == 0 && savedPose.position.y == 0 && savedPose.heading.toDouble() == 0) {
            // We didn't run Auto (or it crashed), so we assume it is the start of the match
            // We might want to change this to the known start pose if testing teleop alone
            // odo.resetPosAndIMU();
            h.pip.resetPosAndIMU();
        } else {
            // We came from Auto so load the data.
            h.pip.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    savedPose.position.x,
                    savedPose.position.y,
                    AngleUnit.RADIANS,
                    savedPose.heading.toDouble()
            ));

        }
        // ^ idk if this does anything honestly ^
        waitForStart();

        while (opModeIsActive()) {
            // Live Webcam Tuning
            if (myVisionPortal != null && myVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl != null && exposureControl.isExposureSupported()) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(webcamExposureMs, TimeUnit.MILLISECONDS);
                }
                GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
                if (gainControl != null) {
                    gainControl.setGain(webcamGain);
                }
            }

            currentVoltage = controlHubVoltageSensor.getVoltage();
            // Poll inputs
            currentDpadLeft         = gamepad1.dpad_left;
            currentPs               = gamepad1.ps;
            currentRightBumper      = gamepad1.right_bumper;
            currentRightStickButton = gamepad1.right_stick_button;
            currentLeftStickButton  = gamepad1.left_stick_button;

            // Read intake from both gamepads, but gamepad1 wins if both are being used
            double gamepad1IntakeCmd = gamepad1.right_trigger - gamepad1.left_trigger;
            double gamepad2IntakeCmd = gamepad2.right_trigger - gamepad2.left_trigger;

            if (Math.abs(gamepad1IntakeCmd) > 0.08) {
                intakeCmd = gamepad1IntakeCmd;
            } else {
                intakeCmd = gamepad2IntakeCmd;
            }

            if (Math.abs(intakeCmd) < 0.08) {
                intakeCmd = 0;
            }

            // Manual override cancels any active auto-sorting routines
            if (currentDpadLeft || currentPs || currentRightBumper || Math.abs(intakeCmd) > 0.1 || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                activeSortingMotif = 0;
            }

            // --- GLOBAL OVERRIDE: GAMEPAD 1 ALWAYS WINS ---
            // Force Firing
            if (gamepad1.right_bumper) {
                currentIntakeState = IntakeState.FIRING;
            }
            // Force Manual Intake
            else if (Math.abs(intakeCmd) > 0.1) {
                currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                activeSortingMotif = 0; // Stop sorting
            }
            // ----------------------------------------------


            // Intake State evaluation
            // Intake State evaluation
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
                } else if (Math.abs(intakeCmd) > 0.1) {
                    currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                } else if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                    currentIntakeState = IntakeState.GATE_MANUAL_CONTROL;
                } else if (activeSortingMotif == 0) { // Only go IDLE if NOT sorting
                    currentIntakeState = IntakeState.IDLE;
                }

            }

            // Update debounce
            dpadLeftWasPressed = currentDpadLeft;
            psWasPressed = currentPs;
            rightBumperWasPressed = currentRightBumper;
            rightStickButtonWasPressed = currentRightStickButton;
            leftStickButtonWasPressed = currentLeftStickButton;



            // Execute current intake state
            switch (currentIntakeState) {
                case IDLE:
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.sickle.setPosition(.85);
                    h.gate.setPosition(0.65);

                    if (activeSortingMotif != 0) {
                        h.swingArm.setPosition(0.9); // Lift to see

                        frontResult = frontPredominantColorProcessor.getAnalysis();
                        middleResult = middlePredominantColorProcessor.getAnalysis();

                        // 1. Check colors
                        boolean match = false;
                        if (activeSortingMotif == 211) {
                            match = (frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                        } else if (activeSortingMotif == 121) {
                            match = (frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN);
                        } else if (activeSortingMotif == 112) {
                            match = (frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                        }

                        // 2. Only re-trigger if we HAVEN'T matched yet AND we aren't currently cycling
                        if (match) {
                            activeSortingMotif = 0; // Success: Stop checking
                            h.swingArm.setPosition(0.65);
                            currentIntakeState = IntakeState.IDLE;
                        } else if (currentIntakeState == IntakeState.IDLE) {
                            // Only trigger a new cycle if the camera failed to match
                            // AND we aren't already running a cycle
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                            intakeStateTimer.reset();
                        }
                    } else {
                        h.swingArm.setPosition(0.65); // Default
                    }
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    // Use the class-level intakeCmd calculated in the polling loop
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.65);
                    h.gate.setPosition(.65);

                    // If triggers are released, return to IDLE
                    if (Math.abs(intakeCmd) < 0.1) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case GATE_MANUAL_CONTROL:
                    if (gamepad1.dpad_right) h.gate.setPosition(0.8);
                    else if (gamepad1.dpad_down) h.gate.setPosition(0.65);
                    else if (gamepad1.dpad_up) h.gate.setPosition(1);
                    else currentIntakeState = IntakeState.IDLE;
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
                        h.indexer.setPower(0);
                        h.intake.setPower(0);
                        h.swingArm.setPosition(.9);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_8;
                    }
                    break;

                case SINGLE_NOTE_CYCLE_STEP_8:
                    if (intakeStateTimer.milliseconds() > 1400) {
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case FIRING:
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                    // This is the only way to exit firing
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
                    if (intakeStateTimer.milliseconds() > 200) currentIntakeState = IntakeState.IDLE;
                    break;
            }

            // PTO Control
            if (gamepad1.left_bumper && !ptoButtonWasPressed) {
                ptoIsEngaged = !ptoIsEngaged;
                ptoButtonWasPressed = true;
                currentPtoDeploymentState = ptoIsEngaged ? PtoDeploymentState.DEPLOYING_R_SERVO : PtoDeploymentState.RETRACTING_L_SERVO;
                ptoDeploymentTimer.reset();
            } else if (!gamepad1.left_bumper) {
                ptoButtonWasPressed = false;
            }

            if(gamepad1.ps){
                h.nautR.setPosition(.1);
                h.nautL.setPosition(.1);
            } else if (gamepad1.left_bumper){
                h.nautR.setPosition(.5);
                h.nautL.setPosition(.5);
            }

            switch (currentPtoDeploymentState) {
                case RETRACTED:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    h.nautR.setPosition(.1);
                    h.nautL.setPosition(.1);
                    break;
                case DEPLOYING_R_SERVO:
                    h.nautR.setPosition(.5);
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    break;
                case WAITING_FOR_DEPLOY_DELAY:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.nautR.setPosition(.5);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    }
                    break;
                case DEPLOYING_L_SERVO:
                    h.nautL.setPosition(.5);
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

            // Drivetrain kinematics
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

            // Odometry / Target Tracking
            vy = h.pip.getVelY(DistanceUnit.INCH);
            vx = h.pip.getVelX(DistanceUnit.INCH);

            tx = -72 - (vx * t);
            ty = -72 - (vy * t);

            h.pip.update();

            robotX = h.pip.getPosX(DistanceUnit.INCH);
            robotY = h.pip.getPosY(DistanceUnit.INCH);

            xl = tx - robotX;
            yl = ty - robotY;
            hypot = Math.sqrt((xl * xl) + (yl * yl));

            // Predictive lookahead scalar
            if (hypot < 84){
                t = 0.003 * hypot + 0.487;
            } else {
                t = 0.001 * hypot + 0.645;
            }

            // Turret calculation
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = h.pip.getHeading(AngleUnit.RADIANS);

            // Shift center 180 deg for rear-facing servo
            double targetTurretRad = angleToGoal - robotHeading - Math.PI;

            // Standardize angle
            while (targetTurretRad > 2 * Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -2 * Math.PI) targetTurretRad += 2 * Math.PI;

            // Apply physical 190-deg limits
            double limitRad = Math.toRadians(190);
            while (targetTurretRad > limitRad) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -limitRad) targetTurretRad += 2 * Math.PI;

            double baseServoDegrees = Math.toDegrees(targetTurretRad) - (395 / 2.0);

            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            // Turret trims
            if (left && !prevleft && !right) samOffset = Range.clip(samOffset + 2.5, -40, 40);
            if (right && !prevright && !left) samOffset = Range.clip(samOffset - 2.5, -40, 40);
            prevleft = left;
            prevright = right;


            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + samOffset;
            double trueServoPos = Math.abs((finalServoDegrees) / 395);

            if ((gamepad1.ps && !psWasPressed) || gamepad2.ps) {
                if (currentIntakeState != IntakeState.INTAKE_TILL_FULL_INIT &&
                        currentIntakeState != IntakeState.INTAKE_TILL_FULL_RUNNING) {
                    h.turret1.setPosition(0.5);
                    h.turret2.setPosition(0.5);
                }
            } else {
                h.turret1.setPosition(Range.clip(trueServoPos, .1, .9));
                h.turret2.setPosition(Range.clip(trueServoPos, .1, .9));
            }

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -59.54706627552903, -37.79224545936885, AngleUnit.DEGREES, -88.13463592529297));
                samOffset = 0;
                fly.samOffsetV = 400;
            } else if (gamepad1.dpad_down) {
                h.pip.resetPosAndIMU();
                samOffset = 0;
                fly.samOffsetV = 400;
            }

            // Flywheel Update
            fly.hypot = hypot;
            fly.voltage = currentVoltage;
            fly.up = gamepad2.dpad_up;
            fly.down = gamepad2.dpad_down;

            if (gamepad1.x) v.var = 1;
            else if (gamepad1.y) v.var = 0;

            if (v.var == 1) fly.runFlywheel();
            else fly.power0();

            fly.loop();

            // Hood linear regression
            if (hypot < 72) {
                hpos = -0.004 * hypot + .892;
            } else if(hypot > 72 && hypot < 96){
                hpos = -0.013 * hypot + 1.536;
            } else if(hypot > 96){
                hpos = 0;
            }
            hpos = Range.clip(hpos, 0, .7);
            h.hood.setPosition(hpos);

            // --- MOTIF OVERRIDE & LEDS ---
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


            // --- ENDGAME & MACRO TRIGGERS ---


            if(h.topDistSensor.getState() == true && h.midDistSensor.getState() == true && h.frontDistSensor.getState() == true){
                all = true;
            } else {
                all = true;
            }

            // ---------------------------------------------------------
            // MANUAL GAMEPAD SORTING CHECKS ONLY
            // ---------------------------------------------------------
            // this is for manual gpp (X)
            if (gamepad2.x && all == true && activeSortingMotif == 0) {
                activeSortingMotif = 211;
                gamepad2.runLedEffect(motif211Effect);
            }

            // this is for manual pgp (A)
            if (gamepad2.a && all == true && activeSortingMotif == 0) {
                activeSortingMotif = 121;
                gamepad2.runLedEffect(motif121Effect);
            }

            // this is for manual ppg (B)
            if (gamepad2.b && all == true && activeSortingMotif == 0) {
                activeSortingMotif = 112;
                gamepad2.runLedEffect(motif112Effect);
            }

            /* if (gamepad1.touchpad) {
                lift = 1;
            } else {
                lift = 0;
            }
            distanceIn = h.revDist.getDistance(DistanceUnit.INCH);
            if (lift == 1) {
                if (distanceIn < 15){
                    h.frontLeft.setPower(1);
                    h.backLeft.setPower(1);
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

            } */

            Pose2d pose = new Pose2d(h.pip.getPosX(DistanceUnit.INCH), h.pip.getPosY(DistanceUnit.INCH), Math.toRadians(h.pip.getHeading(AngleUnit.DEGREES)));
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#D100FF");
            Drawing.drawRobot(packet.fieldOverlay(), pose);

            double velError1 = ((h.flywheel2.getVelocity() * 60) / 28) - fly.target2;
            double velError2 = ((h.flywheel1.getVelocity() * 60) / 28) - fly.target;

            double velError1f = ((h.flywheel2.getVelocity() * 60) / 37.333) - fly.target2;
            double velError2f = ((h.flywheel1.getVelocity() * 60) / 37.333) - fly.target;



            telemetry.addData("Control Hub Voltage", "%.2f Volts", currentVoltage);
            packet.put("currentVoltage", currentVoltage);
            packet.put("encoder1 tps", h.flywheel1.getVelocity());
            packet.put("encoder2 tps", h.flywheel2.getVelocity());
            packet.put("VEL ERROR1 with motor as telem", velError1);
            packet.put("VEL ERROR2 with motor as telem", velError2);
            packet.put("VEL ERROR1 with flywheel as telem", velError1f);
            packet.put("VEL ERROR2 with flywheel as telem", velError2f);
            telemetry.addData("hypot",hypot);
            telemetry.addData("CURRENT MOTIF MEMORY", motif);
            telemetry.addData("ACTIVE SORTING MOTIF", activeSortingMotif);
            telemetry.addData("ARE SENSORS ALL TRUE?", all);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("hpos", hpos);
            telemetry.addData("RPM Flywheel Average", "%.3f", fly.getRotation(flywheelSub.MeasureUnit.REVOLUTIONS, flywheelSub.TimeScale.MINUTES));
            telemetry.addData("RPM flywheel 1", "%.3f", ((h.flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((h.flywheel2.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 1 motor", "%.3f", ((h.flywheel1.getVelocity() * 60) / 28));
            telemetry.addData("RPM flywheel 2 motor", "%.3f", ((h.flywheel2.getVelocity() * 60) / 28));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", h.pip.getPosY(DistanceUnit.INCH));
            telemetry.addData("intake", currentIntakeState.toString());

            telemetry.addData("finalServoDegrees", finalServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name());

            if (currentIntakeState == IntakeState.INTAKE_TILL_PPG || currentIntakeState == IntakeState.INTAKE_TILL_PGP || currentIntakeState == IntakeState.INTAKE_TILL_GPP) {
                telemetry.addData("Cycle Sub-Step", cycleSubStep);
            }

            telemetry.addData("odometry x", h.pip.getEncoderX());
            telemetry.addData("odometry y", h.pip.getEncoderY());
            telemetry.addData("servo pos", servopos);
            telemetry.addData("servo angle", servoAngle);
            telemetry.addData("Detected Motif", motif);

            telemetry.update();
        }
    }
}