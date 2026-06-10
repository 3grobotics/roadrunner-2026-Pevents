package org.firstinspires.ftc.teamcode.roadrunner.autos.fix;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "sorted twelve", group = "competition")
public class sortedTwelve extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private Limelight3A limelight;
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, indexer;
    public Servo hood, turret1, turret2, gate, swingarm, sickle;
    public GoBildaPinpointDriver pip;

    double vTarget;
    double hpos;

    double tx = -72;
    double ty = 72;
    double t = 1;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double offset = 20;
    boolean done1 = false;
    boolean done2 = false;
    boolean done3 = false;
    public flywheelSub fly;

    private VoltageSensor controlHubVoltageSensor;
    double currentVoltage;

    private PredominantColorProcessor frontPredominantColorProcessor;
    private PredominantColorProcessor middlePredominantColorProcessor;
    private PredominantColorProcessor.Result frontResult;
    private PredominantColorProcessor.Result middleResult;

    private IntakeState currentIntakeState = IntakeState.IDLE;
    private final ElapsedTime intakeStateTimer = new ElapsedTime();

    private int activeSortingMotif = 0;

    VisionPortal myVisionPortal;

    private FtcDashboard dashboard;
    private CameraStreamProcessor dashboardCamStream;

    public enum IntakeState {
        IDLE,
        SINGLE_NOTE_CYCLE_INIT,
        SINGLE_NOTE_CYCLE_STEP_1,
        SINGLE_NOTE_CYCLE_STEP_2,
        SINGLE_NOTE_CYCLE_STEP_3,
        SINGLE_NOTE_CYCLE_STEP_4,
        SINGLE_NOTE_CYCLE_STEP_5,
        SINGLE_NOTE_CYCLE_STEP_6,
        SINGLE_NOTE_CYCLE_STEP_7,
        SINGLE_NOTE_CYCLE_STEP_8,
        SINGLE_NOTE_CYCLE_STEP_9
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bitmap);
            lastFrame.set(bitmap);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // Nothing to draw. This processor only copies frames for FTC Dashboard.
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    public class Robot {

        private MecanumDrive driver;

        public Robot(MecanumDrive driver) {
            this.driver = driver;
        }

        private class intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(-1);
                indexer.setPower(-1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false;
            }
        }

        public Action intake() {
            return new intake();
        }

        private class outtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                indexer.setPower(1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false;
            }
        }

        public Action outtake() {
            return new outtake();
        }

        private class fire implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                gate.setPosition(.97);
                indexer.setPower(-1);
                intake.setPower(-1);
                swingarm.setPosition(.95);
                sickle.setPosition(.85);
                return false;
            }
        }

        public Action fire() {
            return new fire();
        }

        private class slowfire implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                gate.setPosition(.97);
                swingarm.setPosition(.95);
                indexer.setPower(-1);
                intake.setPower(-1);
                sickle.setPosition(0.7);
                sleep(500);
                sickle.setPosition(.85);
                sleep(500);
                sickle.setPosition(0.7);
                sleep(500);
                sickle.setPosition(.85);
                sleep(500);
                sickle.setPosition(0.7);
                sleep(500);
                sickle.setPosition(.85);
                sleep(500);
                indexer.setPower(0);
                intake.setPower(0);
                return false;
            }
        }

        public Action slowfire() {
            return new slowfire();
        }

        private class stopFire implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                indexer.setPower(0);
                sickle.setPosition(.85);
                gate.setPosition(0.65);
                return false;
            }
        }

        public Action stopFire() {
            return new stopFire();
        }

        private class flywheelUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                if (opModeIsActive()){
                    // 1. Grab the live velocity and position directly from RoadRunner
                    PoseVelocity2d currentVel = driver.updatePoseEstimate();
                    vx = currentVel.linearVel.x;
                    vy = currentVel.linearVel.y;

                    tx = -72 - (vx * t);
                    ty =  72 - (vy * t);

                    // 2. Use RoadRunner's offset-adjusted field coordinates
                    robotX = driver.localizer.getPose().position.x;
                    robotY = driver.localizer.getPose().position.y;

                    xl = tx - robotX;
                    yl = ty - robotY;
                    hypot = Math.sqrt((xl * xl) + (yl * yl));

                    if ( hypot < 89){
                        t                      =  0.004 * hypot + 0.468;
                    } else if ( hypot > 89 && hypot < 121)  {
                        t                      =  0.833;
                    } else if ( hypot > 121) {
                        t                      = -0.004 * hypot + 1.317;
                    }
                    currentVoltage = controlHubVoltageSensor.getVoltage();
                    fly.runFlywheel();
                    fly.hypot = hypot + offset; // Pass fresh data first
                    fly.voltage = currentVoltage;
                    fly.loop();        // Calculate power

                    if (hypot < 75) {
                        hpos = -0.005 * hypot + 0.96;
                    } else if(hypot > 75 && hypot < 100){
                        hpos = -0.014 * hypot + 1.608;
                    } else if(hypot > 100){
                        hpos = 0;
                    }

                    // Clip base hood pos
                    hpos = Range.clip(hpos, 0, .7);
                    hood.setPosition(hpos);
                    return true;
                }
                else return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }

        private class sorting1 implements Action {
            private boolean initialized = false;
            private final ElapsedTime sorting1Timeout = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (!opModeIsActive()) {
                    return false;
                }

                if (!initialized) {
                    initialized = true;
                    done1 = false;
                    currentIntakeState = IntakeState.IDLE;
                    intakeStateTimer.reset();
                    sorting1Timeout.reset();
                }

                boolean timedOut = sorting1Timeout.seconds() > 4.0;

                if (timedOut) {
                    intake.setPower(0);
                    indexer.setPower(0);
                    gate.setPosition(0.65);
                    swingarm.setPosition(0.65);
                    done1 = true;
                }

                switch (currentIntakeState) {
                    case IDLE:

                        if (activeSortingMotif == 0) {
                            // No Limelight motif was picked in init.
                            // Do not trap auto forever. Just finish and continue.
                            done1 = true;
                            break;
                        }

                        frontResult = frontPredominantColorProcessor.getAnalysis();
                        middleResult = middlePredominantColorProcessor.getAnalysis();

                        boolean match = false;

                        if (activeSortingMotif == 211) {
                            match = frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
                                    && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE;
                        } else if (activeSortingMotif == 121) {
                            match = frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
                                    && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN;
                        } else if (activeSortingMotif == 112) {
                            match = frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN
                                    && middleResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE;
                        }

                        if (match) {
                            activeSortingMotif = 0;
                            done1 = true;
                            currentIntakeState = IntakeState.IDLE;

                            intake.setPower(0);
                            indexer.setPower(0);
                            gate.setPosition(0.65);
                            swingarm.setPosition(0.65);
                        } else {
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                            intakeStateTimer.reset();
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_INIT:
                        sickle.setPosition(0.7);
                        swingarm.setPosition(0.55);
                        gate.setPosition(0.82);
                        intakeStateTimer.reset();
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_1;
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_1:
                        if (intakeStateTimer.milliseconds() > 150) {
                            intake.setPower(-1);
                            indexer.setPower(-1);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_2;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_2:
                        if (intakeStateTimer.milliseconds() > 300) {
                            sickle.setPosition(.75);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_3:
                        if (intakeStateTimer.milliseconds() > 550) {
                            gate.setPosition(0.85);
                            swingarm.setPosition(0.2);
                            intake.setPower(0);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_4:
                        if (intakeStateTimer.milliseconds() > 800) {
                            intake.setPower(0.5);
                            indexer.setPower(-1);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_5:
                        if (intakeStateTimer.milliseconds() > 1000) {
                            indexer.setPower(1);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_6:
                        if (intakeStateTimer.milliseconds() > 1100) {
                            gate.setPosition(0.65);
                            indexer.setPower(0);
                            swingarm.setPosition(0.55);
                            intake.setPower(-1);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_7:
                        if (intakeStateTimer.milliseconds() > 1200) {
                            indexer.setPower(0);
                            intake.setPower(0);
                            swingarm.setPosition(.9);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_8;
                        }
                        break;

                    case SINGLE_NOTE_CYCLE_STEP_8:
                        if (intakeStateTimer.milliseconds() > 1400) {
                            intake.setPower(0);
                            indexer.setPower(0);
                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_9;
                        }
                        break;
                    case SINGLE_NOTE_CYCLE_STEP_9:
                        if (intakeStateTimer.milliseconds() > 1600) {
                            intake.setPower(0);
                            indexer.setPower(0);
                            swingarm.setPosition(.9);
                            currentIntakeState = IntakeState.IDLE;
                        }
                        break;
                }

                p.put("sorting1 done", done1);
                p.put("sorting1 timedOut", timedOut);
                p.put("sorting1 state", currentIntakeState.toString());
                p.put("sorting1 motif", activeSortingMotif);

                return !done1;
            }
        }

        public Action sorting1() {
            return new sorting1();
        }

        private class flywheelUpPre implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                hood.setPosition(.5);
                turret1.setPosition(.485);
                turret2.setPosition(.485);

                flywheel1.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(400, 0, 0, 200)
                );

                flywheel2.setPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(400, 0, 0, 200)
                );

                flywheel1.setVelocity((double) (2000 * 28) / 60);
                flywheel2.setVelocity((double) (2000 * 28) / 60);

                return false;
            }
        }

        public Action flywheelUpPre() {
            return new flywheelUpPre();
        }

        private class raiseVelocity implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                flywheel1.setVelocity((double) (1800 * 37.333) / 60);
                flywheel2.setVelocity((double) (1800 * 37.333) / 60);
                return false;
            }
        }

        public Action raiseVelocity() {
            return new raiseVelocity();
        }

        private class lowerVelocity implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                offset = 5;
                return false;
            }
        }

        public Action lowerVelocity() {
            return new lowerVelocity();
        }

        private class turret1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.27);
                turret2.setPosition(.27);
                return false;
            }
        }

        public Action turret1() {
            return new turret1();
        }

        private class turret2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.13);
                turret2.setPosition(.13);
                return false;
            }
        }

        public Action turret2() {
            return new turret2();
        }

        private class turret3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.30);
                turret2.setPosition(.30);
                return false;
            }
        }

        public Action turret3() {
            return new turret3();
        }

        private class turret4 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                turret1.setPosition(.4);
                turret2.setPosition(.4);
                return false;
            }
        }

        public Action turret4() {
            return new turret4();
        }

        private class fireslow implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                sickle.setPosition(.85);
                gate.setPosition(.97);
                indexer.setPower(-.7);
                intake.setPower( -.7);
                swingarm.setPosition(.95);
                return false;
            }
        }
        public Action fireslow() { return new fireslow(); }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();

        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        frontProcessorBuilder = new PredominantColorProcessor.Builder();
        backProcessorBuilder = new PredominantColorProcessor.Builder();

        frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                80,
                200,
                250,
                400
        ));

        backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                600,
                100,
                800,
                300
        ));

        frontProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
        );

        frontPredominantColorProcessor = frontProcessorBuilder.build();

        backProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
        );

        middlePredominantColorProcessor = backProcessorBuilder.build();

        dashboardCamStream = new CameraStreamProcessor();

        myVisionPortalBuilder = new VisionPortal.Builder();

        myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(middlePredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(dashboardCamStream);

        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        myVisionPortal = myVisionPortalBuilder.build();

        dashboard.startCameraStream(myVisionPortal, 60);

        telemetry.addLine("Dashboard stream: Webcam 1 / VisionPortal");
        telemetry.addLine("Limelight init telemetry: active on Dashboard");
        telemetry.addLine("LL video note: Limelight3A is not a CameraStreamSource in FTC SDK");
        telemetry.update();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        swingarm = hardwareMap.get(Servo.class, "swingArm");
        sickle = hardwareMap.get(Servo.class, "sickle");
        gate = hardwareMap.get(Servo.class, "gate");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        fly = new flywheelSub(hardwareMap);

        VelConstraint adaptiveBrake = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 90;
            double slowVel = 45;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            } else {
                return cruiseVel;
            }
        };

        VelConstraint adaptiveBrakeSlow = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45;
            double slowVel = 20;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel;
            } else {
                return cruiseVel;
            }
        };

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < 15) {
                return 15;
            } else {
                return 50.0;
            }
        };

        VelConstraint slow_as_hell = (robotPose, _path, _disp) -> 20;

        VelConstraint baseVelConstraint2 = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < 38) {
                return 15;
            } else {
                return 50.0;
            }
        };

        // heading (deg): 66.58419541053242
        //x: -48.08085229777625
        //y: 51.98831288607279
        Pose2d initialPose = new Pose2d(-48.08085229777625, 51.98831288607279, Math.toRadians(66.58419541053242));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Robot robot = new Robot(drive);

        Action TEST = drive.actionBuilder(initialPose)

                // first volley driving away from zone (first, second and third artifact shot)
                .waitSeconds(1)
                .afterDisp(0, robot.turret1())
                //.afterDisp(0, robot.raiseVelocity())


                .setTangent(Math.toRadians(280))
                .splineToSplineHeading(new Pose2d(-25, 25, Math.toRadians(37)), Math.toRadians(315))
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())



                // intake first spike (fourth, fifth and sixth artifact pickup)
                .afterDisp(0, robot.lowerVelocity())
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(9, 30, Math.toRadians(90)), Math.toRadians(90))
                //.waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, 60, Math.toRadians(90)), Math.toRadians(90), slow_as_hell)



                // drive to zone for first spike mark shot (fourth, fifth and sixth artifact shot)
                .afterDisp(5, robot.turret2())
                //.afterDisp(0, robot.raiseVelocity())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(5, 36, Math.toRadians(90)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                .stopAndAdd(robot.sorting1())
                .stopAndAdd( robot.fireslow())
                .waitSeconds(1)

                // intake second spike mark (seventh, eighth and ninth artifact pickup)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(90)), Math.toRadians(90))

                // back to zone after flush (seventh, eighth and ninth artifact shot)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                .stopAndAdd(robot.sorting1())
                .stopAndAdd( robot.slowfire())

              /*  // intake second spike mark (seventh, eighth and ninth artifact pickup)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(27, 18, Math.toRadians(90)), Math.toRadians(0))
                .afterDisp(0, robot.intake())

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(34, 48, Math.toRadians(45)), Math.toRadians(30))

                // back to zone after flush (seventh, eighth and ninth artifact shot)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(90)), Math.toRadians(225))
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .stopAndAdd(robot.stopFire())*/

                .build();

        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();

            boolean llValid = result != null && result.isValid();

            int tagCount = 0;
            int tagId = -1;
            double targetTx = Double.NaN;
            double targetTy = Double.NaN;
            double targetArea = Double.NaN;

            String tagMessage = "No valid Limelight result yet";
            List<Integer> seenIds = new ArrayList<>();

            if (result != null) {
                targetTx = result.getTx();
                targetTy = result.getTy();
                targetArea = result.getTa();
            }

            if (llValid) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                tagCount = fiducials.size();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    seenIds.add(fiducial.getFiducialId());
                }

                if (!fiducials.isEmpty()) {
                    tagId = seenIds.get(0);
                    targetTx = fiducials.get(0).getTargetXDegrees();

                    if (tagId == 21) {
                        tagMessage = "Limelight sees: Tag 21 (gpp)";
                        activeSortingMotif = 211;
                    } else if (tagId == 22) {
                        tagMessage = "Limelight sees: Tag 22 (pgp)";
                        activeSortingMotif = 121;
                    } else if (tagId == 23) {
                        tagMessage = "Limelight sees: Tag 23 (ppg)";
                        activeSortingMotif = 112;
                    } else {
                        tagMessage = "Limelight sees unknown Tag: " + tagId;
                    }
                } else {
                    tagMessage = "Valid LL result, but no tags found. Centering turret to 151.5...";
                }
            }

            telemetry.addLine("===== INIT LIMELIGHT =====");
            telemetry.addData("LL valid", llValid);
            telemetry.addData("LL connected", limelight.isConnected());
            telemetry.addData("LL running", limelight.isRunning());
            telemetry.addData("LL ms since update", limelight.getTimeSinceLastUpdate());
            telemetry.addData("LL tag count", tagCount);
            telemetry.addData("LL seen IDs", seenIds);
            telemetry.addData("LL first tag", tagId);
            telemetry.addData("LL tx", targetTx);
            telemetry.addData("LL ty", targetTy);
            telemetry.addData("LL area", targetArea);
            telemetry.addData("activeSortingMotif", activeSortingMotif);
            telemetry.addLine(tagMessage);
            telemetry.update();

            TelemetryPacket initPacket = new TelemetryPacket();
            initPacket.put("INIT LL valid", llValid ? 1 : 0);
            initPacket.put("INIT LL connected", limelight.isConnected() ? 1 : 0);
            initPacket.put("INIT LL running", limelight.isRunning() ? 1 : 0);
            initPacket.put("INIT LL ms since update", limelight.getTimeSinceLastUpdate());
            initPacket.put("INIT LL tag count", tagCount);
            initPacket.put("INIT LL first tag", tagId);
            initPacket.put("INIT LL tx", targetTx);
            initPacket.put("INIT LL ty", targetTy);
            initPacket.put("INIT LL area", targetArea);
            initPacket.put("INIT activeSortingMotif", activeSortingMotif);

            dashboard.sendTelemetryPacket(initPacket);

            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        TEST,
                        robot.flywheelUp(),
                        telemetryPacket -> {

                            Pose2d currentPose = drive.localizer.getPose();

                            telemetryPacket.put("X Pose", currentPose.position.x);
                            telemetryPacket.put("Y Pose", currentPose.position.y);
                            telemetryPacket.put("Heading", Math.toDegrees(currentPose.heading.toDouble()));
                            telemetryPacket.put("Distance from goal", hypot);
                            telemetryPacket.put("ABC velocity offset", offset);
                            telemetryPacket.put("activeSortingMotif", activeSortingMotif);
                            telemetryPacket.put("current intake", currentIntakeState.toString());
                            telemetry.addData("intake", currentIntakeState.toString());

                            telemetry.addData("X Pose", currentPose.position.x);
                            telemetry.addData("Y Pose", currentPose.position.y);
                            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
                            telemetry.addData("Distance from goal", hypot);
                            telemetry.addData("vel offset", offset);
                            telemetry.addData("activeSortingMotif", activeSortingMotif);
                            telemetry.update();

                            return true;
                        }
                )
        ));
        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}