/*
* ''
           .:kNK,
        , dKWMMWd
       .lXMMMMMMK,
        .,lkKWMMWd.
            .;oONK,
                ':,...           ..,c,
                 .l0X0Oo. ..,:ldOKNWMd
                 '0MMMMX:.cONMMMMMMMWc
                 , oxdo:.  .;oONMMMMX;
                .l;            'cd0N0'
             .:xXWc               .,;
          , oKWMMX;
        .cONMMMMM0'
        , lkXWMMMk.
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.drivers.Prism.Color;
import org.firstinspires.ftc.teamcode.drivers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.drivers.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.drivers.STM32LedModule;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.drivers.STM32LedModule.Direction;
import org.firstinspires.ftc.teamcode.drivers.STM32LedModule.FadeType;
@Config
@TeleOp(name = "TELEOP RED COWTOWN no sorting (looptimes, yay!)", group = "Teleop")
public class teleopNoSort extends LinearOpMode {
   
    public hardwareSubNewBot h;
    public varSub v;
    public flywheelSub fly;


    public STM32LedModule kirin;
    public Servo led;
    GoBildaPrismDriver prism;

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();



    int intakeState = 0;
    int previousIntakeState = -1;

    ElapsedTime bumperTimer = new ElapsedTime();
    boolean isBumperPressed = false;
    boolean hasFiredThisHold = false;

    ElapsedTime flashTimer = new ElapsedTime();
    boolean isFlashing = false;

    
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

   
    double hpos, hoodangle, servoAngle, servopos;
    public static double samOffset;

    // Target pos
    double tx = -72;
    double ty = 72;
    double t = 1;

  
    double samOffsetv = 0;
    private FtcDashboard dashboard;
    // Inside your OpMode class definition
    private VoltageSensor controlHubVoltageSensor;

    // --- NEW HUB LIST VARIABLE ---
    private List<LynxModule> allHubs;

    double currentVoltage;

    // Input debouncing
    private boolean psWasPressed = false;
    double robotX, robotY, xl, yl, hypot;
    double vx, vy, fl, fr, bl, br, max, angleToGoal, robotHeading, targetTurretRad, limitRad;
    double finalServoDegrees, trueServoPos, baseServoDegrees, velError1, velError2, velError1f, velError2f, hubAmps, intakeCmd, currentIntakePower, currentIndexerPower;
    boolean dPressed = false;
    int twistState = 0;
    @Override
    public void runOpMode() {
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        kirin = hardwareMap.get(STM32LedModule.class, "kirin");
        kirin.assignId('A');
        led = hardwareMap.get(Servo.class, "swingArm");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        solid.setBrightness(50);
        solid.setStartIndex(0);
        solid.setStopIndex(12);

        rainbowSnakes.setNumberOfSnakes(2);
        rainbowSnakes.setSnakeLength(3);
        rainbowSnakes.setSpacingBetween(0);
        rainbowSnakes.setSpeed(.25f);
        rainbowSnakes.setBrightness(1000);

        rainbow.setBrightness(100);
        rainbow.setStartIndex(0);
        rainbow.setStopIndex(255);
        rainbow.setSpeed(.25f);



        // --- MAP THE HUBS HERE ---
        allHubs = hardwareMap.getAll(LynxModule.class);

      

        dashboard = FtcDashboard.getInstance();
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();
        fly = new flywheelSub(hardwareMap);

       

        // Initial hardware pos
        h.sickle.setPosition(1.0);
        h.gate.setPosition(0.65);
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

       


        h.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        h.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbow);

        while (opModeIsActive()) {
         

            currentVoltage = controlHubVoltageSensor.getVoltage();
         

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

            intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                h.gate.setPosition(0.97);
                h.intake.setPower(1.0);
                h.indexer.setPower(1.0);

            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                h.gate.setPosition(0.97);
                h.intake.setPower(-1.0);
                h.indexer.setPower(-1.0);

            } else {
                h.intake.setPower(-intakeCmd);
                h.indexer.setPower(-intakeCmd);

                if (-intakeCmd <= 0) {
                    h.gate.setPosition(0.65);
                } else {
                    h.gate.setPosition(0.97);
                }
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

            tx = -72; // - (vx * t);
            ty = 72 ; // - (vy * t);

            h.pip.update();

            robotX = h.pip.getPosX(DistanceUnit.INCH);
            robotY = h.pip.getPosY(DistanceUnit.INCH);

            xl = tx - robotX;
            yl = ty - robotY;
            hypot = Math.sqrt((xl * xl) + (yl * yl));

            // Predictive lookahead scalar
           // if (hypot < 84){
           //     t = 0.003 * hypot + 0.487;
           // } else {
           //     t = 0.001 * hypot + 0.645;
           // }

            // Turret calculation
            angleToGoal     = Math.atan2(yl, xl);
            robotHeading    = h.pip.getHeading(AngleUnit.RADIANS);
            // Shift center 180 deg for rear-facing servo
             targetTurretRad = angleToGoal - robotHeading - Math.PI;
            // Standardize angle
            while (targetTurretRad > 2 * Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -2 * Math.PI) targetTurretRad += 2 * Math.PI;

            // Apply physical 190-deg limits
            limitRad = Math.toRadians(190);
            while (targetTurretRad > limitRad) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -limitRad) targetTurretRad += 2 * Math.PI;

            baseServoDegrees = Math.toDegrees(targetTurretRad) - (395 / 2.0);

            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            // Turret trims
            if (left && !prevleft && !right) samOffset = Range.clip(samOffset + 2.5, -40, 40);
            if (right && !prevright && !left) samOffset = Range.clip(samOffset - 2.5, -40, 40);
            prevleft = left;
            prevright = right;


            finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + samOffset;
            trueServoPos      = Math.abs((finalServoDegrees) / 395);

            if ((gamepad1.ps && !psWasPressed) || gamepad2.ps) {
                    h.turret1.setPosition(0.5);
                    h.turret2.setPosition(0.5);
            } else {
                h.turret1.setPosition(Range.clip(trueServoPos, .1, .9));
                h.turret2.setPosition(Range.clip(trueServoPos, .1, .9));
            }

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -61.3784, 37.8365, AngleUnit.DEGREES, 90));
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
            hpos = -0.002 * ((h.flywheel2.getVelocity() * 60) / 37.333) + 4.1;
            hpos = Range.clip(hpos, 0, .7);
            h.hood.setPosition(hpos);






            Pose2d pose = new Pose2d(h.pip.getPosX(DistanceUnit.INCH), h.pip.getPosY(DistanceUnit.INCH), Math.toRadians(h.pip.getHeading(AngleUnit.DEGREES)));
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#D100FF");
            Drawing.drawRobot(packet.fieldOverlay(), pose);

            velError1 = ((h.flywheel2.getVelocity() * 60) / 28) - fly.target2;
            velError2 = ((h.flywheel1.getVelocity() * 60) / 28) - fly.target;

            velError1f = ((h.flywheel2.getVelocity() * 60) / 37.333) - fly.target2;
            velError2f = ((h.flywheel1.getVelocity() * 60) / 37.333) - fly.target;

            led.setPosition(Range.clip(0.00045 * velError2f + .457, .28, .72) );

            //prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);

            // --- NEW AMP READING TELEMETRY LOGIC ---
            for (LynxModule hub : allHubs) {
                hubAmps = hub.getCurrent(CurrentUnit.AMPS);
                if (hub.isParent()) {
                    packet.put("Control Hub Amps", hubAmps);
                    telemetry.addData("Control Hub Amps", "%.2f A", hubAmps);
                } else {
                    packet.put("Expansion Hub Amps", hubAmps);
                    telemetry.addData("Expansion Hub Amps", "%.2f A", hubAmps);
                }
            }
            // ---------------------------------------


           /* boolean currentBumper = gamepad1.right_bumper || gamepad2.right_bumper;

            // auto-reset after 2-second flash sequence
            if (isFlashing) {
                if (flashTimer.seconds() > 2.0) {
                    isFlashing = false;
                    intakeState = 0;
                }
            } else {
                // 0.5s hold detection for manual progress
                if (currentBumper && !isBumperPressed) {
                    isBumperPressed = true;
                    hasFiredThisHold = false;
                    bumperTimer.reset();

                } else if (currentBumper && isBumperPressed) {
                    if (bumperTimer.seconds() > 0.5 && !hasFiredThisHold) {
                        if (intakeState < 10) intakeState++;
                        hasFiredThisHold = true; // lock until released
                    }

                } else if (!currentBumper) {
                    isBumperPressed = false;
                }
            }

            // push hardware writes only on state change to prevent I2C bus lockup
            if (intakeState != previousIntakeState) {

                if (intakeState == 0) {
                    // explicitly kill the strobe state and blank the LEDs
                    telemetry.addData("Status", "EMPTY");
                    kirin.turnOff();

                } else if (intakeState < 10) {
                    telemetry.addData("Loaded Element", intakeState + "/10");
                    kirin.addManualProgress(Color.PURPLE, 1, 0.1, 1.0, Direction.RIGHT, 100);

                } else if (intakeState == 10) {
                    telemetry.addData("Status", "MAX CAPACITY - FLASHING!");
                    kirin.setStrobeMode(Color.PURPLE, Color.GOLD, 80, 100);

                    isFlashing = true;
                    flashTimer.reset();
                }

                telemetry.update();
                previousIntakeState = intakeState;
            }*/


            telemetry.addData("Control Hub Voltage", "%.2f Volts", currentVoltage);
            packet.put("currentVoltage", currentVoltage);
            packet.put("encoder1 tps", h.flywheel1.getVelocity());
            packet.put("encoder2 tps", h.flywheel2.getVelocity());
            packet.put("VEL ERROR1 with motor as telem", velError1);
            packet.put("VEL ERROR2 with motor as telem", velError2);
            packet.put("VEL ERROR1 with flywheel as telem", velError1f);
            packet.put("VEL ERROR2 with flywheel as telem", velError2f);
            telemetry.addData("hypot",hypot);
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
            telemetry.addData("finalServoDegrees", finalServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name());
            telemetry.addData("odometry x", h.pip.getEncoderX());
            telemetry.addData("odometry y", h.pip.getEncoderY());
            telemetry.addData("servo pos", servopos);
            telemetry.addData("servo angle", servoAngle);
            telemetry.update();
        }
    }
}