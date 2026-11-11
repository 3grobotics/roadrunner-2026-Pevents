package org.firstinspires.ftc.teamcode.teleop;

import android.util.Base64;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.BadAppleData;
import org.firstinspires.ftc.teamcode.BadAppleHardcoded;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub.MeasureUnit;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub.TimeScale;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub.FlywheelSide;

import java.util.List;

@Config
@TeleOp(name = "teleop", group = "teleop")
public class mainTestingTeleop extends LinearOpMode {

    DcMotorEx             frontRight    ;
    DcMotorEx             backRight     ;
    DcMotorEx             frontLeft     ;
    DcMotorEx             backLeft      ;
    //DcMotorEx             topIntake     ;
    DcMotorEx             middleIntake  ;
    DcMotorEx             bottomIntake  ;
    DcMotorEx             leftFlywheel  ;
    DcMotorEx             rightFlywheel ;
    ServoImplEx           turret1       ;
    ServoImplEx           turret2       ;
    Servo                 gate          ;
    Servo                 flower        ;
    Servo                 arm           ;
    Servo                 claw          ;
    Servo                 light         ;
    GoBildaPinpointDriver pip           ;



    private BadAppleHardcoded.Renderer display;
    private ElapsedTime frameTimer;
    private ElapsedTime intakeTimer;
    private ElapsedTime trigTimer;
    private ElapsedTime runtime;

    final int VIDEO_WIDTH = 64;
    final int VIDEO_HEIGHT = 64;
    final double FRAME_TIME_MS = 100.0; // 10 FPS

    private int currentFrame = 0;

    double axial;
    double lateral;
    double yawCmd;
    double max;

    double fl;
    double fr;
    double bl;
    double br;
    double vel;
    boolean prevaa;
    boolean prevbb;
    boolean aa;
    boolean bb;

    double var;

    double vTarget = -3550;

    boolean triggerPressed;

    double p;
    public flywheelSub flywheelSub;

    double tx = 18; // Target X
    double ty = 13;  // Target Y
    double tiltedTx = -18; // Target X
    double tiltedTy = 13;  // Target Y

    double samOffset = 0;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;


    double hpos;

    double angleToGoal;
    double robotHeading;
    double calculatedTurretRad;
    double finalServoDegrees;

    double iv;
    boolean prevup;
    boolean prevdown;
    boolean up;
    boolean down;
    double fireIntakeCmd;

    boolean flowerDown;

    int tiltState   = 0;

    boolean tPressed = false;

    private VoltageSensor controlHubVoltageSensor;



    // --- NEW HUB LIST VARIABLE ---
    private List<LynxModule> allHubs;

    double currentVoltage;

    public flywheelSub fly;
    public varSub v;
    boolean bPressed = false;
    int armState = 0;
    boolean aPressed = false;
    int grabState = 0;
    @Override
    public void runOpMode() {

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;




        frontRight = hardwareMap.get(DcMotorEx.class,  "frontRight");
        backRight  = hardwareMap.get(DcMotorEx.class,  "backRight");
        backLeft   = hardwareMap.get(DcMotorEx.class,  "backLeft");
        frontLeft  = hardwareMap.get(DcMotorEx.class,  "frontLeft");

        //topIntake    = hardwareMap.get(DcMotorEx.class,  "topIntake");

        middleIntake   = hardwareMap.get(DcMotorEx.class,  "middleIntake");

        bottomIntake   = hardwareMap.get(DcMotorEx.class,  "bottomIntake");


        rightFlywheel = hardwareMap.get(DcMotorEx.class,  "rightFlywheel");
        //leftFlywheel  = hardwareMap.get(DcMotorEx.class,  "leftFlywheel");
        gate = hardwareMap.get(Servo.class,  "gate");
        flower = hardwareMap.get(Servo.class,  "flower");
        light = hardwareMap.get(Servo.class,  "light");

        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        turret1 = hardwareMap.get(ServoImplEx.class, "turret1");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");

        turret1.setDirection(ServoImplEx.Direction.REVERSE);
        turret2.setDirection(ServoImplEx.Direction.REVERSE);

        turret1.setPwmRange(new PwmControl.PwmRange(550, 2450));
        turret2.setPwmRange(new PwmControl.PwmRange(550, 2450));


        //arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");



        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection (DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection (DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection  (DcMotorSimple.Direction.FORWARD);

        rightFlywheel.setDirection  (DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.setMsTransmissionInterval(80);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        display = new BadAppleHardcoded.Renderer(VIDEO_WIDTH, VIDEO_HEIGHT);
        frameTimer = new ElapsedTime();

        intakeTimer = new ElapsedTime();
        trigTimer = new ElapsedTime();

        runtime = new ElapsedTime();

        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        allHubs = hardwareMap.getAll(LynxModule.class);

        fly = new flywheelSub(hardwareMap);
        v = new varSub();
        waitForStart();

        runtime.reset();


        while (opModeIsActive()) {

            currentVoltage = controlHubVoltageSensor.getVoltage();

            if (gamepad1.dpad_up) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, 62.36844280573327, 9.84, AngleUnit.DEGREES, 180));
            } else if (gamepad1.dpad_down) {
                pip.resetPosAndIMU();
            } else if (gamepad1.dpad_left) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, -9.3, 65.5, AngleUnit.DEGREES, 270));
            }

            robotX = pip.getPosX(DistanceUnit.INCH);
            robotY = pip.getPosY(DistanceUnit.INCH);



            if(robotX > 0){
                xl = tx - robotX;
                yl = ty - robotY;
            } else{
                xl = tiltedTx - robotX;
                yl = tiltedTy - robotY;
            }

            hypot = Math.sqrt((xl * xl) + (yl * yl));

            // Drop this inside your active while loop:
            double progress = 1.0 - Math.abs((runtime.seconds() % 5.0) / 2.5 - 1.0);
            double pos = 0.277 + (0.722 - 0.277) * progress;
            light.setPosition(pos);


            /* Arm*/

                /*if (gamepad1.b && !bPressed) {
                    armState = (armState + 1) % 5;
                    bPressed = true;
                } else if (!gamepad1.b) {
                    bPressed = false;
                }
                switch (armState){
                    case 0:
                        // drive around
                        arm.setPosition(.44);
                        break;
                    case 1:
                        // ready to pick up
                        if (gamepad1.a && !aPressed) {
                            grabState = (grabState + 1) % 2;
                            aPressed = true;
                        } else if (!gamepad1.a) {
                            aPressed = false;
                        }
                        switch (grabState){
                            case 0:
                                arm.setPosition(.58);
                                break;
                            case 1:
                                arm.setPosition(.59);
                                break;
                        }
                        break;
                    case 2:
                        // grab

                        if (arm.getPosition() == .58){
                            claw.setPosition(.75);
                        } else {
                            claw.setPosition(.69);
                        }

                        break;
                    case 3:
                        // extend

                        arm.setPosition(.22);
                        break;
                    case 4:
                        // let go

                        claw.setPosition(.99);

                        break;

                }


            /* turret stuff*/ {
                angleToGoal = Math.atan2(yl, xl);
                robotHeading = pip.getHeading(AngleUnit.RADIANS);

                // target angle
                calculatedTurretRad = angleToGoal - robotHeading;

                while (calculatedTurretRad > Math.PI) {
                    calculatedTurretRad -= 2 * Math.PI;
                }
                while (calculatedTurretRad < -Math.PI) {
                    calculatedTurretRad += 2 * Math.PI;
                }

                finalServoDegrees = Math.toDegrees(calculatedTurretRad) + (388.73743 / 2);

                left = gamepad2.dpad_left;
                right = gamepad2.dpad_right;


                if (left && !prevleft && !right) {
                    samOffset = Range.clip(samOffset + 2.5, -40, 40);
                }
                if (right && !prevright && !left) {
                    samOffset = Range.clip(samOffset - 2.5, -40, 40);
                }

                // leftdate prev AFTER using them
                prevleft = left;
                prevright = right;

                finalServoDegrees += samOffset;

                // 5. Clamp and Set
                finalServoDegrees = Range.clip(finalServoDegrees, 0, 388.73743);
                if (gamepad1.ps || gamepad2.ps) {
                    turret1.setPosition(.5);
                    turret2.setPosition(.5);
                }
                /*else if(true) {
                    turret1.setPosition(.35);
                    turret2.setPosition(.35);
                }*/
                    else {
                    turret1.setPosition(finalServoDegrees / 388.73743);
                    turret2.setPosition(finalServoDegrees / 388.73743);
                }
            }
            



            /* intake stuff */ {


                up = gamepad1.dpad_up;
                down = gamepad1.dpad_down;

                if (up && !prevup && !down) {
                    iv = Range.clip(iv + 50, -6000, 6000);
                }
                if (down && !prevdown && !up) {
                    iv = Range.clip(iv - 50, -6000, 6000);
                }
                //leftFlywheel.setVelocity (((v * 28) / 60));
                //rightFlywheel.setVelocity(((v * 28) / 60));

                prevup = up;
                prevdown = down;

                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = -1.0;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = 1;
                }

                //topIntake.setPower(intakeCmd);
                middleIntake.setPower(intakeCmd);
                bottomIntake.setPower(intakeCmd);

                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    gate.setPosition(.5);
                } else {
                    gate.setPosition(.6);
                }

                flowerDown = atFlower(robotX, robotY, pip.getHeading(AngleUnit.DEGREES));
                flower.setPosition(flowerDown ? 0 : 0.5);



            }

            /*flywheels*/
            // Flywheel Update
            fly.hypot = hypot;
            fly.voltage = currentVoltage;
            fly.up = gamepad2.dpad_up;
            fly.down = gamepad2.dpad_down;
            fly.rightBumper = gamepad1.right_bumper;
            fly.leftBumper = gamepad1.left_bumper;

            if (gamepad1.x) v.var = 1;
            else if (gamepad1.y) v.var = 0;

            if (v.var == 1) fly.runFlywheel();
            else fly.power0();

            fly.loop();



            /* driving inputs */ {
                axial = -gamepad1.left_stick_y;
                lateral = gamepad1.left_stick_x;
                yawCmd = gamepad1.right_stick_x;

                fl = axial + lateral + yawCmd;
                fr = axial - lateral - yawCmd;
                bl = axial - lateral + yawCmd;
                br = axial + lateral - yawCmd;

                max = Math.max(1, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
                frontLeft.setPower ((fl / max) / 1 );
                frontRight.setPower((fr / max) / 1 );
                backLeft.setPower  ((bl / max) / 1 );
                backRight.setPower ((br / max) / 1 );
            }

            /*bad apple*/{
                if (currentFrame >= BadAppleData.FRAMES.length) {
                    currentFrame = 0; // Loop the video
                }

                if (frameTimer.milliseconds() >= FRAME_TIME_MS) {
                    frameTimer.reset();
                    display.clear();

                    // Decode the current frame from Base64 back into raw bytes
                    byte[] frameData = Base64.decode(BadAppleData.FRAMES[currentFrame], Base64.DEFAULT);
                    int bitIndex = 0;

                    for (int y = 0; y < VIDEO_HEIGHT; y++) {
                        for (int x = 0; x < VIDEO_WIDTH; x++) {
                            int bytePos = bitIndex / 8;
                            int bitPos = 7 - (bitIndex % 8);
                            boolean isWhite = ((frameData[bytePos] >> bitPos) & 1) == 1;

                            if (isWhite) {
                                display.setPixel(x, (VIDEO_HEIGHT - 1 - y));
                            }
                            bitIndex++;
                        }
                    }

                    // MOVED: Only advance the frame if the timer has triggered
                    currentFrame++;
                }

                telemetry.addLine(display.renderHtml());
                telemetry.addData("hypot", hypot);

                telemetry.addData("x", pip.getPosX(DistanceUnit.INCH));

                telemetry.addData("y", pip.getPosY(DistanceUnit.INCH));
                //telemetry.addData("actual", gate.getPosition());
                telemetry.addData("wanted rpm ", vTarget);
                telemetry.addData("turret position", turret1.getPosition());
                //telemetry.addData("actual rpm left flywheel", ((leftFlywheel.getVelocity() * 60 ) / 28));
                telemetry.addData("actual rpm right flywheel", (fly.getRotation(MeasureUnit.REVOLUTIONS, TimeScale.MINUTES, FlywheelSide.RIGHT)));
                telemetry.addData("time", (intakeTimer.seconds()));
                telemetry.addData("power", middleIntake.getPower());
                telemetry.addData("angle to goal", angleToGoal);
                telemetry.addData("heading", robotHeading);
                telemetry.update();
                pip.update();
            }
        }

    }
    // ---- class level ----
    private static final double FLOWER_DIST  = 68.0;  // flower distance from field center, toward its wall
    private static final double FLOWER_SLIDE = 23.4;  // flower offset along the wall
    private static final double SIDE_TOL     = 4.0;   // +/- inches of sideways error allowed
    private static final double REACH_MIN    = 40;  // robot must be past this distance toward the wall
    private static final double HEADING_TOL  = 30.0;  // +/- degrees of heading error allowed
    private static final double[] FACING     = {0, 90, 180, 270};  // right, top, left, bottom

    private boolean atFlower(double x, double y, double headingDeg) {
        for (double f : FACING) {
            double r = Math.toRadians(f), c = Math.cos(r), s = Math.sin(r);
            double u =  x * c + y * s;   // distance toward this flower's wall
            double v = -x * s + y * c;   // sideways position along that wall
            double err = Math.abs(((headingDeg - f) % 360 + 540) % 360 - 180);
            if (u > REACH_MIN && Math.abs(v - FLOWER_SLIDE) < SIDE_TOL && err < HEADING_TOL) {
                return true;
            }
        }
        return false;
    }


    public static class Renderer {
        private final int width;
        private final int height;
        private final boolean[][] pixels;

        public Renderer(int width, int height) {
            this.width = width;
            this.height = height;
            this.pixels = new boolean[height][width];
        }

        public void clear() {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    pixels[y][x] = false;
        }

        public void setPixel(int x, int y) {
            int py = (height - 1) - y;
            if (x < 0 || py < 0 || x >= width || py >= height) return;
            pixels[py][x] = true;
        }

        private boolean isTrue(int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) return false;
            return pixels[y][x];
        }

        private char braille(int x, int y) {
            int code = 0;
            if (isTrue(x,     y))     code |= 1;
            if (isTrue(x,     y + 1)) code |= 2;
            if (isTrue(x,     y + 2)) code |= 4;
            if (isTrue(x + 1, y))     code |= 8;
            if (isTrue(x + 1, y + 1)) code |= 16;
            if (isTrue(x + 1, y + 2)) code |= 32;
            if (isTrue(x,     y + 3)) code |= 64;
            if (isTrue(x + 1, y + 3)) code |= 128;
            return (char) (0x2800 + code);
        }

        public String renderHtml() {
            int cellRows = (height + 3) / 4;
            int cellCols = (width + 1) / 2;
            StringBuilder out = new StringBuilder(cellRows * cellCols * 2);
            out.append("<pre style='line-height:1; letter-spacing:0;'>");
            for (int y = 0; y < height; y += 4) {
                for (int x = 0; x < width; x += 2) {
                    out.append(braille(x, y));
                }
                out.append('\n');
            }
            out.append("</pre>");
            return out.toString();
        }
    }
}