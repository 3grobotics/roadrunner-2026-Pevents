package org.firstinspires.ftc.teamcode.teleop;

import android.util.Base64;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    DcMotorEx             leftIntake    ;
    DcMotorEx             leftFlywheel  ;
    DcMotorEx             rightIntake   ;
    DcMotorEx             rightFlywheel ;
    DcMotorEx             gecko         ;
    Servo                 turret1       ;
    Servo                 turret2       ;
    GoBildaPinpointDriver pip           ;



    private BadAppleHardcoded.Renderer display;
    private ElapsedTime frameTimer;
    private ElapsedTime intakeTimer;
    private ElapsedTime trigTimer;

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

    double samOffset = 5;

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

    int tiltState   = 0;

    boolean tPressed = false;

    private VoltageSensor controlHubVoltageSensor;

    // --- NEW HUB LIST VARIABLE ---
    private List<LynxModule> allHubs;

    double currentVoltage;

    public flywheelSub fly;
    public varSub v;
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

        leftIntake    = hardwareMap.get(DcMotorEx.class,  "leftIntake");
        //leftFlywheel  = hardwareMap.get(DcMotorEx.class,  "leftFlywheel");
        rightIntake   = hardwareMap.get(DcMotorEx.class,  "rightIntake");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,  "rightFlywheel");

        gecko = hardwareMap.get(DcMotorEx.class,  "gecko");

        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);



        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection (DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection (DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection  (DcMotorSimple.Direction.FORWARD);

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

        pip.setOffsets(-105.091, -119.462, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        allHubs = hardwareMap.getAll(LynxModule.class);

        fly = new flywheelSub(hardwareMap);
        v = new varSub();
        waitForStart();


        while (opModeIsActive()) {

            currentVoltage = controlHubVoltageSensor.getVoltage();

            if (gamepad1.dpad_up) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, 62.36844280573327, 9.84, AngleUnit.DEGREES, 180));
            } else if (gamepad1.dpad_down) {
                pip.resetPosAndIMU();
            } else if (gamepad1.dpad_left) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, 66, 66, AngleUnit.DEGREES, 90));
            }

            robotX = pip.getPosX(DistanceUnit.INCH);
            robotY = pip.getPosY(DistanceUnit.INCH);


            if (gamepad1.touchpad && !tPressed) {
                tiltState = (tiltState + 1) % 2;
                tPressed = true;
            } else if (!gamepad1.touchpad) {
                tPressed = false;
            }

            switch (tiltState){
                case 0:
                    xl = tx - robotX;
                    yl = ty - robotY;
                    break;
                case 1:
                    xl = tiltedTx - robotX;
                    yl = tiltedTy - robotY;
                    break;
            }

            hypot = Math.sqrt((xl * xl) + (yl * yl));



            /* turret stuff*/ {
                angleToGoal = Math.atan2(yl, xl);
                robotHeading = pip.getHeading(AngleUnit.RADIANS);

                calculatedTurretRad = angleToGoal - robotHeading;

                while (calculatedTurretRad > Math.PI) {
                    calculatedTurretRad -= 2 * Math.PI;
                }
                while (calculatedTurretRad < -Math.PI) {
                    calculatedTurretRad += 2 * Math.PI;
                }

                finalServoDegrees = Math.toDegrees(calculatedTurretRad) + (543.636 / 2);

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
                finalServoDegrees = Range.clip(finalServoDegrees, 0, 543.636);
                if (gamepad1.ps || gamepad2.ps) {
                    turret1.setPosition(.5);
                    turret2.setPosition(.5);
                } else {
                    turret1.setPosition(finalServoDegrees / 543.636);
                    turret2.setPosition(finalServoDegrees / 543.636);
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

            /*flywheels*/
                // Flywheel Update
                fly.hypot = hypot;
                fly.voltage = currentVoltage;
                fly.up = gamepad1.b;
                fly.down = gamepad1.a;

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

                max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
                frontLeft.setPower(fl / max);
                frontRight.setPower(fr / max);
                backLeft.setPower(bl / max);
                backRight.setPower(br / max);
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
            //telemetry.addData("actual", gate.getPosition());
            telemetry.addData("wanted rpm ", vTarget);
            //telemetry.addData("actual rpm left flywheel", ((leftFlywheel.getVelocity() * 60 ) / 28));
            telemetry.addData("actual rpm right flywheel", (fly.getRotation(MeasureUnit.REVOLUTIONS, TimeScale.MINUTES, FlywheelSide.RIGHT)));
            telemetry.addData("time", (intakeTimer.seconds()));
            telemetry.addData("power", rightIntake.getPower());
            telemetry.addData("intake velocity", (rightIntake.getVelocity() * 60 ) / 28);
            telemetry.addData("i v", (iv));

            telemetry.update();
            pip.update();
            }
        }

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