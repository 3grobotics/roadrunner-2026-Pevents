package org.firstinspires.ftc.teamcode.teleop;

import android.util.Base64;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BadAppleData;
import org.firstinspires.ftc.teamcode.BadAppleHardcoded;

@Config
@TeleOp(name = "drivetrain", group = "teleop")
public class drivetrainTeleop extends LinearOpMode {

    DcMotorEx frontRight;
    DcMotorEx backRight;
    DcMotorEx frontLeft;
    DcMotorEx backLeft;

    private BadAppleHardcoded.Renderer display;
    private ElapsedTime frameTimer;

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
    @Override
    public void runOpMode() {

        frontRight = hardwareMap.get(DcMotorEx.class,  "frontRight");
        backRight  = hardwareMap.get(DcMotorEx.class,  "backRight ");
        backLeft   = hardwareMap.get(DcMotorEx.class,  "backLeft  ");
        frontLeft  = hardwareMap.get(DcMotorEx.class,  "frontLeft ");

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

        waitForStart();


        while (opModeIsActive()) {

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
            telemetry.update();}
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