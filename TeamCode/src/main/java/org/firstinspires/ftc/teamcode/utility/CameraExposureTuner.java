package org.firstinspires.ftc.teamcode.utility;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name = "Camera Exposure Tuner", group = "Utility")
public class CameraExposureTuner extends LinearOpMode {

    // Edit live from the FTC Dashboard right-hand panel
    public static long EXPOSURE_MS = 15;
    public static int GAIN = 250;

    VisionPortal myVisionPortal;
    private FtcDashboard dashboard;
    private CameraStreamProcessor dashboardCamStream;

    // Debounce variables for gamepad inputs
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadRightPrev = false;
    private boolean dpadLeftPrev = false;

    // EXACT CameraStreamProcessor from sortedTwelve
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

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Initializing VisionPortal...");
        telemetry.update();

        // EXACT Initialization sequence from sortedTwelve
        dashboardCamStream = new CameraStreamProcessor();

        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();

        myVisionPortalBuilder.addProcessor(dashboardCamStream);
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        myVisionPortal = myVisionPortalBuilder.build();

        dashboard.startCameraStream(dashboardCamStream, 60);
        // End EXACT initialization

        telemetry.addLine("Dashboard stream: Webcam 1 / VisionPortal");
        telemetry.addLine("Ready. Use Dashboard OR Gamepad 1 D-Pad to tune:");
        telemetry.addLine("DPad Up/Down = Adjust Exposure (+/- 1ms)");
        telemetry.addLine("DPad Right/Left = Adjust Gain (+/- 5)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. Process Gamepad inputs for manual live-tuning
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;

            if (dpadUp && !dpadUpPrev) EXPOSURE_MS += 1;
            if (dpadDown && !dpadDownPrev) EXPOSURE_MS -= 1;

            if (dpadRight && !dpadRightPrev) GAIN += 5;
            if (dpadLeft && !dpadLeftPrev) GAIN -= 5;

            dpadUpPrev = dpadUp;
            dpadDownPrev = dpadDown;
            dpadRightPrev = dpadRight;
            dpadLeftPrev = dpadLeft;

            // Constrain values to prevent hardware crashes
            EXPOSURE_MS = Math.max(1, EXPOSURE_MS); // Exposure cannot be 0 or negative
            GAIN = Range.clip(GAIN, 0, 255); // Typical standard webcam gain limits

            // 2. Apply Hardware Camera Controls
            if (myVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
                GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);

                if (exposureControl != null && exposureControl.isExposureSupported()) {
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                    }
                    exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                }

                if (gainControl != null) {
                    gainControl.setGain(GAIN);
                }

                telemetry.addData("Camera State", "STREAMING");
            } else {
                telemetry.addData("Camera State", myVisionPortal.getCameraState().toString());
            }

            // 3. Output Telemetry
            telemetry.addData("Current Exposure (ms)", EXPOSURE_MS);
            telemetry.addData("Current Gain", GAIN);
            telemetry.addLine("---");
            telemetry.addLine("Adjust via Gamepad D-Pad or Dashboard Config panel.");
            telemetry.update();
        }

        myVisionPortal.close();
    }
}