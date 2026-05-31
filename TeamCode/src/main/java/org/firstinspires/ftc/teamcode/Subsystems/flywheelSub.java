package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class flywheelSub {
    // --- Enums ---
    public enum MeasureUnit {
        DEGREES, RADIANS, GRADIANS, REVOLUTIONS, TICKS,
        INCHES, FEET, MILES, MILLIMETERS, METERS, KILOMETERS
    }

    public enum TimeScale {
        MILLISECONDS, SECONDS, MINUTES, HOURS
    }

    // --- Flywheel Hardware & Measurement Variables ---
    public static double WHEEL_DIAMETER_INCHES = 2.834646;
    public static double MOTOR_TICKS_PER_REV = 28.0;     // Bare goBILDA motor without gearbox
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    // Local calculation variables
    public double ticksPerSec;
    public double revsPerSec;
    public double revsPerTime;
    public double linearInches;

    // --- Target & Distance Variables ---
    public volatile double target = 0;
    public volatile double target2 = 0;
    public double hypot = 0; // OpMode must update this before calling loop()!

    public double samOffsetV = 400;
    public double samOffsetV2 = 400;
    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    // --- PID gains ---
    public double kP = .00033;
    public double kI = .05;
    public double kD = 0.000;
    public double kF = 0.0003;
    public boolean up = false;
    public boolean down = false;
    public boolean prevUp = false;
    public boolean prevDown = false;

    public boolean up2 = false;
    public boolean down2 = false;
    public boolean prevUp2 = false;
    public boolean prevDown2 = false;


    public volatile boolean loopActive = false;
    public PIDFController spinController = new PIDFController(kP, kI, kD, kF);

    public flywheelSub(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop() {
        if (!loopActive) return;

        // 1) Calculate target RPM into a temporary local variable first
        double newTarget = 0;

        if (hypot < 72) {
            newTarget = 2.083 * hypot + 1600.016;
        } else if (hypot > 72 && hypot < 96) {
            newTarget = 8.333 * hypot + 1150.024;
        } else if (hypot > 96 && hypot < 120) {
            newTarget = 16.667 * hypot + 349.968;
        } else if (hypot > 120 && hypot < 144) {
            newTarget = 6.25 * hypot + 1600;
        } else if (hypot > 144) {
            newTarget = 12.5 * hypot - 300;
        }

        // 2) Clip the local variable and assign it to the volatile 'target' in ONE atomic step
        target = Range.clip(newTarget, 0, 6000);
        if (up && !prevUp && !down) {
            samOffsetV = Range.clip(samOffsetV + 50, -2000, 2000);
        }
        if (down && !prevDown && !up) {
            samOffsetV = Range.clip(samOffsetV - 50, -2000, 2000);
        }

        prevUp = up;
        prevDown = down;

        target = target + samOffsetV;

        // 2) Read sensors (using the dynamic gear ratio instead of hardcoded 37.333)
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;
        double currentSpeed = (((flywheel1.getVelocity() + flywheel2.getVelocity()) / 2) * 60) / effectiveTPR;

        // 3) PID calculation
        double pid = spinController.calculate(currentSpeed, target);

        // 4) Apply power (clip at ±1)
        double power = Math.max(-1, Math.min(1, pid));



        flywheel1.setPower(power);
        flywheel2.setPower(power);


       /* // 2) Clip the local variable and assign it to the volatile 'target' in ONE atomic step
        target2 = Range.clip(newTarget, 0, 6000);
        if (up2 && !prevUp2 && !down2) {
            samOffsetV2 = Range.clip(samOffsetV2 + 50, -2000, 2000);
        }
        if (down2 && !prevDown2 && !up2) {
            samOffsetV2 = Range.clip(samOffsetV2 - 50, -2000, 2000);
        }

        prevUp2 = up2;
        prevDown2 = down2;

        target2 = target2 + samOffsetV2;

        // 2) Read sensors (using the dynamic gear ratio instead of hardcoded 37.333)
        double effectiveTPR2 = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;
        double currentSpeed2 = ((flywheel2.getVelocity() / 2) * 60) / effectiveTPR2;

        // 3) PID calculation
        double pid2 = spinController.calculate(currentSpeed2, target2);

        // 4) Apply power (clip at ±1)
        double power2 = Math.max(-1, Math.min(1, pid2));




        flywheel2.setPower(power2);*/
    }

    /**
     * Calculates the current velocity of the flywheel in a specified measurement unit and time scale.
     * <p>
     * This method dynamically converts the raw motor ticks into either rotational speed
     * (e.g., RPM, degrees per second) or linear surface speed (e.g., miles per hour, meters per second).
     * <p>
     * <b>Important:</b> Any calculation requesting a linear {@link MeasureUnit} (INCHES, FEET, MILES, etc.)
     * relies entirely on the {@code WHEEL_DIAMETER_INCHES} and the {@code EXTERNAL_GEAR_RATIO} class variables. Ensure these variables are set to the
     * exact physical diameter and gear ratio of your flywheel for accurate telemetry.
     * @param unit      The desired unit of measurement (e.g., {@code MeasureUnit.MILES}, {@code MeasureUnit.REVOLUTIONS}).
     * @param timeScale The desired time division (e.g., {@code TimeScale.HOURS}, {@code TimeScale.MINUTES}).
     * @return          The calculated speed of the flywheel as a {@code double}.
     * <p>
     *  <b>Example Usage:</b>
     * <pre>{@code
     * // Get Revolutions Per Minute (RPM)
     * double rpm = getRot(MeasureUnit.REVOLUTIONS, TimeScale.MINUTES);
     *  // Get surface speed in Miles Per Hour (MPH)
     * double mph = getRot(MeasureUnit.MILES, TimeScale.HOURS);
     * }</pre>
     */
    public double getRotation(MeasureUnit unit, TimeScale timeScale) {

        // Calculate the true ticks-per-revolution of the final wheel structure
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

        // 1) Get raw ticks per second from the motors
        ticksPerSec = (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;

        // 2) Convert to a baseline of Revolutions Per Second using the dynamic ratio
        revsPerSec = ticksPerSec / effectiveTPR;

        // 3) Scale to the requested Time format
        revsPerTime = revsPerSec;
        switch (timeScale) {
            case MILLISECONDS: revsPerTime /= 1000.0; break;
            case SECONDS:      break; // Default base
            case MINUTES:      revsPerTime *= 60.0; break;
            case HOURS:        revsPerTime *= 3600.0; break;
        }

        // 4) Convert to the requested Measurement Unit
        switch (unit) {
            // --- Rotational Math ---
            case TICKS:       return revsPerTime * effectiveTPR;
            case REVOLUTIONS: return revsPerTime;
            case DEGREES:     return revsPerTime * 360.0;
            case RADIANS:     return revsPerTime * 2.0 * Math.PI;
            case GRADIANS:    return revsPerTime * 400.0;

            // --- Linear Math ---
            case INCHES:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return linearInches;
            case FEET:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return linearInches / 12.0;
            case MILES:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return linearInches / 63360.0;
            case MILLIMETERS:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return linearInches * 25.4;
            case METERS:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return (linearInches * 25.4) / 1000.0;
            case KILOMETERS:
                linearInches = revsPerTime * (Math.PI * WHEEL_DIAMETER_INCHES);
                return (linearInches * 25.4) / 1000000.0;
            default:
                return 0;
        }
    }

    // --- Activation Voids ---

    public void runFlywheel() {
        loopActive = true;
        // The loop will now automatically handle setting 'target' based on 'hypot'
    }

    public void power0() {
        loopActive = false;
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }
}