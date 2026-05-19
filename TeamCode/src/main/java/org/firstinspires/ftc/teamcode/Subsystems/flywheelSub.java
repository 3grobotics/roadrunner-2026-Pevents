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
    public double hypot = 0; // OpMode must update this before calling loop()!

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    // --- PID gains ---
    public double kP = .00033;
    public double kI = .05;
    public double kD = 0.000;
    public double kF = 0.0003;

    public volatile boolean loopActive = false;
    public PIDFController spinController = new PIDFController(kP, kI, kD, kF);

    public flywheelSub(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        if (!loopActive) return;

        // 1) Calculate target RPM into a temporary local variable first
        double newTarget = 0;

        if (hypot < 75) {
            newTarget = 11.091 * hypot + 1257.177;
        } else if (hypot > 75 && hypot < 96.3) {
            newTarget = 15.333 * hypot + 939.025;
            //bad after here
        } else if (hypot > 96.3 && hypot < 129) {
            newTarget = 9.817 * hypot + 1079.623;
        } else if (hypot > 151) {
            newTarget = 11.727 * hypot + 833.217;
        }

        // 2) Clip the local variable and assign it to the volatile 'target' in ONE atomic step
        target = Range.clip(newTarget, 0, 6000);

        // 2) Read sensors (using the dynamic gear ratio instead of hardcoded 37.333)
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;
        double currentSpeed = (((flywheel1.getVelocity() + flywheel2.getVelocity()) / 2) * 60) / effectiveTPR;

        // 3) PID calculation
        spinController.setPIDF(kP, kI, kD, kF);
        double pid = spinController.calculate(currentSpeed, target);

        // 4) Apply power (clip at ±1)
        double power = Math.max(-1, Math.min(1, pid));
        flywheel1.setPower(power);
        flywheel2.setPower(power);
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