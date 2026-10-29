package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public enum FlywheelSide {
        LEFT, RIGHT, AVERAGE
    }

    // --- Flywheel Hardware & Measurement Variables ---
    public static double WHEEL_DIAMETER_INCHES = 2.834646;
    public static double MOTOR_TICKS_PER_REV = 28.0;     // Bare goBILDA motor without gearbox
    public static double EXTERNAL_GEAR_RATIO = 1.333333;

    // --- Target & Distance Variables ---
    public volatile double rightTarget = 0;
    public volatile double leftTarget = 0;
    public double hypot = 0; // OpMode must update this before calling loop()!

    // Offsets
    public double voltageOffset = 0;
    public double manualOffsetRight = 0;
    public double manualOffsetLeft = 0;

    private final DcMotorEx rightFlywheel;
    //private final DcMotorEx leftFlywheel;

    // Right Turret PIDF Gains
    public static double rightKp = 0.0007;
    public static double rightKi = 0.0001;
    public static double rightKd = 0;
    public static double rightKf = 0.00026;

    // Left Turret PIDF Gains
    public static double leftKp = .00033;
    public static double leftKi = .05;
    public static double leftKd = 0.000;
    public static double leftKf = 0.0003;

    // Inputs for Right Flywheel manual offset
    public boolean up = false;
    public boolean down = false;
    public boolean prevUp = false;
    public boolean prevDown = false;

    // Inputs for Left Flywheel manual offset
    public boolean up2 = false;
    public boolean down2 = false;
    public boolean prevUp2 = false;
    public boolean prevDown2 = false;

    public double voltage; // OpMode must update this!
    public volatile boolean loopActive = false;

    // Independent PID Controllers for each flywheel
    public PIDFController rightSpinController = new PIDFController(rightKp, rightKi, rightKd, rightKf);
    //public PIDFController leftSpinController = new PIDFController(leftKp, leftKi, leftKd, leftKf);

    public flywheelSub(HardwareMap hardwareMap) {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        //leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");

        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //leftFlywheel .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // CHECK YOUR BUILD: If these are side-by-side flywheels, one usually needs
        // to be FORWARD and the other REVERSE to feed a piece through.
        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        //leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop() {
        if (!loopActive) return;

        // 1) Calculate base target RPM based on distance
        double baseTarget = 0;

        if (hypot <= 38.8) {
            baseTarget =  5.682 * hypot + 2179.54;
        } else if (hypot > 38.8 && hypot <= 44.6) {
            baseTarget =  8.621 * hypot + 2065.505;
        }else if (hypot > 44.6) {
            baseTarget = 13.636 * hypot + 1841.834;
        }

        baseTarget = Range.clip(baseTarget, 0, 6000);

        // 2) Handle Manual Offsets
        // Right Wheel Offset
        if (up && !prevUp && !down) {
            manualOffsetRight = Range.clip(manualOffsetRight + 50, -2000, 2000);
        }
        if (down && !prevDown && !up) {
            manualOffsetRight = Range.clip(manualOffsetRight - 50, -2000, 2000);
        }
        prevUp = up;
        prevDown = down;

        // Left Wheel Offset
        //if (up2 && !prevUp2 && !down2) {
        //    manualOffsetLeft = Range.clip(manualOffsetLeft + 50, -2000, 2000);
        //}
        //if (down2 && !prevDown2 && !up2) {
        //    manualOffsetLeft = Range.clip(manualOffsetLeft - 50, -2000, 2000);
        //}
        //prevUp2 = up2;
        //prevDown2 = down2;

        // 3) Calculate shared Voltage Offset
        voltageOffset = -230.769 * Range.clip(voltage, 0, 12.2) + 2715.382;

        // 4) Assign final specific targets in ONE atomic step
        rightTarget = baseTarget /* + voltageOffset*/ + manualOffsetRight;
        //leftTarget = baseTarget + voltageOffset + manualOffsetLeft;

        // 5) Read sensors independently
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

        double currentSpeedRight = (rightFlywheel.getVelocity() * 60.0) / effectiveTPR;
        //double currentSpeedLeft = (leftFlywheel.getVelocity() * 60.0) / effectiveTPR;

        // 6) Independent PID calculations
        double pidRight = rightSpinController.calculate(currentSpeedRight, rightTarget);
        //double pidLeft = leftSpinController.calculate(currentSpeedLeft, leftTarget);

        // 7) Apply power (clip at ±1)
        double powerRight = Math.max(-1, Math.min(1, pidRight));
        //double powerLeft = Math.max(-1, Math.min(1, pidLeft));

        rightFlywheel.setPower(powerRight);
        //leftFlywheel.setPower(powerLeft);
    }

    /**
     * Default getRotation assumes you want the AVERAGE of both wheels.
     */
    public double getRotation(MeasureUnit unit, TimeScale timeScale) {
        return getRotation(unit, timeScale, FlywheelSide.AVERAGE);
    }

    /**
     * Calculates the current velocity of a specific flywheel in a specified measurement unit and time scale.
     * <p>
     * This method dynamically converts the raw motor ticks into either rotational speed
     * (e.g., RPM, degrees per second) or linear surface speed (e.g., miles per hour, meters per second).
     * <p>
     * <b>Important:</b> Any calculation requesting a linear {@link MeasureUnit} (INCHES, FEET, MILES, etc.)
     * relies entirely on the {@code WHEEL_DIAMETER_INCHES} and the {@code EXTERNAL_GEAR_RATIO} class variables. Ensure these variables are set to the
     * exact physical diameter and gear ratio of your flywheel for accurate telemetry.
     *
     * @param unit      The desired unit of measurement (e.g., {@code MeasureUnit.MILES}, {@code MeasureUnit.REVOLUTIONS}).
     * @param timeScale The desired time division (e.g., {@code TimeScale.HOURS}, {@code TimeScale.MINUTES}).
     * @param side      The specific flywheel to measure (e.g., {@code FlywheelSide.LEFT}, {@code FlywheelSide.RIGHT}).
     * @return          The calculated speed of the flywheel as a {@code double}.
     * <p>
     * <b>Example Usage:</b>
     * <pre>{@code
     * // Get average Revolutions Per Minute (RPM) of both flywheels
     * double avgRpm = fly.getRotation(MeasureUnit.REVOLUTIONS, TimeScale.MINUTES);
     *
     * // Get surface speed of just the left flywheel in Miles Per Hour (MPH)
     * double leftMph = fly.getRotation(MeasureUnit.MILES, TimeScale.HOURS, FlywheelSide.LEFT);
     * }</pre>
     */
    public double getRotation(MeasureUnit unit, TimeScale timeScale, FlywheelSide side) {

        // Calculate the true ticks-per-revolution of the final wheel structure
        double effectiveTPR = MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO;

        // Use local variables to prevent Thread cross-contamination in telemetry
        double ticksPerSec = 0;

        // 1) Get raw ticks per second from the requested motor(s)
        switch (side) {
            case RIGHT:
                ticksPerSec = rightFlywheel.getVelocity();
                break;
            //case LEFT:
            //    ticksPerSec = leftFlywheel.getVelocity();
            //    break;
            case AVERAGE:
            default:
                ticksPerSec = (rightFlywheel.getVelocity()) /*+ leftFlywheel.getVelocity()) / 2.0*/;
                break;
        }

        // 2) Convert to a baseline of Revolutions Per Second using the dynamic ratio
        double revsPerSec = ticksPerSec / effectiveTPR;

        // 3) Scale to the requested Time format
        double revsPerTime = revsPerSec;
        switch (timeScale) {
            case MILLISECONDS: revsPerTime /= 1000.0; break;
            case SECONDS:      break; // Default base
            case MINUTES:      revsPerTime *= 60.0; break;
            case HOURS:        revsPerTime *= 3600.0; break;
        }

        // 4) Convert to the requested Measurement Unit
        double linearInches;
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
    }

    public void power0() {
        loopActive = false;
        rightFlywheel.setPower(0);
        //leftFlywheel.setPower(0);
    }
}