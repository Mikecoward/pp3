package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LifterCalibrator {

    public static final double STALL_CURRENT_AMPS = 1.0;
    public static final double INITIAL_POWER      = -0.1;
    public static final double POWER_RAMP_PER_SEC = 0.05;
    public static final int    BACKOFF_TICKS       = 15;
    public static final double BACKOFF_POWER       = 0.1;

    /**
     * Ramps lifter power until stall, backs off, and resets encoder to 0.
     *
     * @param lifter           lifter motor (direction/brake already configured)
     * @param telemetry        for status output
     * @param runtimeSeconds   runtime supplier, e.g. this::getRuntime
     * @param continueRunning  abort check, e.g. this::opModeIsActive or () -> true
     * @return stall position in ticks (before encoder reset), or 0 if aborted
     */
    public static int calibrate(DcMotorEx lifter, Telemetry telemetry,
                                DoubleSupplier runtimeSeconds, BooleanSupplier continueRunning) {
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power    = INITIAL_POWER;
        double lastTime = runtimeSeconds.getAsDouble();
        lifter.setPower(power);

        // Phase 1: ramp power until stall
        while (continueRunning.getAsBoolean()) {
            double current = lifter.getCurrent(CurrentUnit.AMPS);
            telemetry.addLine("Calibrating lifter...");
            telemetry.addData("Power",       "%.2f", power);
            telemetry.addData("Current (A)", "%.3f", current);
            telemetry.addData("Encoder",     lifter.getCurrentPosition());
            telemetry.update();

            if (current > STALL_CURRENT_AMPS) break;

            double now = runtimeSeconds.getAsDouble();
            power -= POWER_RAMP_PER_SEC * (now - lastTime);
            power  = Math.max(power, -1.0);
            lastTime = now;
            lifter.setPower(power);
            sleep(20);
        }

        lifter.setPower(0);
        if (!continueRunning.getAsBoolean()) return 0;

        // Phase 2: back off
        int stallPos      = lifter.getCurrentPosition();
        int backoffTarget = stallPos + BACKOFF_TICKS;
        lifter.setTargetPosition(backoffTarget);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(BACKOFF_POWER);

        while (continueRunning.getAsBoolean() && lifter.isBusy()) {
            telemetry.addLine("Lifter: backing off...");
            telemetry.addData("Stall position",   stallPos);
            telemetry.addData("Current position", lifter.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        lifter.setPower(0);

        // Phase 3: reset encoder — this position is now 0
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PoseStorage.lifterCalibrated = true;
        telemetry.addLine("Lifter calibration complete.");
        telemetry.update();
        return stallPos;
    }

    private static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}
