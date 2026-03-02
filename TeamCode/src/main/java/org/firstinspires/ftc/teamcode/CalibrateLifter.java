package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Calibrate Lifter", group = "Test")
public class CalibrateLifter extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        lifter.setDirection(DcMotor.Direction.REVERSE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Lifter Calibration Ready");
        telemetry.addLine("Press Play to begin.");
        telemetry.update();

        waitForStart();

        int stallPos = LifterCalibrator.calibrate(
                lifter, telemetry, this::getRuntime, this::opModeIsActive);

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.addLine("Calibration complete!");
            telemetry.addData("Stall was at tick",       stallPos);
            telemetry.addData("Encoder (should be ~0)", lifter.getCurrentPosition());
            telemetry.update();
        }
    }
}
