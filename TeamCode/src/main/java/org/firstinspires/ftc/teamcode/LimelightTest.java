package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {

    // Two reference positions in PedroPathing coordinates (inches)
    private static final double REF1_X = 40.48670487;
    private static final double REF1_Y = 102.3371022;
    private static final double REF2_X = 98.51712049;
    private static final double REF2_Y = 109.2137467;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(50);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL", "Temp: %.1fC  CPU: %.1f%%  FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addData("Latency ms", "%.1f capture + %.1f targeting",
                        result.getCaptureLatency(), result.getTargetingLatency());

                // ---- AprilTag (fiducial) detections ----
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                telemetry.addData("Tags seen", tags.size());
                for (LLResultTypes.FiducialResult fr : tags) {
                    telemetry.addData(
                            String.format("  Tag %d (%s)", fr.getFiducialId(), fr.getFamily()),
                            "tx=%.2f°  ty=%.2f°  area=%.4f",
                            fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getTargetArea());
                }

                // ---- MT1 pose (getBotpose = MegaTag1) ----
                Pose3D mt1 = result.getBotpose();
                if (mt1 != null) {
                    double llX = mt1.getPosition().x;  // meters
                    double llY = mt1.getPosition().y;  // meters
                    double llZ = mt1.getPosition().z;  // meters

                    YawPitchRollAngles ypr = mt1.getOrientation();
                    double yawDeg   = ypr.getYaw(AngleUnit.DEGREES);
                    double pitchDeg = ypr.getPitch(AngleUnit.DEGREES);
                    double rollDeg  = ypr.getRoll(AngleUnit.DEGREES);

                    telemetry.addLine("--- MT1 (Limelight coords) ---");
                    telemetry.addData("  LL x/y/z (m)", "%.4f, %.4f, %.4f", llX, llY, llZ);
                    telemetry.addData("  Yaw/Pitch/Roll", "%.2f°, %.2f°, %.2f°",
                            yawDeg, pitchDeg, rollDeg);

                    // ---- Convert to PedroPathing coordinates ----
                    // PP x = 72 + LL_y_in_inches
                    // PP y = 72 - LL_x_in_inches
                    double llXIn = DistanceUnit.METER.toInches(llX);
                    double llYIn = DistanceUnit.METER.toInches(llY);
                    double ppX = 72.0 + llYIn;
                    double ppY = 72.0 - llXIn;
                    double ppHeadingDeg = Math.toDegrees(
                            AngleUnit.normalizeRadians(ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(90)));

                    telemetry.addLine("--- MT1 (PedroPathing coords) ---");
                    telemetry.addData("  PP x/y (in)", "%.3f, %.3f", ppX, ppY);
                    telemetry.addData("  PP heading", "%.2f°", ppHeadingDeg);

                    // ---- Distances from reference positions ----
                    double dist1 = Math.hypot(ppX - REF1_X, ppY - REF1_Y);
                    double dist2 = Math.hypot(ppX - REF2_X, ppY - REF2_Y);

                    telemetry.addLine("--- Distances ---");
                    telemetry.addData("  Dist to Ref1", "%.3f in",
                            dist1);
                    telemetry.addData("  Dist to Ref2", "%.3f in",
                            dist2);
                } else {
                    telemetry.addLine("MT1 pose: null (not enough tags for localization)");
                }

            } else {
                telemetry.addLine("No valid Limelight result");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
