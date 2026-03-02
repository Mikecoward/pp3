package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public abstract class BaseCatBot extends OpMode {

    // ---- Alliance ----
    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    private static final double FIELD_SIZE_IN = 144.0;

    protected Pose mirrorBlueToRed(Pose bluePose) {
        double x = FIELD_SIZE_IN - bluePose.getX();
        double y = bluePose.getY();
        double h = AngleUnit.normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    // ---- Hardware ----
    protected Follower     follower;
    protected Limelight3A  limelight;
    protected DcMotorEx    intake     = null;
    protected DcMotorEx    catapult1  = null;
    protected DcMotorEx    catapult2  = null;
    protected DcMotorEx    lifter     = null;
    protected Servo        catstrength = null;

    // ---- Catstrength ----
    protected static final double CATSTRENGTH_INITIAL_POSITION = 0.65;
    protected static final double CATSTRENGTH_MIN_POSITION     = 0.25;
    protected static final double CATSTRENGTH_MAX_POSITION     = 0.75;
    protected static final double CATSTRENGTH_INCREMENT        = 0.01;
    protected double catstrengthPosition = CATSTRENGTH_INITIAL_POSITION;

    // ---- Power constants ----
    protected double INTAKE_IN_POWER          = 1.0;
    protected double INTAKE_OFF_POWER         =  0.0;

    protected double INTAKE_OUT_POWER         = -1.0;
    protected double CATAPULT_UP_POWER        =  1.0;
    protected double CATAPULT_DOWN_POWER      = -1.0;
    protected double CATAPULT_HOLD_DOWN_POWER = -0.2;

    protected boolean slowMode           = false;
    protected double  slowModeMultiplier = 0.5;

    /**
     * Initialises all shared hardware: limelight, drive motors, catstrength servo,
     * and lifter direction/brake. Subclasses call this at the start of their init().
     */
    protected void initHardware() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        intake    = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        catapult1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rcat");
        catapult2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lcat");

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        catstrength = hardwareMap.get(Servo.class, "catstrength");
        catstrengthPosition = CATSTRENGTH_INITIAL_POSITION;
        catstrength.setPosition(catstrengthPosition);

        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        lifter.setDirection(DcMotor.Direction.REVERSE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---- Limelight pose conversion ----
    protected Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D llpose = result.getBotpose();
        if (llpose == null) return null;

        double xMeters = llpose.getPosition().x;
        double yMeters = llpose.getPosition().y;

        double xInches = 72 + DistanceUnit.METER.toInches(yMeters);
        double yInches = 72 - DistanceUnit.METER.toInches(xMeters);

        YawPitchRollAngles ypr = llpose.getOrientation();
        double headingRad = AngleUnit.normalizeRadians(ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(90));

        return new Pose(xInches, yInches, headingRad);
    }

    // ---- Catapult helpers ----
    protected void catapultUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }
    protected void catapultDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }
    protected void catapultHold() {
        catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
        catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

    // ---- Intake helpers ----
    protected void intakeIn()  { intake.setPower(INTAKE_IN_POWER); }
    protected void intakeOut() { intake.setPower(INTAKE_OUT_POWER); }
    protected void intakeOff() { intake.setPower(INTAKE_OFF_POWER); }



    // ---- Lifter calibration ----
    protected void calibrateLifter() {
        LifterCalibrator.calibrate(lifter, telemetry, this::getRuntime, () -> true);
    }

    // ---- Lifter position helpers ----
    protected static final int    LIFTER_UP_POSITION  = 135;
    protected static final int    LIFTER_DOWN_POSITION = 0;
    protected static final double LIFTER_POWER         = 0.2;

    protected void lifterUp() {
        lifter.setTargetPosition(LIFTER_UP_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_POWER);
    }

    protected void lifterDown() {
        lifter.setTargetPosition(LIFTER_DOWN_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_POWER);
    }

    // ---- Utility ----
    protected static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}
