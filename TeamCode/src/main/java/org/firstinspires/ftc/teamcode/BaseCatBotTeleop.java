package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.Supplier;

@Configurable
public abstract class BaseCatBotTeleop extends BaseCatBot {

    // --- Drive ---
    int upperlimit = 550;
    int lowerlimit = 0;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;
    boolean robotCentric = true;
    protected int numPaths = 5;
    protected Supplier<PathChain>[] pathArray;

    // BLUE "source of truth"
    protected static final Pose[] poseArrayBlue = {
            new Pose(23.9, 132.56, Math.toRadians(144)),   // 0 Blue Start Pose
            new Pose(29.4, 121.8 , Math.toRadians(144)),  // 1 Blue Scoring Pose
            new Pose(104 ,   34,    Math.toRadians(-131)),// 2 Blue Parking Pose
            new Pose(144-16,   16,    Math.toRadians(-180)),// 3 Blue Pickup Pose
            new Pose(28,   70,    Math.toRadians(-180)) // 4 Blue Gate
    };
    protected int lifterTargetPosition = 0;
    protected static final int LIFTER_INCREMENT_TICKS = 1;
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        SCORING(1),
        PARKING(2),
        PICKUP(3),
        GATE(4);

        public final int value;
        AutoTarget(int value) { this.value = value; }
    }

    protected enum DriveMode { CATAPULT, INTAKE }
    protected DriveMode driveMode = DriveMode.INTAKE;

    // Speed cap for all drive movements (0.0 to 1.0)
    protected double driveSpeedCap = 0.5;

    // Smooth command state
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 1;

    // Lifter current limiting
    private static final double LIFTER_BASE_POWER         = 0.3;
    private static final double LIFTER_CURRENT_LIMIT_AMPS = 3.0;
    private static final double LIFTER_KP_DOWN            = 0.1;
    private static final double LIFTER_KP_UP              = 0.01;
    private double lifterPowerScale = 1.0;

    protected boolean dpadUpPressed   = false;
    protected boolean dpadDownPressed = false;
    protected boolean guidePressed    = false;

    // SRSHub / lidar
    protected SRSHub srsHub = null;

    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (getAlliance() == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();

        pathArray = new Supplier[numPaths];
        for (int i = 0; i < numPaths; i++) {
            final int index = i;
            pathArray[index] = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, poseArray[index])))
                    .setHeadingInterpolation(
                            HeadingInterpolator.linearFromPoint(
                                    follower::getHeading,
                                    poseArray[index].getHeading(),
                                    0.8
                            )
                    )
                    .build();
        }

        initHardware();

        // SRSHub init (blocks ~2.5 s while sensors boot)
        try {
            SRSHub.Config hubConfig = new SRSHub.Config();
            hubConfig.addI2CDevice(1, new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4));
            srsHub = hardwareMap.get(SRSHub.class, "srshub");
            srsHub.init(hubConfig);
        } catch (Exception e) {
            telemetry.addData("SRSHub init error", e.getMessage());
            srsHub = null;
        }

        // Lifter-specific setup (beyond direction/brake set in initHardware)
        if (!PoseStorage.lifterCalibrated) {
            calibrateLifter();
        }
        lifterTargetPosition = 0;
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setCurrentAlert(1.0, CurrentUnit.AMPS);
        lifter.setPower(LIFTER_BASE_POWER);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        updatePoseFromLL();

        telemetry.addData("Alliance", getAlliance() == Alliance.BLUE ? "Blue" : "Red");

        if (!automatedDrive) {
            double targetY    = gamepad1.right_stick_y * Math.pow(Math.abs(gamepad1.right_stick_y), 1.5);
            double targetX    = gamepad1.right_stick_x * Math.pow(Math.abs(gamepad1.right_stick_x), 1.5);
            double targetTurn = gamepad1.left_stick_x  * Math.pow(Math.abs(gamepad1.left_stick_x),  1.5) / 1.5;

            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            double mult = slowMode ? slowModeMultiplier : 1.0;
            mult *= driveSpeedCap;

            if (gamepad1.startWasPressed()) {
                robotCentric = !robotCentric;
            }
            driveMode = (lifterTargetPosition < 20) ? DriveMode.INTAKE : DriveMode.CATAPULT;

            if (robotCentric) {
                telemetry.addLine("Robot Centric");
                telemetry.addData("Drive Mode", driveMode == DriveMode.CATAPULT ? "Catapult Front" : "Intake Front");
            } else {
                telemetry.addLine("Field Centric");
                if (getAlliance() == Alliance.BLUE) {
                    cmdX = -cmdX;
                    cmdY = -cmdY;
                }
            }

            double effectiveY = (robotCentric && driveMode == DriveMode.INTAKE) ? -cmdY : cmdY;
            double effectiveX = (robotCentric && driveMode == DriveMode.INTAKE) ? -cmdX : cmdX;
            follower.setTeleOpDrive(
                    -effectiveY * mult,
                    -effectiveX * mult,
                    -cmdTurn * mult,
                    robotCentric
            );
        }

        if (gamepad1.a) {  // move lifterTargetPosition up while going to shooting position
            lifterTargetPosition = upperlimit;
        }

        handleAutoTarget(gamepad1.aWasPressed(), gamepad1.a, AutoTarget.SCORING);
        handleAutoTarget(gamepad1.bWasPressed(), gamepad1.b, AutoTarget.PARKING);
        handleAutoTarget(gamepad1.xWasPressed(), gamepad1.x, AutoTarget.PICKUP);
        handleAutoTarget(gamepad1.yWasPressed(), gamepad1.y, AutoTarget.GATE);

        if (gamepad1.rightBumperWasPressed()) {
            shootCatapult();
        }

        boolean lifterDown     = lifter.getCurrentPosition() < 20;
        boolean lifterGoingUp  = lifterTargetPosition == upperlimit;
        boolean lifterNearTop  = lifter.getCurrentPosition() >= upperlimit - 30;

        if (lifterGoingUp && !lifterNearTop) {
            // Lifter heading up but not yet near the top — funnel game pieces with half-speed intake
            intake.setPower(INTAKE_IN_POWER * 0.5);
        } else if (!lifterDown) {
            intake.setPower(INTAKE_OFF_POWER);
        } else if (gamepad1.left_bumper) {
            intakeOut();
            telemetry.addData("Intake Vel/mA", "%4.2f, %4.2f",
                    intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else if (gamepad1.left_trigger > 0.1) {
            intakeIn();
            telemetry.addData("Intake Vel/mA", "%4.2f, %4.2f",
                    intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else {
            intakeOff();
        }

        // Catstrength servo controls
        {
            if (gamepad2.dpad_up && !dpadUpPressed) {
                catstrengthPosition += CATSTRENGTH_INCREMENT;
                if (catstrengthPosition > CATSTRENGTH_MAX_POSITION) catstrengthPosition = CATSTRENGTH_MAX_POSITION;
                catstrength.setPosition(catstrengthPosition);
                dpadUpPressed = true;
            } else if (!gamepad2.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad2.dpad_down && !dpadDownPressed) {
                catstrengthPosition -= CATSTRENGTH_INCREMENT;
                if (catstrengthPosition < CATSTRENGTH_MIN_POSITION) catstrengthPosition = CATSTRENGTH_MIN_POSITION;
                catstrength.setPosition(catstrengthPosition);
                dpadDownPressed = true;
            } else if (!gamepad2.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.dpad_down) lifterTargetPosition = lowerlimit;
            if (gamepad1.dpad_up)  lifterTargetPosition = upperlimit;

            if (gamepad2.dpad_left) {
                upperlimit += LIFTER_INCREMENT_TICKS;
                lifterTargetPosition = upperlimit;
            }
            if (gamepad2.dpad_right) {
                upperlimit -= LIFTER_INCREMENT_TICKS;
            }
            lifterTargetPosition = Math.max(lifterTargetPosition, lowerlimit);
            lifterTargetPosition = Math.min(lifterTargetPosition, upperlimit);
            lifter.setTargetPosition(lifterTargetPosition);
        }

        double lifterCurrentA = lifter.getCurrent(CurrentUnit.AMPS);
        double currentError = lifterCurrentA - LIFTER_CURRENT_LIMIT_AMPS;
        if (currentError > 0) {
            lifterPowerScale -= currentError * LIFTER_KP_DOWN;
            telemetry.addLine("*** LIFTER CURRENT LIMIT ***");
        } else {
            lifterPowerScale += (-currentError) * LIFTER_KP_UP;
        }
        lifterPowerScale = clamp(lifterPowerScale, 0.0, 1.0);
        lifter.setPower(LIFTER_BASE_POWER * lifterPowerScale);

        telemetry.addData("Catstrength", "%.2f", catstrengthPosition);
        telemetry.addData("Lifter Encoder", lifter.getCurrentPosition());
        telemetry.addData("Lifter Target", lifterTargetPosition);
        telemetry.addData("Lifter Upper Limit", upperlimit);
        telemetry.addData("Lifter Current (mA)", lifterCurrentA * 1000);

        // SRSHub distance grids
        if (srsHub != null) {
            srsHub.update();
            if (srsHub.disconnected()) {
                telemetry.addLine("SRSHub disconnected");
            } else {
                SRSHub.VL53L5CX lidar1 = srsHub.getI2CDevice(1, SRSHub.VL53L5CX.class);

                telemetry.addLine("=== Sensor 1 ===");
                if (!lidar1.disconnected) {
                    for (int row = 0; row < 4; row++) {
                        StringBuilder rowData = new StringBuilder();
                        for (int col = 0; col < 4; col++) {
                            rowData.append(String.format("%4d ", lidar1.distances[row * 4 + col]));
                        }
                        telemetry.addData("S1 R" + row, rowData.toString());
                    }
                } else {
                    telemetry.addLine("Sensor 1 disconnected");
                }
            }
        }

        telemetry.update();
    }

    protected void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();
        Pose ppPose = follower.getPose();

        telemetry.addData("PP X/Y/H", "%4.2f, %4.2f, %4.1f°",
                ppPose.getX(), ppPose.getY(), Math.toDegrees(ppPose.getHeading()));

        if (llPose != null) {
            telemetry.addLine("LL Data Valid");
            if (gamepad1.back) {
                follower.setPose(llPose);
                telemetry.addData("Set Pose from LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
            } else {
                telemetry.addData("LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
            }
            double distErr = Math.hypot(llPose.getX() - ppPose.getX(), llPose.getY() - ppPose.getY());
            double headingErrDeg = Math.toDegrees(
                    AngleUnit.normalizeRadians(llPose.getHeading() - ppPose.getHeading()));
            telemetry.addData("LL-PP error", "dist=%.2f in  hdg=%.1f°", distErr, headingErrDeg);
        } else {
            telemetry.addLine("No LL Data");
        }
    }

    protected void shootCatapult() {
        catapultUp();
        sleep(500);
        catapultDown();
        sleep(500);
        catapultHold();
        lifterTargetPosition = 0;
        lifter.setTargetPosition(0);
    }


    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    protected void handleAutoTarget(boolean buttonWasPressed, boolean buttonIsHeld, AutoTarget target) {
        if (buttonWasPressed && !automatedDrive) {
            follower.followPath(pathArray[target.value].get());
            automatedDrive = true;
            currentAutoTarget = target;
        }
        if (!buttonIsHeld && automatedDrive && currentAutoTarget == target) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }
    }
}
