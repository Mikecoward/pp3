package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.PoseStorage;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Configurable
public abstract class BaseCatBotAuto extends BaseCatBot {

    // ---- Auto-specific constants ----
    protected double INTAKE_OUT_POWER = 0.9;

    // ---- Path / state machine ----
    protected int numPaths = 4;
    protected double intakeChainSpeed        = 0.5;  // speed for approach/return legs
    protected double intakeSegmentSpeed      = 0.2;  // speed while collecting (slow)
    protected double shootVelocityThreshold  = 1.0;  // in/s — wait for robot to settle before shooting
    protected PathChain[] pathChains;

    protected ActionScheduler scheduler = new ActionScheduler();
    protected Timer stateTimer = new Timer();
    public int state = 0;

    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    // ---- Debug step mode ----
    protected boolean debugMode    = false;
    private   boolean debugWaiting = false;
    private   boolean initAPressed = false;

    // ---- CSV data log ----
    private static final String LOG_DIR = "/sdcard/FIRST/";
    private BufferedWriter debugLog = null;

    // BLUE "source of truth"
    protected static final Pose[] poseArrayBlue = {
            new Pose(23.9, 132.56, Math.toRadians(144)),   // 0 Blue Start Pose
            new Pose(29.4, 121.8 , Math.toRadians(144)),  // 1 Blue Scoring Pose
            new Pose(40,   34,    Math.toRadians(-131)),  // 2 Blue Parking Pose
            new Pose(24,   100,    Math.toRadians(90)),    // 3 Blue intake A start
            new Pose(24,   90,    Math.toRadians(90)),    // 4 Blue intake A end
            new Pose(24,   76,    Math.toRadians(90)),    // 5 Blue intake B start
            new Pose(24,   66,    Math.toRadians(90)),    // 6 Blue intake B end
            new Pose(24,   52,    Math.toRadians(90)),    // 7 Blue intake C start
            new Pose(24,   42,    Math.toRadians(90)),    // 8 Blue intake C end
            new Pose(44,   105,   Math.toRadians(180)),   // 9 Unused
            new Pose(20,   70,    Math.toRadians(180)),   // 10 Gate Start
            new Pose(12,   70,    Math.toRadians(180)),   // 11 Gate End
    };

    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        SCORING(1),
        PARKING(2),
        AUTO_A_START(3),
        AUTO_A_END(4),
        AUTO_B_START(5),
        AUTO_B_END(6),
        AUTO_C_START(7),
        AUTO_C_END(8),
        AUTO_LINE_SEG(9),
        AUTO_GATE_START(10),
        AUTO_GATE_END(11);

        public final int idx;
        AutoTarget(int idx) { this.idx = idx; }
    }

    @Override
    public void init() {
        Alliance alliance = getAlliance();
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (alliance == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
            if (alliance == Alliance.RED && i == 0) {
                poseArray[i] = new Pose(121, 129.3, Math.toRadians(36)); // Red Start Pose override
            } else {
                poseArray[i] = (alliance == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
            }
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[AutoTarget.STARTING.idx]);
        follower.update();

        buildPaths();

        initHardware();

        calibrateLifter();
        lifterUp();

        catapultHold();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a && !initAPressed) {
            debugMode = !debugMode;
            initAPressed = true;
        } else if (!gamepad1.a) {
            initAPressed = false;
        }
        telemetry.addData("Debug Mode", debugMode ? "ON" : "OFF");
        telemetry.addLine("Press A to toggle debug | Press Play to start");
        telemetry.update();
    }

    @Override
    public void start() {
//        catapultDown();
//        scheduler.atSec(getRuntime() + 0.25, this::catapultHold);

        try {
            String ts = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            debugLog = new BufferedWriter(new FileWriter(LOG_DIR + "autoDebug_" + ts + ".csv"));
            debugLog.write("time_s,state,debug_waiting," +
                    "pp_x,pp_y,pp_hdeg," +
                    "ll_valid,ll_x,ll_y,ll_hdeg,ll_dist_err," +
                    "lifter_pos,lifter_target,lifter_pwr," +
                    "vel_x,vel_y,vel_mag,vel_ang_degs," +
                    "follower_busy\n");
        } catch (IOException e) {
            telemetry.addData("Log open error", e.getMessage());
        }
    }

    @Override
    public void stop() {
        if (debugLog != null) {
            try { debugLog.flush(); debugLog.close(); } catch (IOException ignored) {}
            debugLog = null;
        }
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("state", state);
        PoseStorage.currentPose = follower.getPose();

        scheduler.update(getRuntime());

        if (debugMode && debugWaiting) {
            addDebugTelemetry();
            telemetry.addLine("*** DEBUG: Press A to advance ***");
            if (gamepad1.aWasPressed()) {
                debugWaiting = false;
            }
            logDebugData();
            telemetry.update();
            return;
        }

        int prevState = state;
        switch (state) {
            case 0: // Start -> Scoring
                follower.followPath(pathChains[0], true);
                state = 1;
                break;

            case 1: // Arrived at scoring - wait for settle, then shoot
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < shootVelocityThreshold) {
                    //shootCatapultnew();
                    scheduler.atSec(getRuntime() + 1.5, this::shootCatapultnew);
                    scheduler.atSec(getRuntime() + 2.5, this::setstate2);
                    state = 100;
                }
                break;

            case 2: // Intake A: Scoring -> A_Start -> A_End -> Scoring
                follower.followPath(pathChains[1], intakeChainSpeed, true);
                state = 3;
                break;

            case 3: // Arrived at scoring after A - wait for settle, then shoot
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < shootVelocityThreshold) {
                    shootCatapultnew();
                    scheduler.atSec(getRuntime() + 0.5, this::setstate4);
                    state = 100;
                }
                break;

            case 4: // Intake B: Scoring -> B_Start -> B_End -> Scoring
                follower.followPath(pathChains[2], intakeChainSpeed, true);
                state = 5;
                break;

            case 5: // Arrived at scoring after B - wait for settle, then shoot
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < shootVelocityThreshold) {
                    shootCatapultnew();
                    scheduler.atSec(getRuntime() + 0.5, this::setstate6);
                    state = 100;
                }
                break;

            case 6: // Intake C: Scoring -> C_Start -> C_End -> Scoring
                follower.followPath(pathChains[3], intakeChainSpeed, true);
                state = 7;
                break;

            case 7: // Arrived at scoring after C - wait for settle, then shoot
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < shootVelocityThreshold) {
                    shootCatapultnew();
                    state = 100;
                }
                break;

            default:
                break;
        }

        if (debugMode && state != prevState) {
            debugWaiting = true;
        }

        logDebugData();
        telemetry.update();
    }

    // ---- Catapult sequences (scheduler-based for non-blocking use in auto) ----
    protected void shootCatapult() {
        double now = getRuntime();
        //scheduler.atSec(now,       this::catapultUp);
        lifterDown();
        //catapultUp();
        scheduler.atSec(now + 0.2, this::catapultUp);
        scheduler.atSec(now + 0.8, this::catapultDown);
        scheduler.atSec(now + 1.2, this::catapultHold);

    }

    protected void shootCatapultnew() {
        shootCatapult();
        /*
        double now = getRuntime();
        scheduler.atSec(now,       this::catapultUp);
        scheduler.atSec(now + 0.5, this::catapultHold);
        scheduler.atSec(now + 1.5, this::catapultDown);
        scheduler.atSec(now + 2.0, this::catapultHold);
        scheduler.atSec(now + 2.1, this::lifterDown);
         */
    }

    // ---- State setters (used as scheduler callbacks) ----
    protected void setstate2() { state = 2; }
    protected void setstate4() { state = 4; }
    protected void setstate6() { state = 6; }

    // ---- Debug telemetry ----
    private void addDebugTelemetry() {
        Pose ppPose = follower.getPose();
        telemetry.addData("PP X/Y/H", "%4.2f, %4.2f, %4.1f°",
                ppPose.getX(), ppPose.getY(), Math.toDegrees(ppPose.getHeading()));
        Pose llPose = getRobotPoseFromCamera();
        if (llPose != null) {
            telemetry.addData("LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                    llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
        } else {
            telemetry.addLine("No LL Data");
        }
    }

    private void logDebugData() {
        if (debugLog == null) return;
        Pose pp = follower.getPose();
        Pose ll = getRobotPoseFromCamera();
        double llX = 0, llY = 0, llH = 0, distErr = 0;
        boolean llValid = (ll != null);
        if (llValid) {
            llX = ll.getX(); llY = ll.getY(); llH = Math.toDegrees(ll.getHeading());
            distErr = Math.hypot(llX - pp.getX(), llY - pp.getY());
        }
        com.pedropathing.math.Vector vel = follower.getVelocity();
        double velX   = vel.getXComponent();
        double velY   = vel.getYComponent();
        double velMag = vel.getMagnitude();
        double velAng = Math.toDegrees(follower.getAngularVelocity());
        try {
            debugLog.write(String.format(Locale.US,
                    "%.3f,%d,%b,%.2f,%.2f,%.1f,%b,%.2f,%.2f,%.1f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%b\n",
                    getRuntime(), state, debugWaiting,
                    pp.getX(), pp.getY(), Math.toDegrees(pp.getHeading()),
                    llValid, llX, llY, llH, distErr,
                    lifter.getCurrentPosition(), lifter.getTargetPosition(), lifter.getPower(),
                    velX, velY, velMag, velAng,
                    follower.isBusy()));
        } catch (IOException ignored) {}
    }

    // ---- Path building ----
    protected void buildPaths() {
        pathChains = new PathChain[numPaths];

        // 0: Starting pose -> Scoring (robot stops here to shoot)
        pathChains[0] = simplePathChain(AutoTarget.STARTING.idx, AutoTarget.SCORING.idx, 0.8);

        // 1: Scoring -> A_Start -> A_End -> Scoring (smooth, no stop at waypoints)
        //    Intake turns on as robot approaches A_Start (t=0.8 on segment 0).
        //    Lifter rises and intake schedules off as robot departs A_End (t=0.05 on segment 2).
        pathChains[1] = intakeChain(AutoTarget.AUTO_A_START.idx, AutoTarget.AUTO_A_END.idx);

        // 2: Scoring -> B_Start -> B_End -> Scoring
        pathChains[2] = intakeChain(AutoTarget.AUTO_B_START.idx, AutoTarget.AUTO_B_END.idx);

        // 3: Scoring -> C_Start -> C_End -> Scoring
        pathChains[3] = intakeChain(AutoTarget.AUTO_C_START.idx, AutoTarget.AUTO_C_END.idx);
    }

    /** Builds a 3-segment intake PathChain: Scoring -> start -> end -> Scoring.
     *  Callbacks handle intake on/off, lifter, and per-segment speed without stopping. */
    private PathChain intakeChain(int startIdx, int endIdx) {
        int s = AutoTarget.SCORING.idx;
        return follower.pathBuilder()
                // Segment 0: Scoring -> intake start (fast approach)
                .addPath(new BezierLine(poseArray[s], poseArray[startIdx]))
                .setLinearHeadingInterpolation(
                        poseArray[s].getHeading(), poseArray[startIdx].getHeading(), 0.4)
                .addParametricCallback(0.8, this::intakeIn)   // intake on while approaching waypoint
                .addParametricCallback(0.9, () ->             // slow down just before intake segment
                        follower.setMaxPowerScaling(intakeSegmentSpeed))

                // Segment 1: intake start -> intake end (slow collection)
                .addPath(new BezierLine(poseArray[startIdx], poseArray[endIdx]))
                .setLinearHeadingInterpolation(
                        poseArray[startIdx].getHeading(), poseArray[endIdx].getHeading(), 0.8)

                // Segment 2: intake end -> Scoring (return at full speed)
                .addPath(new BezierLine(poseArray[endIdx], poseArray[s]))
                .setLinearHeadingInterpolation(
                        poseArray[endIdx].getHeading(), poseArray[s].getHeading(), 0.8)
                .addParametricCallback(0.05, () -> {          // early in return leg:
                    follower.setMaxPowerScaling(intakeChainSpeed); // restore speed
                    lifterUp();                               //   raise lifter for scoring
                    scheduler.atSec(getRuntime() + 1.5, this::intakeOff); // stop intake
                })
                .build();
    }

    protected PathChain simplePathChain(int start, int end, double headingTime) {
        return follower.pathBuilder()
                .addPath(new BezierLine(poseArray[start], poseArray[end]))
                .setLinearHeadingInterpolation(
                        poseArray[start].getHeading(),
                        poseArray[end].getHeading(),
                        headingTime
                )
                .build();
    }
}
