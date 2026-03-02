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
    protected int numPaths = 19;
    protected double speedfactor = 0.20;
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
            new Pose(24.8, 134.2, Math.toRadians(144)),   // 0 Blue Start Pose
            new Pose(32.3, 117.9 , Math.toRadians(144)),  // 1 Blue Scoring Pose
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
            case 0:
                follower.followPath(pathChains[0], true);
                state = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    shootCatapultnew();
                    scheduler.atSec(getRuntime() + 0.5, this::setstate2);
                    state = 100;
                }
                break;

            case 2: // Scoring -> A start
                follower.followPath(pathChains[1], true);
                state = 3;
                break;
            case 3: // A Start -> A End
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[2], speedfactor, true);
                    state = 4;
                }
                break;

            case 4: // A End -> Scoring
                if (!follower.isBusy()) {
                    lifterUp();
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[3], true);
                    state = 5;
                }
                break;

            case 5: // Shoot
                if (!follower.isBusy()) {
                    shootCatapultnew();
                    scheduler.atSec(getRuntime() + 0.5, this::setstate6);
                    state = 100;
                }
                break;

            case 6: // Scoring -> B Start
                follower.followPath(pathChains[4], true);
                state = 7;
                break;

            case 7: // B Start -> B End
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[5], speedfactor, true);
                    state = 8;
                }
                break;

            case 8: // B End -> Scoring
                if (!follower.isBusy()) {
                    lifterUp();
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[6], true);
                    state = 9;
                }
                break;

            case 9: // Shoot
                if (!follower.isBusy()) {
                    shootCatapultnew();
                    scheduler.atSec(getRuntime() + 0.5, this::setstate10);
                    state = 100;
                }
                break;

            case 10: // Scoring -> C Start
                follower.followPath(pathChains[7], true);
                state = 11;
                break;

            case 11: // C Start -> C End
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[8], speedfactor, true);
                    state = 12;
                }
                break;

            case 12: // C End
                if (!follower.isBusy()) {
                    lifterUp();
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[15], true);
                    state = 100;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[9], true);
                    state = 14;
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
        scheduler.atSec(now,       this::catapultUp);
        scheduler.atSec(now + 0.5, this::catapultDown);
        scheduler.atSec(now + 1.0, this::catapultHold);
    }

    protected void shootCatapultnew() {
        double now = getRuntime();
        scheduler.atSec(now,       this::catapultUp);
        scheduler.atSec(now + 0.5, this::catapultHold);
        scheduler.atSec(now + 1.5, this::catapultDown);
        scheduler.atSec(now + 2.0, this::catapultHold);
        scheduler.atSec(now + 2.1, this::lifterDown);
    }

    // ---- State setters (used as scheduler callbacks) ----
    protected void setstate2()  { state = 2; }
    protected void setstate6()  { state = 6; }
    protected void setstate10() { state = 10; }

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

        pathChains[0]  = simplePathChain(AutoTarget.STARTING.idx,      AutoTarget.SCORING.idx,       0.8);
        pathChains[1]  = simplePathChain(AutoTarget.SCORING.idx,       AutoTarget.AUTO_A_START.idx,  0.4);
        pathChains[2]  = simplePathChain(AutoTarget.AUTO_A_START.idx,  AutoTarget.AUTO_A_END.idx,    0.8);
        pathChains[3]  = simplePathChain(AutoTarget.AUTO_A_END.idx,    AutoTarget.SCORING.idx,       0.8);

        pathChains[4]  = simplePathChain(AutoTarget.SCORING.idx,       AutoTarget.AUTO_B_START.idx,  0.4);
        pathChains[5]  = simplePathChain(AutoTarget.AUTO_B_START.idx,  AutoTarget.AUTO_B_END.idx,    0.8);
        pathChains[6]  = simplePathChain(AutoTarget.AUTO_B_END.idx,    AutoTarget.SCORING.idx,       0.8);

        pathChains[7]  = simplePathChain(AutoTarget.SCORING.idx,       AutoTarget.AUTO_C_START.idx,  0.4);
        pathChains[8]  = simplePathChain(AutoTarget.AUTO_C_START.idx,  AutoTarget.AUTO_C_END.idx,    0.8);
        pathChains[9]  = simplePathChain(AutoTarget.AUTO_C_START.idx,  AutoTarget.AUTO_GATE_START.idx, 0.8);

        pathChains[10] = simplePathChain(AutoTarget.SCORING.idx,       AutoTarget.AUTO_LINE_SEG.idx, 0.8);
        pathChains[11] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx, AutoTarget.AUTO_A_START.idx,  0.8);
        pathChains[12] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx, AutoTarget.AUTO_B_START.idx,  0.8);
        pathChains[13] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx, AutoTarget.AUTO_C_START.idx,  0.8);

        pathChains[14] = simplePathChain(AutoTarget.AUTO_B_END.idx,    AutoTarget.AUTO_B_START.idx,  0.8);
        pathChains[15] = simplePathChain(AutoTarget.AUTO_C_END.idx,    AutoTarget.AUTO_GATE_START.idx, 0.8);
        pathChains[16] = simplePathChain(AutoTarget.AUTO_A_END.idx,    AutoTarget.SCORING.idx,       0.8);

        pathChains[17] = simplePathChain(AutoTarget.AUTO_GATE_START.idx, AutoTarget.AUTO_GATE_END.idx, 0.8);
        pathChains[18] = simplePathChain(AutoTarget.AUTO_GATE_END.idx,   AutoTarget.SCORING.idx,       0.8);
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
