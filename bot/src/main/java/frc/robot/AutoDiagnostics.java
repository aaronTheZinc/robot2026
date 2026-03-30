package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * NetworkTables + SignalLogger diagnostics for autonomous and PathPlanner path following.
 * Topics live under {@code Auto/} and {@code PathFollower/} for dashboards and AdvantageScope.
 */
public final class AutoDiagnostics {
    private static final NetworkTableInstance kNt = NetworkTableInstance.getDefault();
    private static final NetworkTable kAuto = kNt.getTable("Auto");
    private static final NetworkTable kPath = kNt.getTable("PathFollower");

    private static final StringPublisher kRegisteredNamed =
            kAuto.getStringTopic("registeredNamedCommands").publish();
    /** Raw {@link SendableChooser#getSelected()} at auto init (may be null). */
    private static final StringPublisher kChooserCommandName =
            kAuto.getStringTopic("chooserCommandName").publish();
    private static final StringPublisher kChooserCommandClass =
            kAuto.getStringTopic("chooserCommandClass").publish();
    /** Command actually scheduled for autonomous (after null-safe fallback). */
    private static final StringPublisher kResolvedName =
            kAuto.getStringTopic("resolvedAutoCommandName").publish();
    private static final StringPublisher kResolvedClass =
            kAuto.getStringTopic("resolvedAutoCommandClass").publish();
    private static final BooleanPublisher kAutoUsedFallback =
            kAuto.getBooleanTopic("autoUsedChooserFallback").publish();
    /** Backward-compatible aliases for dashboards; same as resolved command. */
    private static final StringPublisher kSelectedName =
            kAuto.getStringTopic("selectedCommandName").publish();
    private static final StringPublisher kSelectedClass =
            kAuto.getStringTopic("selectedCommandClass").publish();
    /** Set from {@link Robot#autonomousInit} when default drive is canceled for auto. */
    private static final BooleanPublisher kDefaultDriveCanceledForAuto =
            kAuto.getBooleanTopic("defaultDriveCanceledForAuto").publish();
    private static final StringPublisher kDriveReqCmd =
            kAuto.getStringTopic("driveSubsystemCommand").publish();
    private static final StringPublisher kLastEvent = kAuto.getStringTopic("lastEvent").publish();
    private static final IntegerPublisher kEventIndex = kAuto.getIntegerTopic("eventIndex").publish();
    private static final BooleanPublisher kAutoCommandScheduled =
            kAuto.getBooleanTopic("autonomousCommandScheduled").publish();
    private static final BooleanPublisher kDefaultDriveScheduled =
            kAuto.getBooleanTopic("defaultDriveScheduled").publish();

    /**
     * When false (default), high-rate debug streams to NetworkTables and SignalLogger are suppressed.
     * Writable from SmartDashboard, NT clients, and {@link #setDebugTelemetryEnabled}.
     */
    private static final BooleanPublisher kDebugTelemetryEnabledPub =
            kAuto.getBooleanTopic("debugTelemetryEnabled").publish();
    private static final BooleanSubscriber kDebugTelemetryEnabledSub =
            kAuto.getBooleanTopic("debugTelemetryEnabled").subscribe(false);

    private static final String kSmartDashboardDebugTelemetryKey = "Debug/Verbose NT Telemetry";

    private static volatile boolean s_debugTelemetryEnabled = false;

    private static final BooleanPublisher kAlliancePresent =
            kPath.getBooleanTopic("alliancePresent").publish();
    private static final BooleanPublisher kFlipPathForRed =
            kPath.getBooleanTopic("flipPathForRed").publish();
    private static final DoublePublisher kCfgMassKg = kPath.getDoubleTopic("configMassKg").publish();
    private static final DoublePublisher kCfgMoi = kPath.getDoubleTopic("configMoiKgM2").publish();
    private static final DoublePublisher kCfgCof = kPath.getDoubleTopic("configWheelCOF").publish();
    private static final DoublePublisher kCfgTransKp = kPath.getDoubleTopic("configTransKp").publish();
    private static final DoublePublisher kCfgRotKp = kPath.getDoubleTopic("configRotKp").publish();

    private static final StructPublisher<ChassisSpeeds> kCmdSpeeds =
            kPath.getStructTopic("commandedSpeeds", ChassisSpeeds.struct).publish();
    private static final DoublePublisher kCmdVx = kPath.getDoubleTopic("commandedVx").publish();
    private static final DoublePublisher kCmdVy = kPath.getDoubleTopic("commandedVy").publish();
    private static final DoublePublisher kCmdOmega = kPath.getDoubleTopic("commandedOmega").publish();
    private static final DoublePublisher kFfMag = kPath.getDoubleTopic("feedforwardForceSumN").publish();
    /** Increments every time PathPlanner's AutoBuilder output consumer runs (detect "follower never runs"). */
    private static final IntegerPublisher kOutputInvokes =
            kPath.getIntegerTopic("outputInvokeCount").publish();
    private static final BooleanPublisher kPathOutputRecent =
            kPath.getBooleanTopic("pathOutputRecent").publish();

    private static final DoublePublisher kErrVx = kPath.getDoubleTopic("vxError").publish();
    private static final DoublePublisher kErrVy = kPath.getDoubleTopic("vyError").publish();
    private static final DoublePublisher kErrOmega = kPath.getDoubleTopic("omegaError").publish();

    /** Allow telemetry that runs on a different thread than the scheduler to still see "recent" output. */
    private static final long kPathOutputStaleNs = 400_000_000L; // 400 ms

    private static int s_outputInvokes;

    private static volatile ChassisSpeeds s_lastPathCommanded = new ChassisSpeeds();
    private static volatile long s_lastPathOutputNs;

    private static int s_eventSeq;

    static {
        kDebugTelemetryEnabledPub.setDefault(false);
    }

    private AutoDiagnostics() {}

    /** @return whether high-rate debug telemetry to NT and SignalLogger is enabled */
    public static boolean isDebugTelemetryEnabled() {
        return s_debugTelemetryEnabled;
    }

    /**
     * Sets high-rate debug telemetry. Updates {@code Auto/debugTelemetryEnabled} and SmartDashboard mirror.
     */
    public static void setDebugTelemetryEnabled(boolean enabled) {
        s_debugTelemetryEnabled = enabled;
        kDebugTelemetryEnabledPub.set(enabled);
        SmartDashboard.putBoolean(kSmartDashboardDebugTelemetryKey, enabled);
    }

    /**
     * Merges SmartDashboard and NetworkTables sources for the debug toggle. Call from {@link Robot#robotPeriodic}.
     */
    public static void periodicDebugTelemetrySync() {
        boolean sd = SmartDashboard.getBoolean(kSmartDashboardDebugTelemetryKey, s_debugTelemetryEnabled);
        boolean nt = kDebugTelemetryEnabledSub.get();
        if (sd != s_debugTelemetryEnabled) {
            s_debugTelemetryEnabled = sd;
            kDebugTelemetryEnabledPub.set(sd);
        } else if (nt != s_debugTelemetryEnabled) {
            s_debugTelemetryEnabled = nt;
            SmartDashboard.putBoolean(kSmartDashboardDebugTelemetryKey, nt);
        }
    }

    public static void publishRegisteredNamedCommands(String csv) {
        kRegisteredNamed.set(csv);
    }

    public static void publishChooserAutoSelection(Command cmd) {
        if (cmd == null) {
            kChooserCommandName.set("(null)");
            kChooserCommandClass.set("(null)");
        } else {
            kChooserCommandName.set(cmd.getName());
            kChooserCommandClass.set(cmd.getClass().getName());
        }
    }

    /**
     * Publishes the command that will be scheduled in autonomous (chooser selection or fallback).
     * Also updates {@code selectedCommandName} / {@code selectedCommandClass} for older dashboards.
     */
    public static void publishResolvedAutonomous(Command resolved, boolean usedChooserFallback) {
        kAutoUsedFallback.set(usedChooserFallback);
        if (resolved == null) {
            kResolvedName.set("(null)");
            kResolvedClass.set("(null)");
            kSelectedName.set("(null)");
            kSelectedClass.set("(null)");
            return;
        }
        String name = resolved.getName();
        String cls = resolved.getClass().getName();
        kResolvedName.set(name);
        kResolvedClass.set(cls);
        kSelectedName.set(name);
        kSelectedClass.set(cls);
    }

    /** @deprecated Use {@link #publishChooserAutoSelection} and {@link #publishResolvedAutonomous}. */
    @Deprecated
    public static void publishSelectedAuto(Command cmd) {
        publishChooserAutoSelection(cmd);
        publishResolvedAutonomous(cmd, false);
    }

    public static void publishDefaultDriveCanceledForAuto(boolean canceled) {
        kDefaultDriveCanceledForAuto.set(canceled);
    }

    public static void publishActiveDriveCommand(Command requiringDrive) {
        if (requiringDrive == null) {
            kDriveReqCmd.set("(none)");
        } else {
            kDriveReqCmd.set(requiringDrive.getName() + " [" + requiringDrive.getClass().getSimpleName() + "]");
        }
    }

    /**
     * After {@link CommandScheduler#run()}: whether the main auto command is still scheduled and whether
     * the drivetrain default (teleop) command got scheduled — both true usually means joystick drive is
     * winning over PathPlanner for the drivetrain.
     */
    public static void publishAutonomousSchedulerSnap(
            Command autonomousCommand, CommandScheduler scheduler, Subsystem drivetrain) {
        boolean autoOn = autonomousCommand != null && scheduler.isScheduled(autonomousCommand);
        Command def = scheduler.getDefaultCommand(drivetrain);
        boolean defOn = def != null && scheduler.isScheduled(def);
        kAutoCommandScheduled.set(autoOn);
        kDefaultDriveScheduled.set(defOn);
    }

    public static void logEvent(String message) {
        s_eventSeq++;
        kEventIndex.set(s_eventSeq);
        kLastEvent.set(message);
        SignalLogger.writeString("Auto/lastEvent", message);
        SignalLogger.writeDouble("Auto/eventIndex", s_eventSeq, "");
    }

    public static void logPoseReset(Pose2d pose) {
        String msg =
                String.format(
                        "resetPose x=%.3f y=%.3f deg=%.2f",
                        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        logEvent(msg);
    }

    public static void publishPathPlannerRobotConfig(
            double massKg,
            double moiKgM2,
            double wheelCof,
            double transKp,
            double rotKp) {
        kCfgMassKg.set(massKg);
        kCfgMoi.set(moiKgM2);
        kCfgCof.set(wheelCof);
        kCfgTransKp.set(transKp);
        kCfgRotKp.set(rotKp);
    }

    public static void updateAllianceAndFlip(boolean flipPathForRed) {
        boolean present = DriverStation.getAlliance().isPresent();
        kAlliancePresent.set(present);
        kFlipPathForRed.set(flipPathForRed);
    }

    public static void recordPathPlannerOutput(ChassisSpeeds discretized, double feedforwardForceSumN) {
        s_outputInvokes++;
        s_lastPathCommanded = discretized;
        s_lastPathOutputNs = System.nanoTime();

        if (!isDebugTelemetryEnabled()) {
            return;
        }

        kOutputInvokes.set(s_outputInvokes);
        SignalLogger.writeDouble("PathFollower/outputInvokeCount", s_outputInvokes, "");

        kCmdSpeeds.set(discretized);
        kCmdVx.set(discretized.vxMetersPerSecond);
        kCmdVy.set(discretized.vyMetersPerSecond);
        kCmdOmega.set(discretized.omegaRadiansPerSecond);
        kFfMag.set(feedforwardForceSumN);
        kPathOutputRecent.set(true);

        SignalLogger.writeStruct("PathFollower/commandedSpeeds", ChassisSpeeds.struct, discretized);
        SignalLogger.writeDouble("PathFollower/commandedVx", discretized.vxMetersPerSecond, "mps");
        SignalLogger.writeDouble("PathFollower/commandedVy", discretized.vyMetersPerSecond, "mps");
        SignalLogger.writeDouble("PathFollower/commandedOmega", discretized.omegaRadiansPerSecond, "radps");
        SignalLogger.writeDouble("PathFollower/feedforwardForceSumN", feedforwardForceSumN, "N");
    }

    public static void publishDriveErrorVsPathFollower(ChassisSpeeds measured) {
        if (!isDebugTelemetryEnabled()) {
            return;
        }

        long age = System.nanoTime() - s_lastPathOutputNs;
        boolean recent = age >= 0 && age < kPathOutputStaleNs;
        kPathOutputRecent.set(recent);
        if (!recent) {
            kErrVx.set(0);
            kErrVy.set(0);
            kErrOmega.set(0);
            return;
        }
        ChassisSpeeds cmd = s_lastPathCommanded;
        double evx = cmd.vxMetersPerSecond - measured.vxMetersPerSecond;
        double evy = cmd.vyMetersPerSecond - measured.vyMetersPerSecond;
        double ew = cmd.omegaRadiansPerSecond - measured.omegaRadiansPerSecond;
        kErrVx.set(evx);
        kErrVy.set(evy);
        kErrOmega.set(ew);
        SignalLogger.writeDouble("PathFollower/vxError", evx, "mps");
        SignalLogger.writeDouble("PathFollower/vyError", evy, "mps");
        SignalLogger.writeDouble("PathFollower/omegaError", ew, "radps");
    }

    public static void reportPathLoadFailed(String pathName, String err) {
        logEvent("path load FAILED '" + pathName + "': " + err);
    }
}
