package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

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
    private static final StringPublisher kSelectedName =
            kAuto.getStringTopic("selectedCommandName").publish();
    private static final StringPublisher kSelectedClass =
            kAuto.getStringTopic("selectedCommandClass").publish();
    private static final StringPublisher kDriveReqCmd =
            kAuto.getStringTopic("driveSubsystemCommand").publish();
    private static final StringPublisher kLastEvent = kAuto.getStringTopic("lastEvent").publish();
    private static final IntegerPublisher kEventIndex = kAuto.getIntegerTopic("eventIndex").publish();

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
    private static final BooleanPublisher kPathOutputRecent =
            kPath.getBooleanTopic("pathOutputRecent").publish();

    private static final DoublePublisher kErrVx = kPath.getDoubleTopic("vxError").publish();
    private static final DoublePublisher kErrVy = kPath.getDoubleTopic("vyError").publish();
    private static final DoublePublisher kErrOmega = kPath.getDoubleTopic("omegaError").publish();

    private static final long kPathOutputStaleNs = 150_000_000L; // 150 ms

    private static volatile ChassisSpeeds s_lastPathCommanded = new ChassisSpeeds();
    private static volatile long s_lastPathOutputNs;

    private static int s_eventSeq;

    private AutoDiagnostics() {}

    public static void publishRegisteredNamedCommands(String csv) {
        kRegisteredNamed.set(csv);
    }

    public static void publishSelectedAuto(Command cmd) {
        if (cmd == null) {
            kSelectedName.set("(null)");
            kSelectedClass.set("(null)");
        } else {
            kSelectedName.set(cmd.getName());
            kSelectedClass.set(cmd.getClass().getName());
        }
    }

    public static void publishActiveDriveCommand(Command requiringDrive) {
        if (requiringDrive == null) {
            kDriveReqCmd.set("(none)");
        } else {
            kDriveReqCmd.set(requiringDrive.getName() + " [" + requiringDrive.getClass().getSimpleName() + "]");
        }
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
        s_lastPathCommanded = discretized;
        s_lastPathOutputNs = System.nanoTime();
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
