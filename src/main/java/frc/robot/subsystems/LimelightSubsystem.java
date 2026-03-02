package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OI;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimelightSubsystem extends SubsystemBase {

    private static final double YAW_TOLERANCE = 0.8;
    private static final double X_TOLERANCE = 0.15;
    private static final double Z_TOLERANCE = 0.1;

    private static final double ROT_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.6;
    private static final double FORWARD_SPEED = 0.75;
    private static final double GRAVITY = 9.81;
    private static final double SHOOTER_ANGLE_DEG = 69.0;
    private static final double TARGET_HEIGHT = 1.42;

    public enum AlignmentTarget {
        CENTER,
        LEFT,
        RIGHT,
        NONE
    }

    private AlignmentTarget currentTarget = AlignmentTarget.NONE;

    private boolean isAligned = false;

    private static double x = LimelightSubsystem.getZ();

    double velocity_required = Math.sqrt(
            (GRAVITY * x * x) / (2 * Math.cos(SHOOTER_ANGLE_DEG) * (x * Math.tan(SHOOTER_ANGLE_DEG) - TARGET_HEIGHT)));

    public Command alignToPose(
            SwerveSubsystem swerve,
            double targetX,
            double targetZ,
            double targetYaw,
            AlignmentTarget targetType) {

        return new FunctionalCommand(

                () -> {
                    isAligned = false;
                    currentTarget = targetType;
                },

                () -> {

                    if (!LimelightHelpers.getTV(OI.LL_NAME)) {
                        swerve.drive(new Translation2d(0, 0), 0, false);
                        isAligned = false;
                        currentTarget = AlignmentTarget.NONE;
                        return;
                    }
                    double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);
                    if (pose.length < 6)
                        return;

                    double currentX = pose[0];
                    double currentZ = pose[2];
                    double currentYaw = pose[5];

                    double yawError = currentYaw - targetYaw;
                    double xError = currentX - targetX;
                    double zError = currentZ - targetZ;

                    double rot = 0;
                    double strafe = 0;
                    double forward = 0;

                    if (Math.abs(yawError) > YAW_TOLERANCE)
                        rot = yawError > 0 ? ROT_SPEED : -ROT_SPEED;

                    if (Math.abs(xError) > X_TOLERANCE)
                        strafe = xError > 0 ? -STRAFE_SPEED : STRAFE_SPEED;

                    if (Math.abs(zError) > Z_TOLERANCE)
                        forward = zError > 0 ? FORWARD_SPEED : -FORWARD_SPEED;

                    swerve.drive(
                            new Translation2d(forward, strafe),
                            rot,
                            false);

                    isAligned = Math.abs(yawError) <= YAW_TOLERANCE &&
                            Math.abs(xError) <= X_TOLERANCE &&
                            Math.abs(zError) <= Z_TOLERANCE;
                },

                interrupted -> isAligned = true,

                () -> isAligned,

                swerve);
    }

    public static double getZ() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);
        if (pose.length < 6)
            return 0;
        return pose[2];
    }

    public boolean isAligned() {
        return isAligned;
    }

    public AlignmentTarget getCurrentTarget() {
        return currentTarget;
    }

    @Override
    public void periodic() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);
        if (pose.length < 6)
            return;
        double currentX = pose[0];
        double currentZ = pose[2];
        double currentYaw = pose[5];

        SmartDashboard.putNumber("LL/TZ", currentZ);
        SmartDashboard.putNumber("LL/TX", currentX);
        SmartDashboard.putNumber("LL/TYAW", currentYaw);
    }
}