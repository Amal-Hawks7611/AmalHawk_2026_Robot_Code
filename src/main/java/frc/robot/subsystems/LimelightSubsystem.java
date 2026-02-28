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

    private boolean isAligned = false;

    public Command alignToPose(
            SwerveSubsystem swerve,
            double targetX,
            double targetZ,
            double targetYaw) {

        return new FunctionalCommand(

                () -> isAligned = false,

                () -> {

                    if (!LimelightHelpers.getTV(OI.LL_NAME)) {
                        swerve.drive(new Translation2d(0, 0), 0, false);
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

                    isAligned =
                            Math.abs(yawError) <= YAW_TOLERANCE &&
                            Math.abs(xError) <= X_TOLERANCE &&
                            Math.abs(zError) <= Z_TOLERANCE;

                    SmartDashboard.putNumber("LL/X", currentX);
                    SmartDashboard.putNumber("LL/Z", currentZ);
                    SmartDashboard.putNumber("LL/Yaw", currentYaw);
                },

                interrupted -> swerve.drive(new Translation2d(0, 0), 0, false),

                () -> isAligned,

                swerve);
    }

    @Override
    public void periodic() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);
        if (pose.length < 6)
            return;

        SmartDashboard.putNumber("LL/X", pose[0]);
        SmartDashboard.putNumber("LL/Z", pose[2]);
        SmartDashboard.putNumber("LL/Yaw", pose[5]);
    }
}