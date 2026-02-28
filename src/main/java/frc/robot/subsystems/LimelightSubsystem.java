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

    // Toleranslar
    private static final double YAW_TOLERANCE = 0.8;
    private static final double X_TOLERANCE = 0.15;
    private static final double Z_TOLERANCE = 0.1;

    // Sabit hızlar
    private static final double ROT_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.61;
    private static final double FORWARD_SPEED = 0.75;

    private int stage = 0;
    public boolean isAligned = false;

    public Command autoAlignAndApproach(
            SwerveSubsystem swerve,
            double targetDistance) {

        return new FunctionalCommand(

                () -> {
                    stage = 0;
                    isAligned = false;
                },

                () -> {

                    if (!LimelightHelpers.getTV(OI.LL_NAME)) {
                        swerve.drive(new Translation2d(0, 0), 0, false);
                        SmartDashboard.putString("LL Status", "No Target");
                        return;
                    }

                    double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);
                    if (pose.length < 6)
                        return;

                    double x = pose[0];
                    double z = pose[2];
                    double yaw = pose[5];

                    SmartDashboard.putNumber("LL/X", x);
                    SmartDashboard.putNumber("LL/Z", z);
                    SmartDashboard.putNumber("LL/Yaw", yaw);
                    SmartDashboard.putNumber("LL/Stage", stage);

                    if (stage == 0) {

                        if (Math.abs(yaw) > YAW_TOLERANCE) {

                            double rot = yaw > 0 ? ROT_SPEED : -ROT_SPEED;

                            swerve.drive(
                                    new Translation2d(0, 0),
                                    rot,
                                    false);
                            return;
                        }

                        stage = 1; 
                    }
                    if (stage == 1) {

                        if (Math.abs(x) > X_TOLERANCE) {

                            double strafe = x > 0 ? -STRAFE_SPEED : STRAFE_SPEED;

                            swerve.drive(
                                    new Translation2d(0, strafe),
                                    0,
                                    false);
                            return;
                        }

                        stage = 2;
                    }
                    if (stage == 2) {

                        double zError = z - targetDistance;

                        if (Math.abs(zError) > Z_TOLERANCE) {

                            double forward = zError > 0 ? FORWARD_SPEED : -FORWARD_SPEED;

                            swerve.drive(
                                    new Translation2d(forward, 0),
                                    0,
                                    false);
                            return;
                        }

                        stage = 3;
                    }
                    if (stage == 3) {
                        swerve.drive(new Translation2d(0, 0), 0, false);
                        isAligned = true;                     
                    }
                },

                interrupted -> {
                    swerve.drive(new Translation2d(0, 0), 0, false);
                    isAligned = true;
                },

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