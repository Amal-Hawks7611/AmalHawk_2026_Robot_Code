package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OI;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimelightSubsystem extends SubsystemBase {
    private final PIDController xPID = new PIDController(0.6, 0, 0);
    private final PIDController zPID = new PIDController(0.5, 0, 0);
    private final PIDController yawPID = new PIDController(0.02, 0, 0);

    public LimelightSubsystem() {
        xPID.setTolerance(0.05);
        zPID.setTolerance(0.05);
        yawPID.setTolerance(1.0);
        yawPID.enableContinuousInput(-180, 180);
    }

    public Command autoAlignAndApproach(
            SwerveSubsystem swerve,
            double targetDistance) {

        return new FunctionalCommand(

                () -> {
                    xPID.reset();
                    zPID.reset();
                    yawPID.reset();
                },

                () -> {

                    if (!LimelightHelpers.getTV(OI.LL_NAME)) {
                        swerve.drive(new Translation2d(0, 0), 0, false);
                        SmartDashboard.putString("LL Status", "No Target");
                        return;
                    }

                    double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);

                    double currentDistance = pose[2];

                    SmartDashboard.putNumber("LimelightPlus/LL Distance (Z)", currentDistance);
                    SmartDashboard.putNumber("LimelightPlus/LL X Error", pose[0]);
                    SmartDashboard.putNumber("LimelightPlus/LL Yaw Error", pose[5]);

                    double xError = pose[0];
                    double zError = pose[2] - targetDistance;
                    double yawError = pose[5];

                    double xSpeed = xPID.calculate(xError, 0);
                    double zSpeed = zPID.calculate(zError, 0);
                    double rotSpeed = yawPID.calculate(yawError, 0);

                    SmartDashboard.putNumber("LimelightPlus/xSpeed", xSpeed);
                    SmartDashboard.putNumber("LimelightPlus/rotSpeed", rotSpeed);
                    SmartDashboard.putNumber("LimelightPlus/zSpeed", zSpeed);
                    // swerve.drive(
                    // new Translation2d(zSpeed, xSpeed),
                    // rotSpeed,
                    // false);
                },

                interrupted -> {
                    swerve.drive(
                            new Translation2d(0, 0),
                            0,
                            false);
                },

                () -> xPID.atSetpoint() &&
                        zPID.atSetpoint() &&
                        yawPID.atSetpoint(),

                swerve);
    }

    @Override
    public void periodic() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(OI.LL_NAME);

        double currentDistance = pose[2];

        SmartDashboard.putNumber("LimelightPlus/LL Distance (Z)", currentDistance);
        SmartDashboard.putNumber("LimelightPlus/LL X Error", pose[0]);
        SmartDashboard.putNumber("LimelightPlus/LL Yaw Error", pose[5]);

        double xError = pose[0];
        double zError = pose[2] - 10;
        double yawError = pose[5];

        double xSpeed = xPID.calculate(xError, 0);
        double zSpeed = zPID.calculate(zError, 0);
        double rotSpeed = yawPID.calculate(yawError, 0);

        SmartDashboard.putNumber("LimelightPlus/xSpeed", xSpeed);
        SmartDashboard.putNumber("LimelightPlus/rotSpeed", rotSpeed);
        SmartDashboard.putNumber("LimelightPlus/zSpeed", zSpeed);
    }
}
