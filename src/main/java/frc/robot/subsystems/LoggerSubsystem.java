package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; 
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class LoggerSubsystem extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private BufferedWriter writer = null;
    private boolean isRecording = false;
    private double startTime = 0;

    public LoggerSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public void startRecording(String filename) {
        if (isRecording)
            return;

        String path = "/home/lvuser/logs/" + filename + ".csv";
        try {
            writer = new BufferedWriter(new FileWriter(path));
            writer.write("time,xSpeed,ySpeed,omega\n"); 
            startTime = Timer.getFPGATimestamp();
            isRecording = true;
            SmartDashboard.putString("Logger/Status", "Recording: " + filename);
        } catch (IOException e) {
            SmartDashboard.putString("Logger/Error", e.getMessage());
        }
    }

    public void stopRecording() {
        if (!isRecording)
            return;

        isRecording = false;
        try {
            if (writer != null) {
                writer.close();
                writer = null;
            }
            SmartDashboard.putString("Logger/Status", "Stopped");
        } catch (IOException e) {
            SmartDashboard.putString("Logger/Error", "Close failed: " + e.getMessage());
        }
    }

    @Override
    public void periodic() {
        if (!isRecording || writer == null)
            return;

        double now = Timer.getFPGATimestamp() - startTime;

        ChassisSpeeds speeds = swerve.getFieldVelocity(); 

        try {
            writer.write(String.format("%.3f,%.3f,%.3f,%.3f\n",
                    now,
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond));
        } catch (IOException e) {
        }
    }

    public boolean isRecording() {
        return isRecording;
    }
}