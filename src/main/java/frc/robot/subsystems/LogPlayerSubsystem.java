package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class LogPlayerSubsystem extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private List<LogEntry> logEntries = new ArrayList<>();
    private boolean isPlaying = false;
    private int currentIndex = 0;
    private double playStartTime = 0;

    private static class LogEntry {
        double time;
        double xSpeed;
        double ySpeed;
        double omega;
    }

    public LogPlayerSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public boolean loadLog(String filename) {
        String path = "/home/lvuser/logs/" + filename + ".csv";
        logEntries.clear();
        currentIndex = 0;

        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            reader.readLine();

            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 4) {
                    LogEntry entry = new LogEntry();
                    entry.time = Double.parseDouble(parts[0]);
                    entry.xSpeed = Double.parseDouble(parts[1]);
                    entry.ySpeed = Double.parseDouble(parts[2]);
                    entry.omega = Double.parseDouble(parts[3]);
                    logEntries.add(entry);
                }
            }
            SmartDashboard.putString("LogPlayer/Status", "Loaded: " + logEntries.size() + " entries");
            return true;
        } catch (IOException | NumberFormatException e) {
            SmartDashboard.putString("LogPlayer/Error", e.getMessage());
            return false;
        }
    }

    public void startPlaying() {
        if (logEntries.isEmpty() || isPlaying)
            return;
        currentIndex = 0;
        playStartTime = Timer.getFPGATimestamp();
        isPlaying = true;
        SmartDashboard.putString("LogPlayer/Status", "Playing");
    }

    public void stopPlaying() {
        isPlaying = false;
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("LogPlayer/Status", "Stopped");
    }

    @Override
    public void periodic() {
        if (!isPlaying || logEntries.isEmpty())
            return;

        double elapsed = Timer.getFPGATimestamp() - playStartTime;

        while (currentIndex < logEntries.size() &&
                elapsed >= logEntries.get(currentIndex).time) {

            LogEntry entry = logEntries.get(currentIndex);
            swerve.drive(new ChassisSpeeds(entry.xSpeed, entry.ySpeed, entry.omega));
            currentIndex++;
        }

        if (currentIndex >= logEntries.size()) {
            stopPlaying();
        }
    }

    public boolean isPlaying() {
        return isPlaying;
    }
}