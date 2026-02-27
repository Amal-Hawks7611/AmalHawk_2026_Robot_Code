package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoggerSubsystem;

public class StopRecordingCommand extends Command {
    private final LoggerSubsystem logger;

    public StopRecordingCommand(LoggerSubsystem logger) {
        this.logger = logger;
        System.out.println("Recording Stopped");
        addRequirements(logger);
    }

    @Override
    public void initialize() {
        logger.stopRecording();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}