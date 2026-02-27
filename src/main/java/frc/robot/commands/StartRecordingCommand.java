package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoggerSubsystem;

public class StartRecordingCommand extends Command {
    private final LoggerSubsystem logger;
    private final String filename;

    public StartRecordingCommand(LoggerSubsystem logger, String filename) {
        this.logger = logger;
        this.filename = filename;
        System.out.println("Recording Started");
        addRequirements(logger);
    }

    @Override
    public void initialize() {
        logger.startRecording(filename);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}