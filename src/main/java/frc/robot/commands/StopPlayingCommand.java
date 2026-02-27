package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LogPlayerSubsystem;

public class StopPlayingCommand extends Command {
    private final LogPlayerSubsystem player;

    public StopPlayingCommand(LogPlayerSubsystem player) {
        this.player = player;
        System.out.println("Playing Stopped");
        addRequirements(player);
    }

    @Override
    public void initialize() {
        player.stopPlaying();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}