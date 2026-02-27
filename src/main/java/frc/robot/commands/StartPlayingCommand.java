package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LogPlayerSubsystem;

public class StartPlayingCommand extends Command {
    private final LogPlayerSubsystem player;

    public StartPlayingCommand(LogPlayerSubsystem player) {
        this.player = player;
        System.out.println("Playing Started");
        addRequirements(player);
    }

    @Override
    public void initialize() {
        player.startPlaying();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}