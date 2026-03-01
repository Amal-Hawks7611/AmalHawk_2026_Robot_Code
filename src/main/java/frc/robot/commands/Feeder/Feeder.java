package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledParts;
import frc.robot.subsystems.FeederSubsystem;

public class Feeder extends Command {
    public final FeederSubsystem feederSubsystem;
    public final RobotContainer container;
    public static Timer timer;
    public Feeder(FeederSubsystem feederSubsystem, RobotContainer container) {
        this.feederSubsystem = feederSubsystem;
        this.container = container;
        timer = new Timer();
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.setFeeding(true);
        System.out.println("Fuel Feeding Ininialized");  
        timer.restart();
    }

    @Override
    public void execute() {
        
            feederSubsystem.Feed();
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Fuel Feeding Ended");
        feederSubsystem.stopMotors();
        feederSubsystem.setFeeding(false);
        timer.reset();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !feederSubsystem.isFeeding();
    }
}
