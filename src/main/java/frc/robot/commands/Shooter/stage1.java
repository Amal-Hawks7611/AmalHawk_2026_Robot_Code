package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;

public class stage1 extends Command {
    public final ShooterSubsystem shooterSubsystem;
    public static Timer timer2;
    public stage1(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        timer2 = new Timer();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Fuel STAGE1 Ininialized");
        shooterSubsystem.setShooting(true);
        timer2.restart();
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_SHOOTER_ENABLED && timer2.hasElapsed(Shooter.SHOOTER_TIME-Feeder.FEEDER_TIME) && !timer2.hasElapsed(Shooter.SHOOTER_TIME)){
            shooterSubsystem.Shoot(Shooter.STAGE1_SPEED);
        } else if ( timer2.hasElapsed(Shooter.SHOOTER_TIME)){
            this.end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Fuel STAGE1 Ended");
        shooterSubsystem.stopMotors();
        shooterSubsystem.setShooting(false);
        timer2.reset();
        timer2.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
