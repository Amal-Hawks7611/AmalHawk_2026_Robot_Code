package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ShooterSubsystem;

public class stage3 extends Command {
    public final ShooterSubsystem shooterSubsystem;
    public final ColorSensors colorSensors;
    public static Timer timer2;
    public static Boolean isShot;
    public stage3(ShooterSubsystem shooterSubsystem, ColorSensors colorSensors) {
        this.shooterSubsystem = shooterSubsystem;
        this.colorSensors = colorSensors;
        timer2 = new Timer();
        addRequirements(shooterSubsystem, colorSensors);
    }

    @Override
    public void initialize() {
        System.out.println("Fuel STAGE3 Ininialized");
        shooterSubsystem.setShooting(true);
        timer2.reset();
        isShot = false;
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_SHOOTER_ENABLED){
            shooterSubsystem.Shoot(Shooter.STAGE3_SPEED);
            if (colorSensors.isBallDetected())
            {
                timer2.start();
            }
            else if ( !colorSensors.isBallDetected()){
            if (timer2.hasElapsed(2)){
                this.end(false);
            }else{
                shooterSubsystem.Shoot(Shooter.STAGE3_SPEED);}
           if(colorSensors.isBallDetected() && timer2.get() > 0){
            timer2.restart();
           }
        }
        } 
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Fuel STAGE3 Ended");
        shooterSubsystem.stopMotors();
        shooterSubsystem.setShooting(false);
        timer2.reset();
        timer2.stop();
    }

    @Override
    public boolean isFinished() {
        return !shooterSubsystem.isShooting();
    }
}
