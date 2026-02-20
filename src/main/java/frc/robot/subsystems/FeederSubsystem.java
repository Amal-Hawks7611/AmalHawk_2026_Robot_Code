package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.Feeder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class FeederSubsystem extends SubsystemBase {
    public TalonFX leaderMotor;
    private StatusSignal<Angle> leaderMotorPosition;
    public RobotContainer container;
    public boolean isFeeding = false;

    public Timer timer = new Timer();

    public FeederSubsystem(RobotContainer container) {
        leaderMotor = new TalonFX(Feeder.FEEDER_LEADER_MOTOR_PORT, new CANBus("arch"));
        leaderMotorPosition = leaderMotor.getPosition();
        this.container = container;
        resetEncoders();
    }

    public double getLeaderMotorEncoder() {
        return leaderMotorPosition.refresh().getValueAsDouble();
    }

    public void resetEncoders() {
        leaderMotor.setPosition(0);
    }
    public boolean isFeeding(){
        return isFeeding;
    }
    public void setFeeding(boolean status){
        isFeeding = status;
    }
    public void stopMotors(){
        leaderMotor.set(0);
    }
    public void Feed() {
                leaderMotor.set(Feeder.FEEDER_SPEED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ısFeeding", isFeeding);
        SmartDashboard.putNumber("Feeder Leader Motor Value", getLeaderMotorEncoder());
        SmartDashboard.putNumber("FeederSpeed", leaderMotor.get());
    }
}
