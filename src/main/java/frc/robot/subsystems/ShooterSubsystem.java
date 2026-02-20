package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX leaderMotor;
    public TalonFX slaveMotor;
    public StatusSignal<Angle> slaveMotorPosition;
    private StatusSignal<Angle> leaderMotorPosition;
    public RobotContainer container;
    public boolean isShooting = false;

    public Timer timer = new Timer();

    public ShooterSubsystem(RobotContainer container) {
        leaderMotor = new TalonFX(Shooter.SHOOTER_LEADER_PORT, new CANBus("arch"));
        slaveMotor = new TalonFX(Shooter.SHOOTER_SLAVE_PORT, new CANBus("arch"));
        leaderMotorPosition = leaderMotor.getPosition();
        slaveMotorPosition = slaveMotor.getPosition();

        this.container = container;
        resetEncoders();
    }

    public double getLeaderMotorEncoder() {
        return leaderMotorPosition.refresh().getValueAsDouble();
    }

    public double getSlaveMotorEncoder() {
        return slaveMotorPosition.refresh().getValueAsDouble();
    }

    public void resetEncoders() {
        leaderMotor.setPosition(0);
        slaveMotor.setPosition(0);
    }

    public void Shoot(double speed) {
        leaderMotor.set(speed);
        slaveMotor.set(speed);
    }
    public boolean isShooting(){
        return isShooting;
    }
    public void setShooting(boolean status){
        isShooting = status;
    }
    public void stopMotors(){
        leaderMotor.set(0);
        slaveMotor.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isShooting", isShooting);
        SmartDashboard.putNumber("Feeder Leader Motor Value", getLeaderMotorEncoder());
        SmartDashboard.putNumber("Feeder Slave  Motor Value", getSlaveMotorEncoder());
        SmartDashboard.putNumber("FeederSpeed", leaderMotor.get());
    }
}
