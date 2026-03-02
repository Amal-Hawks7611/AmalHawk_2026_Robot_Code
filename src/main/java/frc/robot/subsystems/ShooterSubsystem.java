package frc.robot.subsystems;

import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX leaderMotor;
    public TalonFX slaveMotor;
    public StatusSignal<Angle> slaveMotorPosition;
    private StatusSignal<Angle> leaderMotorPosition;
    public boolean isShooting = false;
    public Timer timer = new Timer();
    public final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.45;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leaderMotor = new TalonFX(Shooter.SHOOTER_LEADER_PORT, new CANBus("arch"));
        slaveMotor = new TalonFX(Shooter.SHOOTER_SLAVE_PORT, new CANBus("arch"));
        leaderMotor.getDutyCycle().setUpdateFrequency(50);
        slaveMotor.getDutyCycle().setUpdateFrequency(50);
        leaderMotor.getConfigurator().apply(config);
        leaderMotorPosition = leaderMotor.getPosition();
        slaveMotorPosition = slaveMotor.getPosition();
        slaveMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
      
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
        leaderMotor.setControl(velocityVoltage.withVelocity(speed));
    }
    public void ShootBack(){
        leaderMotor.setControl(velocityVoltage.withVelocity(-Shooter.STAGE2_SPEED));
    }
    public boolean isShooting(){
        return isShooting;
    }
    public void setShooting(boolean status){
        isShooting = status;
    }
    public void stopMotors(){
       leaderMotor.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isShooting", isShooting);
        SmartDashboard.putNumber("ShooterSpeed", leaderMotor.get());
        SmartDashboard.putNumber("GetZ", LimelightSubsystem.getZ());
    }
}
