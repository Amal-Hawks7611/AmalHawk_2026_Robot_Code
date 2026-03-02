package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.Feeder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederSubsystem extends SubsystemBase {
    public TalonFX leaderMotor;
    private StatusSignal<Angle> leaderMotorPosition;
    public RobotContainer container;
    public boolean isFeeding = false;
    public Timer timer = new Timer();
    public final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public FeederSubsystem(RobotContainer container) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.45;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leaderMotor = new TalonFX(Feeder.FEEDER_LEADER_MOTOR_PORT, new CANBus("arch"));
        leaderMotor.getDutyCycle().setUpdateFrequency(50);
        leaderMotor.getConfigurator().apply(config);
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

    public boolean isFeeding() {
        return isFeeding;
    }

    public void setFeeding(boolean status) {
        isFeeding = status;
    }

    public void stopMotors() {
        leaderMotor.set(0);
    }

    public void Feed() {
        leaderMotor.setControl(velocityVoltage.withVelocity(Feeder.FEEDER_SPEED));
    }

    public void FeedPerio() {
        leaderMotor.setControl(velocityVoltage.withVelocity(Feeder.FEEDER_PERIODIC_SPEED));
    }

    public void FeedBack() {
        leaderMotor.setControl(velocityVoltage.withVelocity(-Feeder.FEEDER_PERIODIC_SPEED));
    }

    @Override
    public void periodic() {
        if (!container.intakeArm.getCanMoveUp()) {
            leaderMotor.set(0);
        }
    }
}
