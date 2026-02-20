package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.Constants.OI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX leaderMotor;
    private StatusSignal<Angle> leaderMotorPosition;
    public RobotContainer container;

    public Timer timer = new Timer();

    public IntakeSubsystem(RobotContainer container) {
        leaderMotor = new TalonFX(Intake.INTAKE_LEADER_MOTOR_PORT, new CANBus("arch"));
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

    public void Intake() {
                leaderMotor.set(Intake.INTAKE_SPEED);
            }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsIntaking", OI.IS_INTAKING);
        SmartDashboard.putNumber("Intake Leader Motor Value", getLeaderMotorEncoder());
        SmartDashboard.putNumber("IntakeSPeed", leaderMotor.get());
    }
}
