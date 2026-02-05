package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OI;

public class IntakeArm extends SubsystemBase {
    public TalonFX leader_motor;
    public TalonFX slave_motor;
    public Timer timer;
    public boolean current_movement;

    public IntakeArm() {
        leader_motor = new TalonFX(Constants.IntakeArm.INTAKE_ARM_LEADER_PORT, new CANBus("arch"));
        slave_motor = new TalonFX(Constants.IntakeArm.INTAKE_ARM_SLAVE_PORT, new CANBus("arch"));
        current_movement = true;
    }

    public double getLeaderEncoder() {
        return leader_motor.getPosition().getValueAsDouble();
    }

    public double getSlaveEncoder() {
        return slave_motor.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        leader_motor.setPosition(0);
        slave_motor.setPosition(0);
    }

    public void ArmUP() {
        leader_motor.set(Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
        slave_motor.set(Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
    }

    public void ArmDOWN() {
        leader_motor.set(-Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
        slave_motor.set(-Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
    }

    public void StopMotors() {
        leader_motor.set(0);
        slave_motor.set(0);
    }

    public void OcalPID() {
        if (Math.abs(
                Constants.IntakeArm.INTAKE_SETPOINT - getLeaderEncoder()) < Constants.IntakeArm.OCALPID_TOLERANCE_VALUE
                &&
                getLeaderEncoder() < Constants.IntakeArm.INTAKE_SETPOINT && OI.IS_PROCESSING) {
            leader_motor.set(Constants.IntakeArm.INTAKE_OCALPID_SPEED);
            slave_motor.set(Constants.IntakeArm.INTAKE_OCALPID_SPEED);
        } else {
            StopMotors();
            OI.IS_PROCESSING = false;
        }
    }

    public void startPeriodic() {
        leader_motor.set(current_movement ? Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED
                : -Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
        slave_motor.set(current_movement ? Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED
                : -Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsIntakeArmProcessing", OI.IS_PROCESSING);

        if (getLeaderEncoder() >= Constants.IntakeArm.TOP_BREAK ||
                getLeaderEncoder() <= Constants.IntakeArm.DOWN_BREAK) {
            OI.IS_PROCESSING = false;
            StopMotors();
        }

        if (!OI.IS_PROCESSING) {
            double time = Timer.getFPGATimestamp();
            double wiggle = Constants.IntakeArm.INTAKE_ARM_MANUAL_SPEED * Math.sin(time * 15.0);
            leader_motor.set(wiggle);
            slave_motor.set(wiggle);
        } else {
            StopMotors();
        }
    }
}
