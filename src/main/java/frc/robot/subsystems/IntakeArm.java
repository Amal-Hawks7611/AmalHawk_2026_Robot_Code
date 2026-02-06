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
        if (Math.abs(Constants.IntakeArm.INTAKE_SETPOINT - getLeaderEncoder()) < Constants.IntakeArm.OCALPID_TOLERANCE_VALUE
                &&
                getLeaderEncoder() < Constants.IntakeArm.INTAKE_SETPOINT && OI.IS_PROCESSING) {
            leader_motor.set(Constants.IntakeArm.INTAKE_OCALPID_SPEED);
            slave_motor.set(Constants.IntakeArm.INTAKE_OCALPID_SPEED);
        } else {
            StopMotors();
            OI.IS_PROCESSING = false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsIntakeArmProcessing", OI.IS_PROCESSING);
        SmartDashboard.putNumber("LeaderEncoder", getLeaderEncoder());
        SmartDashboard.putNumber("SlaveEncoder", getSlaveEncoder());
        SmartDashboard.putBoolean("CanMoveUp", frc.robot.Constants.IntakeArm.CAN_MOVE_UP);
        SmartDashboard.putBoolean("CanMoveDown", frc.robot.Constants.IntakeArm.CAN_MOVE_DOWN);

        if (getLeaderEncoder() >= Constants.IntakeArm.TOP_BREAK) {
                Constants.IntakeArm.CAN_MOVE_UP = false;
                StopMotors();
        }
        else if(getLeaderEncoder() <= Constants.IntakeArm.DOWN_BREAK){
            Constants.IntakeArm.CAN_MOVE_DOWN = false;
            StopMotors();
        }
        else{
            Constants.IntakeArm.CAN_MOVE_DOWN = true;
            Constants.IntakeArm.CAN_MOVE_UP = true;
            if (!OI.IS_PROCESSING) {
            double time = Timer.getFPGATimestamp();
            double wiggle = Constants.IntakeArm.INTAKE_PRD_SPEED * Math.sin(time * 4.0);
            leader_motor.set(-wiggle);
            slave_motor.set(-wiggle);
            }
        }
    }
}
