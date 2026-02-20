package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OI;
import frc.robot.Constants.IntakeArm.Leader;
import frc.robot.Constants.IntakeArm.Slave;

public class IntakeArm extends SubsystemBase {
    public TalonFX leader_motor;
    public TalonFX slave_motor;
    public Timer timer;
    public boolean current_movement;
    private boolean isPidActive = false;
    private boolean canMoveDown = true;
    private boolean canMoveUp = true;

    public IntakeArm() {
        leader_motor = new TalonFX(frc.robot.Constants.IntakeArm.INTAKE_ARM_LEADER_PORT, new CANBus("arch"));
        slave_motor = new TalonFX(frc.robot.Constants.IntakeArm.INTAKE_ARM_SLAVE_PORT, new CANBus("arch"));
        leader_motor.setPosition(0);
        slave_motor.setPosition(0);
        current_movement = true;
    }

    public double getLeaderEncoder() {
        return -leader_motor.getPosition().getValueAsDouble();
    }

    public double getSlaveEncoder() {
        return -slave_motor.getPosition().getValueAsDouble();
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
        double pos = getLeaderEncoder();
        double s_pos = getSlaveEncoder();
        boolean leaderStopped = false;
        boolean slaveStopped = false;
        
        if(Math.abs(pos - Leader.DOWN_LIMIT-0.5) < frc.robot.Constants.IntakeArm.OCALPID_TOLERANCE_VALUE){
            leader_motor.set(0);
            System.out.println("Leader stop");
            leaderStopped = true;
        }else{leader_motor.set(frc.robot.Constants.IntakeArm.INTAKE_OCALPID_SPEED);}
        
        if(Math.abs(s_pos - Slave.DOWN_LIMIT-0.5) < frc.robot.Constants.IntakeArm.OCALPID_TOLERANCE_VALUE){
            slave_motor.set(0);
            System.out.println("Slave stop");
            slaveStopped = true;
        }else{slave_motor.set(frc.robot.Constants.IntakeArm.INTAKE_OCALPID_SPEED);}
        
        if(leaderStopped && slaveStopped){
            isPidActive = false;
        }
    }
    
    public boolean getCanMoveUp(){
        return canMoveUp;
    }
    
    public boolean getCanMoveDown(){
        return canMoveDown;
    }
    
    public boolean getIsPidActive(){
        return isPidActive;
    }
    
    public void setIsPidActive(boolean active){
        isPidActive = active;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsIntakeArmProcessing", OI.IS_PROCESSING);
        SmartDashboard.putNumber("LeaderEncoder", getLeaderEncoder());
        SmartDashboard.putNumber("SlaveEncoder", getSlaveEncoder());
        SmartDashboard.putBoolean("CanMoveUp", canMoveUp);
        SmartDashboard.putBoolean("CanMoveDown", canMoveDown);
        SmartDashboard.putBoolean("PID", isPidActive);
        double pos = getLeaderEncoder();
        double s_pos = getSlaveEncoder();
        if(!OI.IS_PROCESSING){
            if (pos < Leader.TOP_LIMIT){
                // leader_motor.set(-0.15);
            }
            if (s_pos < Slave.TOP_LIMIT){
               //  slave_motor.set(-0.15);
            }

            if (pos > Leader.DOWN_LIMIT){
                // leader_motor.set(0.15);
            }
            if (s_pos > Slave.DOWN_LIMIT){
                // slave_motor.set(0.15);
            }

            if(pos < Leader.DOWN_LIMIT && pos > Leader.TOP_LIMIT) {
                leader_motor.set(0);
            }
            if(s_pos < Slave.DOWN_LIMIT && s_pos > Slave.TOP_LIMIT) {
                 slave_motor.set(0);
            }
        }

        if(pos < Leader.TOP_LIMIT + 1.5 || s_pos < Slave.TOP_LIMIT + 1.5){
            canMoveUp = false;
        }else{canMoveUp = true;}

        if(pos > Leader.DOWN_LIMIT || s_pos > Slave.DOWN_LIMIT){
            canMoveDown = false;
        }else{canMoveDown = true;}
    }
}