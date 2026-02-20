package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.OI;
import frc.robot.subsystems.IntakeArm;

public class ArmDOWN extends Command {
    public final IntakeArm intakeArm;

    public ArmDOWN(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        if (intakeArm.getCanMoveDown()){
            System.out.println("Intake Manual Moving Ininialized");
            OI.IS_PROCESSING = true;
        }
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_INTAKE_ARM_ENABLED && intakeArm.getCanMoveDown()) {
            intakeArm.ArmDOWN();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeArm.StopMotors();
        OI.IS_PROCESSING = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}