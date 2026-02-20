package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnabledParts;
import frc.robot.Constants.OI;
import frc.robot.subsystems.IntakeArm;

public class ArmPID extends Command {
    public final IntakeArm intakeArm;

    public ArmPID(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        System.out.println("Intake PID Moving Ininialized");
        OI.IS_PROCESSING = true;
        intakeArm.setIsPidActive(true);
    }

    @Override
    public void execute() {
        if (EnabledParts.IS_INTAKE_ARM_ENABLED && intakeArm.getCanMoveUp()) {
            intakeArm.OcalPID();
        } else {
            this.end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PID ENDED.");
        OI.IS_PROCESSING = false;
        intakeArm.setIsPidActive(false);
        intakeArm.StopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}