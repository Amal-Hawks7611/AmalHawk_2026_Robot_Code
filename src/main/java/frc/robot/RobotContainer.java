
package frc.robot;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Controlls;
import frc.robot.commands.Intake.Intake;
import frc.robot.commands.Led.LEDMorseScroller;
import frc.robot.commands.Led.LEDStateCycler;
import frc.robot.commands.Trajectory.AutonPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer {

        private final SendableChooser<Command> autoChooser;
        public final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        public final LimelightSubsystem limelight = new LimelightSubsystem();
        public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
        public final StatusLED ledSubsystem = new StatusLED();
        public final AutonPath otonom_path = new AutonPath();
        public CommandPS5Controller driverPs5 = Controlls.DRIVER_CONTROLLER;

        public final Command zerogyro;
        public final Command admin;

        public final Intake f_intake;

        public final LEDStateCycler led_cycle;
        public final LEDMorseScroller led_morse;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverPs5.getLeftY()  * 0.8,
                        () -> -driverPs5.getLeftX()  * 0.8)
                        .withControllerRotationAxis(driverPs5::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPs5::getRightX,
                        driverPs5::getRightY)
                        .headingWhile(true);

        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverPs5.getLeftY(),
                        () -> driverPs5.getLeftX())
                        .withControllerRotationAxis(() -> -driverPs5.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverPs5.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverPs5.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true);

        public RobotContainer() {
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("fIntake", new Intake(intakeSubsystem));

                zerogyro = new InstantCommand(() -> drivebase.zeroGyro());
                admin = new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());

                led_cycle = new LEDStateCycler(ledSubsystem);
                led_morse = new LEDMorseScroller(ledSubsystem, 180, "AMAL IN DA HOUSE");

                f_intake = new Intake(intakeSubsystem);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureBindings();
                configureButtonBindings();
        }

        private void configureBindings() {
                Command driveFieldOrientedCommand = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
                drivebase.setDefaultCommand(driveFieldOrientedCommand);


        }

        public void configureButtonBindings() {

                Controlls.INTAKE.toggleOnTrue(f_intake);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
