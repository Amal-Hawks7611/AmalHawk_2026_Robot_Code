
package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  public static boolean doRejectUpdate;
  public static boolean isErrorWritten;
  public Timer disabledTimer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    doRejectUpdate = false;
    isErrorWritten = false;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    m_robotContainer.drivebase.zeroGyro();
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
    disabledTimer = new Timer();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.setMotorBrake(true);
    CommandScheduler.getInstance().schedule(m_robotContainer.led_morse);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_robotContainer.led_morse.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(m_robotContainer.led_morse);
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void simulationPeriodic() {
  }
}
