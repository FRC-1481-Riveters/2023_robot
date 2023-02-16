package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristPositionCmd extends CommandBase {

    private WristSubsystem m_wristSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;

    public WristPositionCmd( WristSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_wristSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wristSubsystem.setPosition(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( m_wristSubsystem.atPosition() || m_waitAtPosition == false)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
}