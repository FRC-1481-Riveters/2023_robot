package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendSubsystem;

public class ExtendPositionCmd extends CommandBase {

    private ExtendSubsystem m_extendSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;

    public ExtendPositionCmd( ExtendSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_extendSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extendSubsystem.setPosition(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( m_extendSubsystem.atPosition() || m_waitAtPosition == false)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
}