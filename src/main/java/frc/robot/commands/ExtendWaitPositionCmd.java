package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendConstants;
import frc.robot.subsystems.ExtendSubsystem;

public class ExtendWaitPositionCmd extends CommandBase {

    private ExtendSubsystem m_extendSubsystem;
    private boolean m_bLessIfFalse;
    private double m_position;

    public ExtendWaitPositionCmd( ExtendSubsystem subsystem, boolean bLessIfFalse, double position )
    {
        m_extendSubsystem = subsystem;
        m_bLessIfFalse = bLessIfFalse;
        m_position = position;
    }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      boolean retval;
      if( (m_bLessIfFalse == false) && (m_extendSubsystem.getPosition() < m_position + ExtendConstants.EXTEND_TOLERANCE) )
      {
        retval = true;
      }
      else if(m_extendSubsystem.getPosition() > m_position - ExtendConstants.EXTEND_TOLERANCE)
      {
        retval = true;
      }
      else
      {
        retval = false;
      }
      return retval;
  }
}