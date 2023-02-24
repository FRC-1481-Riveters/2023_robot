package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristWaitPositionCmd extends CommandBase {

    private WristSubsystem m_wristSubsystem;
    private boolean m_bLessIfFalse;
    private double m_position;

    public WristWaitPositionCmd( WristSubsystem subsystem, boolean bLessIfFalse, double position )
    {
        m_wristSubsystem = subsystem;
        m_bLessIfFalse = bLessIfFalse;
        m_position = position;
    }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      boolean retval;
      if( (m_bLessIfFalse == false) && (m_wristSubsystem.getPosition() < m_position + WristConstants.WRIST_TOLERANCE) )
      {
        retval = true;
      }
      else if(m_wristSubsystem.getPosition() < m_position + WristConstants.WRIST_TOLERANCE)
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