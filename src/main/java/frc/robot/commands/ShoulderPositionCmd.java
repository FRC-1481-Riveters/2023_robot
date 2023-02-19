package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderPositionCmd extends CommandBase {

    private ShoulderSubsystem m_shoulderSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;
    private int countdown_pid_handoff;

    public ShoulderPositionCmd( ShoulderSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_shoulderSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
        addRequirements(subsystem);
    }

    /*public ShoulderPositionCmd( ShoulderSubsystem subsystem, double position )
    {
        ShoulderPositionCmd(subsystem, position, false);
    }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double position;
    countdown_pid_handoff = 0;
    m_shoulderSubsystem.setShoulder(0);
    position = m_shoulderSubsystem.getPosition();
    if( m_setPosition > ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL )
    {
       m_shoulderSubsystem.selectLowPID(true);
    }
    else
    {
      m_shoulderSubsystem.selectLowPID(false);
    }
    if( atPosition() == false )
    {
      if( position < m_setPosition )
        m_shoulderSubsystem.setShoulder(0.5);
      else
        m_shoulderSubsystem.setShoulder(-0.5);
    }
    //m_shoulderSubsystem.setPosition(m_setPosition);
    System.out.println(System.currentTimeMillis() + " ShoulderPositionCmd " + m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (countdown_pid_handoff == 0 ) &&
        (atPosition() || m_waitAtPosition == false) )
    {
      m_shoulderSubsystem.setShoulder(0);
      countdown_pid_handoff = 1;
      System.out.println(System.currentTimeMillis() + " ShoulderPositionCmd " + m_setPosition + " counting down");
      return false;
    }
    else if( countdown_pid_handoff != 0 )
    {
       ++countdown_pid_handoff;
       if( countdown_pid_handoff == 5 )
       {
          System.out.println(System.currentTimeMillis() + " ShoulderPositionCmd " + m_setPosition + " done");
          return true;
       }
       else
       {
          return false;
       }
    }
    else
    {
      countdown_pid_handoff = 0;
      return false;
    }
  }

  @Override
  public void end( boolean interrupted) 
  {
    m_shoulderSubsystem.setPosition(m_setPosition);
  }

  private boolean atPosition()
  {
    if( Math.abs(m_shoulderSubsystem.getPosition() - m_setPosition) < ShoulderConstants.SHOULDER_TOLERANCE )
    {
        return true;
    }
    else if( ( m_shoulderSubsystem.getShoulderOutput() > 0 ) &&
             (m_shoulderSubsystem.getPosition() > m_setPosition + ShoulderConstants.SHOULDER_TOLERANCE) )
    {
        // oops missed it
        return true;
    }
    else if( ( m_shoulderSubsystem.getShoulderOutput() < 0 ) &&
             (m_shoulderSubsystem.getPosition() < m_setPosition - ShoulderConstants.SHOULDER_TOLERANCE) )
    {
        // oops missed it
        return true;
    }
    else
    {
        return false;
    }
  }
}