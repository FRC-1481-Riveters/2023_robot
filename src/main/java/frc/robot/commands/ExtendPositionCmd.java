package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.Constants.ExtendConstants;

public class ExtendPositionCmd extends CommandBase {

    private ExtendSubsystem m_extendSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;
    private int countdown_pid_handoff;

    public ExtendPositionCmd( ExtendSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_extendSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
        addRequirements(subsystem);
      }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println( "ExtendPositionCmd to " + m_setPosition + ", current " + m_extendSubsystem.getPosition());
    countdown_pid_handoff = 0;
    m_extendSubsystem.setExtend(0);
    if( atPosition() == false )
    {
      if( m_extendSubsystem.getPosition() < m_setPosition )
        m_extendSubsystem.setExtend(0.8);
      else
        m_extendSubsystem.setExtend(-0.8);
    }
    //m_extendSubsystem.setPosition(m_setPosition);
    System.out.println(System.currentTimeMillis() + " ExtendPositionCmd " + m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (countdown_pid_handoff == 0 ) &&
        (atPosition() || m_waitAtPosition == false) )
    {
      m_extendSubsystem.setExtend(0);
      countdown_pid_handoff = 1;
      System.out.println(System.currentTimeMillis() + " ExtendPositionCmd " + m_setPosition + " counting down");
      return false;
    }
    else if( countdown_pid_handoff != 0 )
    {
       ++countdown_pid_handoff;
       if( countdown_pid_handoff == 5 )
       {
          System.out.println(System.currentTimeMillis() + " ExtendPositionCmd " + m_setPosition + " done");
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
    m_extendSubsystem.setPosition(m_setPosition);
  }

  private boolean atPosition()
  { 
    if( Math.abs(m_extendSubsystem.getPosition() - m_setPosition) < ExtendConstants.EXTEND_TOLERANCE )
    {
      System.out.println(System.currentTimeMillis() + " ExtendPositionCmd check: at position");
      return true;
    }
    else if( ( m_extendSubsystem.getExtendOutput() > 0 ) &&
             (m_extendSubsystem.getPosition() > (m_setPosition + ExtendConstants.EXTEND_TOLERANCE)) )
    {
        // oops missed it
        System.out.println(System.currentTimeMillis() + " ExtendPositionCmd check: missed gt");
        return true;
    }
    else if( ( m_extendSubsystem.getExtendOutput() < 0 ) &&
             (m_extendSubsystem.getPosition() < (m_setPosition - ExtendConstants.EXTEND_TOLERANCE)) )
    {
        // oops missed it
        System.out.println(System.currentTimeMillis() + " ExtendPositionCmd check: missed lt");
        return true;
    }
    else
    {
        return false;
    }
  }

}