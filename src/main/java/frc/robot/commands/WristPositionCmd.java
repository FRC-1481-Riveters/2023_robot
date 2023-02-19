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
        addRequirements(subsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristPositionCmd " + m_setPosition);
    m_wristSubsystem.setPosition(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( m_wristSubsystem.atPosition() || m_waitAtPosition == false)
      {
        System.out.println("WristPositionCmd " + m_setPosition + " done");
        return true;
      }
      else
      {
        return false;
      }
  }

  @Override
  public void end( boolean interrupted ) {
    m_wristSubsystem.setWrist(0);
  }
}