package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderPositionCmd extends CommandBase {

    private ShoulderSubsystem m_shoulderSubsystem;
    private double m_setPosition;
    private boolean m_waitAtPosition;

    public ShoulderPositionCmd( ShoulderSubsystem subsystem, double position, boolean waitAtPosition )
    {
        m_shoulderSubsystem = subsystem;
        m_setPosition = position;
        m_waitAtPosition = waitAtPosition;
    }

    /*public ShoulderPositionCmd( ShoulderSubsystem subsystem, double position )
    {
        ShoulderPositionCmd(subsystem, position, false);
    }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulderSubsystem.setPosition(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( m_shoulderSubsystem.atPosition() || m_waitAtPosition == false)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
}