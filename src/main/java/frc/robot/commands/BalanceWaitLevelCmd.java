package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
public class BalanceWaitLevelCmd extends CommandBase {

private SwerveSubsystem m_Subsystem;

    public BalanceWaitLevelCmd(SwerveSubsystem subsystem)
    {
       m_Subsystem = subsystem;
        
    }
    
    @Override
    public void initialize() 
    {
      System.out.println("BalanceWaitLevelCmd started");
    }
  
    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_Subsystem.getPitch()) < 3.5){
      System.out.println("BalanceWaitLevelCmd finished"); 
      return true;
    }
    else{
      return false;
    }
  }
}