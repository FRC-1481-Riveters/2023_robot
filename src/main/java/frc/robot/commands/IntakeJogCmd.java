// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeJogCmd extends CommandBase {

  private enum IntakePiece
  {
    hasNothing,
    hasCube,
    hasCone
  };

  private IntakeSubsystem m_intakeSubsystem;
  private boolean m_isCube, m_isLoading, m_isScoring;;
  private static IntakePiece intakePiece = IntakePiece.hasNothing;


  /** Creates a new ExtendJogUp. */
  public IntakeJogCmd( IntakeSubsystem intakeSubsystem, boolean bIsCube, boolean bIsLoading ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    addRequirements((intakeSubsystem));
    m_isCube = bIsCube;
    m_isLoading= bIsLoading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("intake jog cmd");
    if( m_isLoading == true )
    {
      if( (m_isCube == true) && (intakePiece != IntakePiece.hasCone) )
      {      
        intakePiece = IntakePiece.hasCube;
        m_intakeSubsystem.setIntake( -0.5 );
      }
      else
      {
        intakePiece = IntakePiece.hasCone;
        m_intakeSubsystem.setIntake( 1.0 );
      }
    }
    else
    {      
      if( intakePiece == IntakePiece.hasCone )
      {
        m_intakeSubsystem.setIntake( -0.5 );
      }
      else
      {
        m_intakeSubsystem.setIntake( 0.4 );
      }
      intakePiece = IntakePiece.hasNothing;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if( intakePiece == IntakePiece.hasCone && m_isLoading == true )
    {
      m_intakeSubsystem.setIntake(0.07);
    }
    else
    {
      m_intakeSubsystem.setIntake(0.0);
    }
  }
}
