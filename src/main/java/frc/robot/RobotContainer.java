package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import frc.robot.GamepadAxisButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ShoulderJogUpCmd;
import frc.robot.commands.ShoulderJogDownCmd;
import frc.robot.commands.ShoulderPositionCmd;
import frc.robot.commands.WristJogDownCmd;
import frc.robot.commands.WristJogUpCmd;
import frc.robot.commands.WristPositionCmd;
import frc.robot.commands.ExtendJogInCmd;
import frc.robot.commands.ExtendJogOutCmd;
import frc.robot.commands.ExtendPositionCmd;
import frc.robot.commands.IntakeJogCmd;
import frc.robot.commands.WristWaitPositionCmd;
import frc.robot.commands.ExtendWaitPositionCmd;
import frc.robot.commands.ShoulderWaitPositionCmd;

public class RobotContainer 
{
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final ExtendSubsystem extendSubsystem = new ExtendSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

    private final String trajectoryJSON = "paths/HalfCircle.wpilib.json";

    private Trajectory circleTrajectory = new Trajectory();

    private boolean isPracticeRobot;

    double driveSlowDivider = 2.2;

    GamepadAxisButton m_operatorRightYAxisUp;
    GamepadAxisButton m_operatorRightYAxisDown;
    GamepadAxisButton m_operatorLeftYAxisUp;
    GamepadAxisButton m_operatorLeftYAxisDown;
    GamepadAxisButton m_operatorLeftTrigger;
    GamepadAxisButton m_operatorRightTrigger;
    GamepadAxisButton m_operatorDpadUp;
    GamepadAxisButton m_operatorDpadDown;
    GamepadAxisButton m_operatorDpadLeft;
    GamepadAxisButton m_operatorDpadRight;
    GamepadAxisButton m_driverLT, m_driverRT;



    public RobotContainer() 
    {
        DigitalInput input;

        input = new DigitalInput(9);
        isPracticeRobot = !input.get();
        input.close();
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis) / driveSlowDivider,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis) / driveSlowDivider,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis) / driveSlowDivider,
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();

      try
      {
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         circleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
    }


    private void DriveSlowDividerSet( double divider )
    {
        driveSlowDivider = divider;
    }
    
    private void configureButtonBindings() 
    {
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).whenPressed( () -> swerveSubsystem.zeroHeading() );

        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value)
           .whenPressed(() -> DriveSlowDividerSet(1.0))
           .whenReleased(() -> DriveSlowDividerSet(2.2));
        
        new JoystickButton(operatorJoystick, XboxController.Button.kY.value)
            .whileTrue( ScoreHighCmd()
            .finallyDo( this::RumbleConfirm )
            );
        new JoystickButton(operatorJoystick, XboxController.Button.kB.value)
            .whileTrue( ScoreMidCmd() 
            .finallyDo( this::RumbleConfirm )
            );
        new JoystickButton(operatorJoystick, XboxController.Button.kA.value)
            .whileTrue( ScoreLowCmd() 
            .finallyDo( this::RumbleConfirm )
            );

        new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value)
            .whileTrue( new ConditionalCommand( ShelfLoadConeCmd(), ShelfLoadCubeCmd(), intakeSubsystem::getCone )
            .finallyDo( this::RumbleConfirm )
            );

        new JoystickButton(operatorJoystick, XboxController.Button.kBack.value)
            .whileTrue( new ConditionalCommand( StowCmdLow(), StowCmdHigh(), shoulderSubsystem::isBelowLevel )
            .finallyDo( this::RumbleConfirm )
            );

        new JoystickButton(operatorJoystick, XboxController.Button.kStart.value).whenPressed(() -> extendSubsystem.zeroPosition());

        m_operatorRightYAxisUp = new GamepadAxisButton(this::operatorRightYAxisUp);
        m_operatorRightYAxisUp.whileTrue( new ShoulderJogUpCmd( shoulderSubsystem ) );
        m_operatorRightYAxisDown = new GamepadAxisButton(this::operatorRightYAxisDown);
        m_operatorRightYAxisDown.whileTrue( new ShoulderJogDownCmd( shoulderSubsystem ) );

        m_operatorDpadUp = new GamepadAxisButton(this::operatorDpadUp);
        m_operatorDpadUp.whileTrue( new WristJogUpCmd( wristSubsystem ) );
        m_operatorDpadDown = new GamepadAxisButton(this::operatorDpadDown);
        m_operatorDpadDown.whileTrue( new WristJogDownCmd( wristSubsystem ) );

        m_operatorDpadLeft = new GamepadAxisButton(this::operatorDpadLeft);
        m_operatorDpadLeft.onTrue(
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(true) ),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kLeftRumble, 1.0) ),
                new WaitCommand(0.5),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kLeftRumble, 0.0) )
            )
         );
        m_operatorDpadRight = new GamepadAxisButton(this::operatorDpadRight);
        m_operatorDpadRight.onTrue( 
            new SequentialCommandGroup(
                new InstantCommand( ()-> intakeSubsystem.setCone(false) ),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kRightRumble, 1.0) ),
                new WaitCommand(0.5),
                new InstantCommand( ()-> operatorJoystick.setRumble(RumbleType.kRightRumble, 0.0) )
            )
        );

        m_operatorLeftYAxisUp = new GamepadAxisButton(this::operatorLeftYAxisUp);
        m_operatorLeftYAxisUp.whileTrue( new ExtendJogOutCmd( extendSubsystem ) );
        m_operatorLeftYAxisDown = new GamepadAxisButton(this::operatorLeftYAxisDown);
        m_operatorLeftYAxisDown.whileTrue( new ExtendJogInCmd( extendSubsystem ) );

        m_operatorLeftTrigger = new GamepadAxisButton(this::operatorLeftTrigger);
        m_operatorLeftTrigger
            .whileTrue( new ConditionalCommand( FloorLoadConeCmd(), FloorLoadCubeCmd(), intakeSubsystem::getCone )
            .finallyDo( this::RumbleConfirm )
        );

        m_driverLT = new GamepadAxisButton(this::DriverLTrigger);
        m_driverLT.whileTrue( new IntakeJogCmd( intakeSubsystem, true ) );
        m_driverRT = new GamepadAxisButton(this::DriverRTtrigger);
        m_driverRT.whileTrue( new IntakeJogCmd( intakeSubsystem, false ) );
    }

    public boolean operatorRightYAxisUp()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightY.value ) < -0.3 );
    }

    public boolean operatorRightYAxisDown()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightY.value ) > 0.3 );
    }

    public boolean DriverLTrigger()
    {
        return ( driverJoystick.getRawAxis( XboxController.Axis.kLeftTrigger.value ) > 0.3 );
    }

    public boolean DriverRTtrigger()
    {
        return ( driverJoystick.getRawAxis( XboxController.Axis.kRightTrigger.value ) > 0.3 );
    }

    
    public boolean operatorLeftYAxisUp()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftY.value ) < -0.3 );
    }

    public boolean operatorLeftYAxisDown()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftY.value ) > 0.3 );
    }

    public boolean operatorLeftTrigger()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kLeftTrigger.value ) > 0.3 );
    }

    public boolean operatorRightTrigger()
    {
        return ( operatorJoystick.getRawAxis( XboxController.Axis.kRightTrigger.value ) > 0.3 );
    }

    public boolean operatorDpadUp()
    {
        return ( operatorJoystick.getPOV() == 0 );
    }

    public boolean operatorDpadDown()
    {
        return ( operatorJoystick.getPOV() == 180 );
    }

    public boolean operatorDpadLeft()
    {
        return ( operatorJoystick.getPOV() == 270 );
    }

    public boolean operatorDpadRight()
    {
        return ( operatorJoystick.getPOV() == 90 );
    }


    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                circleTrajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(circleTrajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }


    public Command StowCmdLow()
    {
        // STOW when the arm starts below level
        return new SequentialCommandGroup(
            // Move SHOULDER up to clear the bumper
            new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            new ParallelCommandGroup(
                // Move WRIST all the way in
                new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_STOWED, true),
                new SequentialCommandGroup(
                    // Once WRIST is almost all the way in
                    new WristWaitPositionCmd( wristSubsystem, true, WristConstants.WRIST_POSITION_STOWED - 300 ),
                    // Pull EXTEND all the way in
                    new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
                    // Pull SHOULDER all the way down
                    new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_STOWED, true)
                )
            ),
            new InstantCommand(() -> shoulderSubsystem.setShoulder(0)),
            new InstantCommand(() -> extendSubsystem.setExtend(0)),
            new InstantCommand(() -> intakeSubsystem.setIntake(0)),
            new InstantCommand(() -> wristSubsystem.setWrist(0))
        );
    }

    public Command StowCmdHigh()
    {
        // STOW when the arm is above level
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // Move EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
                // Once EXTEND is in past the shelf, move WRIST all the way in
                new SequentialCommandGroup(
                    new ExtendWaitPositionCmd( extendSubsystem, false, ExtendConstants.EXTEND_POSITION_SHELF ),
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_STOWED, true)
                ),
                // Once WRIST is almost all the way in, start moving SHOULDER down
                new SequentialCommandGroup(
                    new WristWaitPositionCmd(wristSubsystem, true, WristConstants.WRIST_POSITION_STOWED - 300),
                    new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_STOWED, true)
                )
            ),
            new InstantCommand(() -> shoulderSubsystem.setShoulder(0)),
            new InstantCommand(() -> extendSubsystem.setExtend(0)),
            new InstantCommand(() -> intakeSubsystem.setIntake(0)),
            new InstantCommand(() -> wristSubsystem.setWrist(0))
        );
    }

    public Command ScoreHighCmd(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // Pull game piece in a little
                new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to high position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_HIGH, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to HIGH position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_HIGH, true)
                )
            ),
            // Move EXTEND to HIGH position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_HIGH, true)
        );
    }

    public Command ScoreMidCmd(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // Pull game piece in a little
                new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to mid position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_MID, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to MID position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_MID, true)
                )
            ),
            // Move EXTEND to MID position
            new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_MID, true)
        );
    }

    public Command ScoreLowCmd(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                // Pull game piece in a little
                new IntakeJogCmd( intakeSubsystem, true ).withTimeout(0.1),
                // Pull EXTEND all the way in
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true)
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to LEVEL position
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_LEVEL, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    // Move WRIST to LOW position
                    new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_LOW, true)
                )
            ),
            new ParallelCommandGroup(
                // Move SHOULDER to LOW
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_LOW, true),
                // Move EXTEND to LOW
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_LOW, true)
            )
        );
    }

    public Command ShelfLoadConeCmd(){
        return new SequentialCommandGroup(
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            new ParallelCommandGroup (
                // Move SHOULDER to SHELF
                new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    new ParallelCommandGroup (
                        // Move WRIST to SHELF
                        new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_SHELF_CONE, true),
                        // Move EXTEND to SHELF
                        new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_SHELF, true)
                    )
                )
            )
        );
    }

    public Command ShelfLoadCubeCmd(){
        return new SequentialCommandGroup(
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            new ParallelCommandGroup (
                // Move SHOULDER to SHELF
                new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_SHELF, true),
                new SequentialCommandGroup(
                    // Wait for SHOULDER to be above bumper
                    new ShoulderWaitPositionCmd( shoulderSubsystem, false, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL ),
                    new ParallelCommandGroup (
                        // Move WRIST to SHELF
                        new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_SHELF_CUBE, true),
                        // Move EXTEND to SHELF
                        new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_SHELF, true)
                    )
                )
            )
        );
    }

    public Command FloorLoadConeCmd()
    {
        return new SequentialCommandGroup(
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Move SHOULDER to above bumper
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            // Move WRIST to CONE PICKUP
            new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_CONE_PICKUP, true),
            new ParallelCommandGroup (
                // Move EXTEND to CONE PICKUP
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_CONE_PICKUP, true),
                // Move SHOULDER to CONE PICKUP
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_CONE_PICKUP, true)
            )
        );
    }

    public Command FloorLoadCubeCmd()
    {
        return new SequentialCommandGroup(
            // Move EXTEND all the way in
            new ExtendPositionCmd(extendSubsystem, ExtendConstants.EXTEND_MOTOR_MIN, true),
            // Move SHOULDER to above bumper
            new ShoulderPositionCmd(shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_BETWEEN_STOWED_AND_LEVEL, true),
            // Move WRIST to CUBE PICKUP
            new WristPositionCmd(wristSubsystem, WristConstants.WRIST_POSITION_CUBE_PICKUP, true),
            new ParallelCommandGroup (
                // Move EXTEND to CUBE PICKUP
                new ExtendPositionCmd (extendSubsystem, ExtendConstants.EXTEND_POSITION_CUBE_PICKUP, true),
                // Move SHOULDER to CUBE PICKUP
                new ShoulderPositionCmd (shoulderSubsystem, ShoulderConstants.SHOULDER_POSITION_CUBE_PICKUP, true)
            )
        );
    }

    public Command RumbleCmd( double rumbleAmount )
    {
        return new ParallelCommandGroup(
            new InstantCommand(() -> operatorJoystick.setRumble(RumbleType.kLeftRumble, rumbleAmount) ),
            new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kLeftRumble, rumbleAmount) )
        );
    }
    
    public void RumbleConfirm( boolean interrupted )
    {
        if( interrupted == false )
        {
            CommandScheduler.getInstance().schedule(
                RumbleCmd( 1.0 ),
                new WaitCommand(0.5),
                RumbleCmd( 0.0 )
            );
        }
        else
        {
            operatorJoystick.setRumble(RumbleType.kLeftRumble, 0);
            driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
        }
    }
 }
