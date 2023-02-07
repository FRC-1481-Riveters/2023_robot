package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.GamepadAxisButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ShoulderJogUpCmd;
import frc.robot.commands.ShoulderJogDownCmd;


public class RobotContainer 
{
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

    private final String trajectoryJSON = "paths/HalfCircle.wpilib.json";

    private Trajectory circleTrajectory = new Trajectory();

    private boolean isPracticeRobot;

    GamepadAxisButton m_operatorRightYAxisUp;
    GamepadAxisButton m_operatorRightYAxisDown;


    public RobotContainer() 
    {
        DigitalInput input;

        input = new DigitalInput(9);
        isPracticeRobot = !input.get();
        input.close();
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
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



    
    private void configureButtonBindings() 
    {
        new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
        new JoystickButton(operatorJoystick, 4).whenPressed(() -> armSubsystem.setShoulderPosition(ArmConstants.ARM_SHOULDER_POSITION_HIGH));
        new JoystickButton(operatorJoystick, 2).whenPressed(() -> armSubsystem.setShoulderPosition(ArmConstants.ARM_SHOULDER_POSITION_MID));
        new JoystickButton(operatorJoystick, 1).whenPressed(() -> armSubsystem.setShoulderPosition(ArmConstants.ARM_SHOULDER_POSITION_LOW));

        m_operatorRightYAxisUp = new GamepadAxisButton(this::operatorRightYAxisUp);
        m_operatorRightYAxisUp.whileTrue( new ShoulderJogUpCmd( armSubsystem ) );
        m_operatorRightYAxisDown = new GamepadAxisButton(this::operatorRightYAxisDown);
        m_operatorRightYAxisDown.whileTrue( new ShoulderJogDownCmd( armSubsystem ) );
    }

    public boolean operatorRightYAxisUp()
    {
        return ( operatorJoystick.getRawAxis(5) < -0.5 );
    }

    public boolean operatorRightYAxisDown()
    {
        return ( operatorJoystick.getRawAxis(5) > 0.5 );
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
}