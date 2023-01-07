// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_drivetrainSubsystem);

  private final XboxController m_controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(m_visionSubsystem);
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) *  DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) *  DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new Button(m_controller::getStartButton)
            .whenPressed(m_drivetrainSubsystem::zeroPostion);
          }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    
    
    final double MAX_SPEED = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    final double MAX_ACCELERATION = MAX_SPEED/1; 

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_SPEED,MAX_ACCELERATION)
                    .setKinematics(m_drivetrainSubsystem.m_kinematics);

        // // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                
        //         List.of(
        //                 new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                        
        //                 new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                        
        //                 new Pose2d(4, 0, Rotation2d.fromDegrees(180)),

        //                 new Pose2d(2, 1, Rotation2d.fromDegrees(180)),

        //                 new Pose2d(0, 0, Rotation2d.fromDegrees(180))
                        
        //                 ),

        //         trajectoryConfig);
        

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(2, 1),
                        new Translation2d(3, 1)),
                new Pose2d(4, 0, Rotation2d.fromDegrees(180)),
                trajectoryConfig);
        
        final double MAX_ANGLAUR = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(MAX_ANGLAUR ,MAX_ANGLAUR/2);
    

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(3, 0.0, 0.0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_drivetrainSubsystem::getPose,
                m_drivetrainSubsystem.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSubsystem.setPose(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.09);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
