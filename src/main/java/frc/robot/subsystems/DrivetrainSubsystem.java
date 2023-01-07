
package frc.robot.subsystems;

// import com.ctre.phoenix.sensors.PigeonIMU;

import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;


import static frc.robot.Constants.*;


public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  public static final double DRIVETRAIN_CURRENT_LIMIT = 40.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //  Remove if you are using a Pigeon
//   private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  //  Uncomment if you are using a NavX
  
 private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final SwerveDrivePoseEstimator estimator;

  private ChassisSpeeds currentVelocity = new ChassisSpeeds();
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private final SwerveDriveOdometry odometry ;
  private final Field2d m_field = new Field2d();

  public DrivetrainSubsystem() {
        
        // make a Current limit with 50A for each motor
        Mk4ModuleConfiguration mk4ModuleConfiguration = new Mk4ModuleConfiguration();
        mk4ModuleConfiguration.setDriveCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    estimator = new SwerveDrivePoseEstimator(
        getGyroscopeRotation(),
        new Pose2d(), m_kinematics,
        VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
        VecBuilder.fill(0.01), // Gyroscope rotation std-dev
        VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs
  
    odometry = new SwerveDriveOdometry(
        m_kinematics,
        getGyroscopeRotation()
    );
        
    tab.addNumber("Odometry X", () -> getPose().getX()); 
    tab.addNumber("Odometry Y", () -> getPose().getY());
    tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
    tab.add("Field", m_field);
    
    
}

  public void zeroGyroscope() {
    //  Remove if you are using a Pigeon
    // m_pigeon.setFusedHeading(0.0);

    //  Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }

  public Pose2d getPose() {
        // return estimator.getEstimatedPosition();
        return odometry.getPoseMeters();
  }
  
  public void zeroRotation() {
        estimator.resetPosition(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()),
                getGyroscopeRotation());
        odometry.resetPosition(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()),
        getGyroscopeRotation());
  }

  public void zeroPostion(){
        estimator.resetPosition(new Pose2d(3,3.5,new Rotation2d()), getGyroscopeRotation());
        odometry.resetPosition(new Pose2d(3,3.5,new Rotation2d()), getGyroscopeRotation());
  }

  public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
  }

  public void setPose(Pose2d pose) {
        estimator.resetPosition(pose, getGyroscopeRotation());
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

  public Rotation2d getGyroscopeRotation() {
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
        if (m_navx.isMagnetometerCalibrated()) 
                return Rotation2d.fromDegrees(m_navx.getFusedHeading());

        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  

  @Override
  public void periodic() {

        SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
                new Rotation2d(m_frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
                new Rotation2d(m_frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
                new Rotation2d(m_backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(m_backRightModule.getDriveVelocity(),
                new Rotation2d(m_backRightModule.getSteerAngle()));


        currentVelocity = m_kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        estimator.update(getGyroscopeRotation(), currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        //For testing later for accurcy
        odometry.update(getGyroscopeRotation(), currentFrontLeftModuleState, currentFrontRightModuleState, currentBackLeftModuleState, currentBackRightModuleState);
        
        m_field.setRobotPose(odometry.getPoseMeters());

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE , states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE  , states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

       
        }
        
        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
                
                m_frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE , desiredStates[0].angle.getRadians());
                m_frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE  , desiredStates[1].angle.getRadians());
                m_backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
                m_backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());
    }

    public void stopModules() {
                m_frontLeftModule.set(0, 0);
                m_frontRightModule.set(0,0);
                m_backLeftModule.set(0, 0);
                m_backRightModule.set(0, 0);
        }
}
