package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.util.Units;


public class VisionSubsystem extends SubsystemBase {
    
        final double TARGET_HEIGHT_METERS = 0.82;
    
        // private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);
        // private static final double LIMELIGHT_FORWARD = Units.inchesToMeters(5.22);
        private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(34.1);
        private static final double LIMELIGHT_HEIGHT = 0.15;
    
        // private static final double CAMERA_HEIGHT_PIXELS = 720;
        // private static final double CAMERA_WIDTH_PIXELS = 960;
    
        // private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
        // private static final double VERTICAL_FOV = Math.toRadians(45.7);
    
        private final DrivetrainSubsystem drivetrain;
    
        // private final PhotonCamera shooterLimelight = new PhotonCamera("gloworm");
    
        private boolean shooterHasTargets = false;
        private double distanceToTarget = Double.NaN;
        private double angleToTarget = Double.NaN;
        private double theta;
        public VisionSubsystem(DrivetrainSubsystem drivetrain) {
            this.drivetrain = drivetrain;
    
            ShuffleboardTab tab = Shuffleboard.getTab("Vision");
            tab.addBoolean("shooter has targets", () -> shooterHasTargets).withPosition(0, 0).withSize(1, 1);
            tab.addNumber("distance to target", () -> distanceToTarget).withPosition(1, 0).withSize(1, 1);
            tab.addNumber("angle to target", () -> Units.radiansToDegrees(angleToTarget)).withPosition(2, 0).withSize(1, 1);
            tab.addNumber("theta", () -> Math.toDegrees(theta));

        }
    
        public double getDistanceToTarget() {
            return distanceToTarget;
        }
    
        public double getAngleToTarget() {
            return angleToTarget;
        }
    
        public boolean shooterHasTargets() {
            return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
        }
    
        
        @Override
        public void periodic() {
            
            boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;;
            shooterHasTargets = hasTarget;
            double pitch = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            double yaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
            if (hasTarget) {
                theta = Math.toRadians(pitch) + LIMELIGHT_MOUNTING_ANGLE;
                distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) / Math.tan(theta);
                angleToTarget = drivetrain.getGyroscopeRotation().getRadians() + Math.toRadians(yaw);
            }
    
        }
    
    }
