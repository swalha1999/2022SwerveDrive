package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // if (m_translationXSupplier.getAsDouble()!=0 || m_translationYSupplier.getAsDouble() != 0 || m_rotationSupplier.getAsDouble() != 0){
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble() *0.9,
                            m_translationYSupplier.getAsDouble() *0.9,
                            m_rotationSupplier.getAsDouble() * 0.6,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
            );
        // }
        // else{
        //     m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0.1,m_drivetrainSubsystem.getGyroscopeRotation()));
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.05));
    }
}
