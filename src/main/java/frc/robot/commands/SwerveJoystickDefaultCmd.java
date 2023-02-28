package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController m_driver;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController m_driver) {
        this.swerveSubsystem = swerveSubsystem;
        this.m_driver = m_driver;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedCoefficient = (m_driver.getLeftTriggerAxis() > 0.15 || m_driver.getRightTriggerAxis() > 0.15) ? 1 : 0.5; 

        double xSpeed = Math.abs(m_driver.getLeftY()) < OIConstants.kXYDeadband ? 0 : -m_driver.getLeftY() > 0 ? (-m_driver.getLeftY() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband)) : (-m_driver.getLeftY() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband));
        double ySpeed = Math.abs(m_driver.getLeftX()) < OIConstants.kXYDeadband ? 0 : -m_driver.getLeftX() > 0 ? (-m_driver.getLeftX() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband)) : (-m_driver.getLeftX() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband));
        double rotatingSpeed = Math.abs(m_driver.getRightX()) < OIConstants.kZDeadband ? 0 : -m_driver.getRightX() > 0 ? (-m_driver.getRightX() - OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient: (-m_driver.getRightX() + OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;

        ChassisSpeeds chassisSpeeds = (swerveSubsystem.isFR) ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, Rotation2d.fromDegrees(swerveSubsystem.getHeading())) : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, new Rotation2d(0));

        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
