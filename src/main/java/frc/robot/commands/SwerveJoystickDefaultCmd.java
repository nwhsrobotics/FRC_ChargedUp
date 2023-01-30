package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick m_joy0;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, Joystick m_joy0) {
        this.swerveSubsystem = swerveSubsystem;
        this.m_joy0 = m_joy0;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        //corner swerve
        if (m_joy0.getRawButton(3) || m_joy0.getRawButton(4) || m_joy0.getRawButton(5) || m_joy0.getRawButton(6)) {
            double rotatingSpeed = m_joy0.getTwist() * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond; // no deadband and speed selector because corner swerve is for emergencies
            ChassisSpeeds chassisSpeeds = swerveSubsystem.isFR ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotatingSpeed, Rotation2d.fromDegrees(-swerveSubsystem.gyro.getAngle())) : ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotatingSpeed, new Rotation2d(0));

            if (m_joy0.getRawButton(5)) { //front left
                swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)));
            }
            else if (m_joy0.getRawButton(6)) { //front right
                swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2)));
            }
            else if (m_joy0.getRawButton(3)) { //back left
                swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)));
            }
            else { // back right
                swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2)));
            }
        }
        //regular swerve
        else {
            double speedCoefficient = m_joy0.getTrigger() ? 1 : (-m_joy0.getRawAxis(3)) * 0.3 + 0.5; 

            double xSpeed = Math.abs(m_joy0.getY()) < OIConstants.kXYDeadband ? 0 : m_joy0.getY() > 0 ? (m_joy0.getY() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband)) : (m_joy0.getY() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband));
            double ySpeed = Math.abs(m_joy0.getX()) < OIConstants.kXYDeadband ? 0 : m_joy0.getX() > 0 ? (m_joy0.getX() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband)) : (m_joy0.getX() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1/(1-OIConstants.kXYDeadband));
            double rotatingSpeed = Math.abs(m_joy0.getTwist()) < OIConstants.kZDeadband ? 0 : m_joy0.getTwist() > 0 ? (m_joy0.getTwist() - OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient: (m_joy0.getTwist() + OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;

            ChassisSpeeds chassisSpeeds = (swerveSubsystem.isFR) ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, Rotation2d.fromDegrees(-swerveSubsystem.gyro.getAngle())) : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, new Rotation2d(0));

            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        }
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
