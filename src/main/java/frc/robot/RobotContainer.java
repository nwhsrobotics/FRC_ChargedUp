package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);
    public final Joystick m_joy0 = new Joystick(Constants.OIConstants.kJoystickPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joy0));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joy0, 12).onTrue(new InstantCommand(() -> swerveSubsystem.gyro.reset())); //button 12 zeroes gyro to reset if robot drifts
        new JoystickButton(m_joy0, 2).onTrue(new InstantCommand(() -> swerveSubsystem.swtichFR())); //button on the side of stick toggles field relative (default is true)
        new JoystickButton(m_joy0, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())));
    }
}
