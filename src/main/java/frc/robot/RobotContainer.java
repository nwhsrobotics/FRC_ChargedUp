package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.commands.shoulderCommand;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);
    public final XboxController xboxController = new XboxController(0);
    public final Joystick m_joy0 = new Joystick(Constants.OIConstants.kJoystickPort);
    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
    public final shoulderCommand m_shoulderCommand = new shoulderCommand(m_shoulderSubsystem);

    public final JoystickButton m_joyA = new JoystickButton(xboxController, 1); //button A
    public final JoystickButton m_joyB = new JoystickButton(xboxController, 2); // button B

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joy0));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joy0,3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_joy0, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_joy0, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        m_joyA.whileTrue(m_shoulderCommand);
    }
}
