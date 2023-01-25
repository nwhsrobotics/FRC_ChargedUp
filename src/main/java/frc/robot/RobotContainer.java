package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);
    private final Joystick m_joy0 = new Joystick(2);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joy0));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joy0, 7).whenPressed(() -> swerveSubsystem.zeroHeading()); //back button zeroes gyro to reset if robot drifts
        //new JoystickButton(m_joy0, 5).whileHeld(new SwerveJoystickCornerCmd(swerveSubsystem, m_joy0)); // hold left bumper to drift around a corner
        // choose corner with left joystick, rotate with right
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {
        return autoCmd;
    }
}
