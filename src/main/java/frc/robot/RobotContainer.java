package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    // TODO implement SIM and REPLAY subsystem I/O.
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    

    public final XboxController m_driver = new XboxController(0);
    public final XboxController m_operator = new XboxController(1);
    public final JoystickButton m_joyA = new JoystickButton(m_operator, 1); //button A
    public final JoystickButton m_joyBK = new JoystickButton(m_operator, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(m_operator, 8); // Start Button
    public final JoystickButton m_joyLB = new JoystickButton(m_operator, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(m_operator, 6); // Right bumper

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(m_operator);

    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem(m_operator);

    public final WristSubsystem m_wristSubsystem = new WristSubsystem(m_operator);

    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem);
    /*
    public final WristPitchCommand wristPF = new WristPitchCommand(m_wristSubsystem, 0.1); // pitch forward (up)
    public final WristPitchCommand wristPB = new WristPitchCommand(m_wristSubsystem, -0.1); // pitch backward (down)
    public final WristRollCommand wristRR = new WristRollCommand(m_wristSubsystem, 2.5); // roll right
    public final WristRollCommand wristRL = new WristRollCommand(m_wristSubsystem, -2.5); // roll left*/



    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driver, 8).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //menu
        new JoystickButton(m_driver, 1).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR())); //A
        new JoystickButton(m_driver, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //start

        m_joyRB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        m_joyLB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));


    }
}
