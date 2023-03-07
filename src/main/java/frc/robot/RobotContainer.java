package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.ExtendArmDPad;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    

    public final XboxController m_driver = new XboxController(0);
    public final XboxController m_operator = new XboxController(2);
    public final JoystickButton m_joyA = new JoystickButton(m_operator, 1); //button A
    public final JoystickButton m_joyBK = new JoystickButton(m_operator, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(m_operator, 8); // Start Button
    public final JoystickButton m_joyLB = new JoystickButton(m_operator, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(m_operator, 6); // Right bumper

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(m_operator);

    //public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem();
    //public final ExtendArmCmd m_ExtendArmCmd0 = new ExtendArmCmd(m_extendArmSubsystem, 0);
    //public final ExtendArmCmd m_ExtendArmCmd36 = new ExtendArmCmd(m_extendArmSubsystem, 36.0);

    //public final WristSubsystem m_wristSubsystem = new WristSubsystem(m_operator);

    // TODO public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    // TODO public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem);




    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
        //m_extendArmSubsystem.setDefaultCommand(new ExtendArmDPad(m_extendArmSubsystem, m_operator));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driver, 8).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //menu
        new JoystickButton(m_driver, 1).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR())); //A
        new JoystickButton(m_driver, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //start
        //new JoystickButton(m_operator, 2).onTrue(m_ExtendArmCmd36);
        //new JoystickButton(m_operator, 3).onTrue(m_ExtendArmCmd0);
        //TODO m_joyRB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        //TODO m_joyLB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));


    }
}
