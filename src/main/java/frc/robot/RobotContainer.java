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

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    
    public final Joystick m_joystick = new Joystick(1);

    public final XboxController xboxController = new XboxController(2);
    public final JoystickButton m_joyA = new JoystickButton(xboxController, 1); //button A
    public final JoystickButton m_joyBK = new JoystickButton(xboxController, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(xboxController, 8); // Start Button
    public final JoystickButton m_joyLB = new JoystickButton(xboxController, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(xboxController, 6); // Right bumper

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(xboxController);

    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem(xboxController);

    public final WristSubsystem m_wristSubsystem = new WristSubsystem(xboxController, m_shoulderSubsystem);

    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem);
    /*
    public final WristPitchCommand wristPF = new WristPitchCommand(m_wristSubsystem, 0.1); // pitch forward (up)
    public final WristPitchCommand wristPB = new WristPitchCommand(m_wristSubsystem, -0.1); // pitch backward (down)
    public final WristRollCommand wristRR = new WristRollCommand(m_wristSubsystem, 2.5); // roll right
    public final WristRollCommand wristRL = new WristRollCommand(m_wristSubsystem, -2.5); // roll left*/



    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joystick));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joystick,3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_joystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_joystick, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));

        m_joyRB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        m_joyLB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));


        //m_joyST.whileTrue(new RepeatCommand(wristPF));
        //m_joyBK.whileTrue(new RepeatCommand(wristPB));

    }
}
