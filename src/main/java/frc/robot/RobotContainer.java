package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.commands.WristPitchCommand;
import frc.robot.commands.WristRollCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);
    public final Joystick m_joystick = new Joystick(Constants.OIConstants.kJoystickPort);
    public final XboxController xboxController = new XboxController(0);/*
    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
    public final ShoulderControl m_shoulderControl = new ShoulderControl(m_shoulderSubsystem, 0);
    public final ShoulderControl m_shoulderPreset0deg = new ShoulderControl(m_shoulderSubsystem, 0);
    public final ShoulderControl m_shoulderPreset55deg = new ShoulderControl(m_shoulderSubsystem, 30.5);
    public final ShoulderControl m_shoulderPreset110deg = new ShoulderControl(m_shoulderSubsystem, 61.1);
    
  
    public final JoystickButton m_joyA = new JoystickButton(xboxController, 1); //button A
    public final JoystickButton m_joyB = new JoystickButton(xboxController, 2); // button B
    public final JoystickButton m_joyX = new JoystickButton(xboxController, 3); // button X
    public final JoystickButton m_joyY = new JoystickButton(xboxController, 4); // button Y*/
    public final JoystickButton m_joyBK = new JoystickButton(xboxController, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(xboxController, 8); // Start Button
    public final JoystickButton m_joyLB = new JoystickButton(xboxController, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(xboxController, 6); // Right bumper


    public final WristSubsystem m_wristSubsystem = new WristSubsystem();
    public final WristRollCommand m_wristRollRightCommand = new WristRollCommand(m_wristSubsystem,0.1);
    public final WristRollCommand m_wristRollLeftCommand = new WristRollCommand(m_wristSubsystem,-0.1);

    public final WristPitchCommand m_wristPitchUpCommand = new WristPitchCommand(m_wristSubsystem,0.1);
    public final WristPitchCommand m_wristPitchDownCommand = new WristPitchCommand(m_wristSubsystem,-0.1);



    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joystick));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joystick,3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_joystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_joystick, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        /*m_joyA.onTrue(m_shoulderPreset0deg);
        m_joyB.onTrue(m_shoulderPreset55deg);
        m_joyX.onTrue(m_shoulderPreset110deg);*/
        m_joyRB.whileTrue(m_wristRollRightCommand);
        m_joyLB.whileTrue(m_wristRollLeftCommand);

        m_joyST.whileTrue(m_wristPitchUpCommand);
        m_joyBK.whileTrue(m_wristPitchDownCommand);

        
    }
}
