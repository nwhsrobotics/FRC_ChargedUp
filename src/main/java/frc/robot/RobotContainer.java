package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendedArmControl;
import frc.robot.commands.GrabberExtendCommand;
import frc.robot.commands.GrabberRetractCommand;
import frc.robot.commands.GrabberTurnOffCommand;
import frc.robot.commands.ShoulderControl;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class RobotContainer {

    public final XboxController xboxController = new XboxController(0);
    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
    public final ShoulderControl m_shoulderControl = new ShoulderControl(m_shoulderSubsystem,0);
    public final ShoulderControl m_shoulderPreset0deg = new ShoulderControl(m_shoulderSubsystem,0);
    public final ShoulderControl m_shoulderPreset55deg = new ShoulderControl(m_shoulderSubsystem,55);
    public final ShoulderControl m_shoulderPreset110deg = new ShoulderControl(m_shoulderSubsystem,110);


    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem();
    public final ExtendedArmControl m_extendedArmControl = new ExtendedArmControl(m_extendArmSubsystem, 0.0);
    public final ExtendedArmControl m_extendedpresetlength1 = new ExtendedArmControl(m_extendArmSubsystem, 1.0);
    public final ExtendedArmControl m_extendedpresetlength2 = new ExtendedArmControl(m_extendArmSubsystem, 2.0);
    public final ExtendedArmControl m_extendedpresetlength3 = new ExtendedArmControl(m_extendArmSubsystem, -1.0);


    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
    public final GrabberExtendCommand m_grabberExtendControl = new GrabberExtendCommand(m_grabberSubsystem);
    public final GrabberRetractCommand m_grabberRetractControl = new GrabberRetractCommand(m_grabberSubsystem);
    public final GrabberTurnOffCommand m_GrabberTurnOffCommand = new GrabberTurnOffCommand(m_grabberSubsystem);


    public final JoystickButton m_joyA = new JoystickButton(xboxController, 1); // button A
    public final JoystickButton m_joyB = new JoystickButton(xboxController, 2); // button B
    public final JoystickButton m_joyX = new JoystickButton(xboxController, 3); // button X
    public final JoystickButton m_joyY = new JoystickButton(xboxController, 4); // button Y
    public final JoystickButton m_joyLB = new JoystickButton(xboxController, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(xboxController, 6); // Right bumper
    public final JoystickButton m_joyVB = new JoystickButton(xboxController, 7); // View Button
    public final JoystickButton m_joyMB = new JoystickButton(xboxController, 8); // Menu Button

    public final static double buttonMapSwitch = 0;

    public RobotContainer() 
    {
        configureButtonBindings();
    }

    private void configureButtonBindings() 
    {
        m_joyA.onTrue(m_shoulderPreset0deg);
        m_joyB.onTrue(m_shoulderPreset55deg);
        m_joyX.onTrue(m_shoulderPreset110deg);

        m_joyLB.onTrue(m_extendedpresetlength1);
        m_joyRB.onTrue(m_extendedpresetlength3);
    }
}
