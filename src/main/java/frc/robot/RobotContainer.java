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
    
    
    public RobotContainer() 
    {
        configureButtonBindings();
    }

    private void configureButtonBindings() 
    {
    }
}
