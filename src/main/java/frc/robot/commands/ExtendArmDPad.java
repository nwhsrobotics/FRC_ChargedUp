package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

public class ExtendArmDPad extends CommandBase {

    private final ExtendArmSubsystem extendArm;
    private final XboxController m_operator;

    public ExtendArmDPad(ExtendArmSubsystem extendArm, XboxController m_operator) {
        this.extendArm = extendArm;
        this.m_operator = m_operator;
        addRequirements(extendArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_operator.getPOV() == 90) {
            double currentInch =  extendArm.getPos_inch();
            extendArm.setPos_inch(currentInch + ExtendArmConstants.EXTEND_SPEED_IPS * ExtendArmConstants.SECONDS_PER_TICK);
        }
        if (m_operator.getPOV() == 270) {
            double currentInch =  extendArm.getPos_inch();
            extendArm.setPos_inch(currentInch - (ExtendArmConstants.EXTEND_SPEED_IPS * ExtendArmConstants.SECONDS_PER_TICK));     //take the arm exactly by 4 inches backward when LEFT D-Pad button pressed
        }    
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
