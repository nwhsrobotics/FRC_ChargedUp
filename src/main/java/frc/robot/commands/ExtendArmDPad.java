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
        // If the extend arm subsystem is homed
        if (extendArm.m_homed == true) {
            // If the right DPAD is pressed
            if (m_operator.getPOV() == 90) {
                double currentInch = extendArm.getDesiredPos_inch();
                extendArm.setPos_inch(
                        currentInch + ExtendArmConstants.ACCEL_MAX_V_IPS / 2 * ExtendArmConstants.SECONDS_PER_TICK);
            }
            // If the left DPAD is pressed
            if (m_operator.getPOV() == 270) {
                double currentInch = extendArm.getDesiredPos_inch();
                extendArm.setPos_inch(
                        currentInch - (ExtendArmConstants.ACCEL_MAX_V_IPS / 2 * ExtendArmConstants.SECONDS_PER_TICK));
            }
        }
        // If the extend arm subsystem is not homed then cancel and return
        else {
            return;
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
