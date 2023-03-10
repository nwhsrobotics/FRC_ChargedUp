package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;

public class ExtendArmCmd extends CommandBase {

    private final ExtendArmSubsystem extendArm;
    private final double setPoint;

    public ExtendArmCmd(ExtendArmSubsystem extendArm, double setPoint) {
        this.extendArm = extendArm;
        this.setPoint = setPoint;
        addRequirements(extendArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(extendArm.m_homed == true)
        {
            extendArm.setPos_inch(setPoint);
        }
        else{
            return;
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
