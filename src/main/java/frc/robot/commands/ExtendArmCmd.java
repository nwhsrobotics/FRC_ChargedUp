package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
        extendArm.setPos_inch(setPoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
