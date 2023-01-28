package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class Tuner extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private final Joystick m_joy0;
  private PIDController tuningPidController = new PIDController(0.5, 0, 0);

  public Tuner(SwerveSubsystem swerveSubsystem, Joystick m_joy0) {
    this.swerveSubsystem = swerveSubsystem;
    this.m_joy0 = m_joy0;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.straightenModules();
  }

  @Override
  public void execute() {
    if(m_joy0.getRawButton(5)) {
        swerveSubsystem.frontLeft.isTuning = true;
        swerveSubsystem.frontLeft.turningMotor.set(tuningPidController.calculate(swerveSubsystem.frontLeft.getAbsoluteEncoderRad(), swerveSubsystem.frontLeft.getAbsoluteEncoderRad() + Math.PI/4));
    }
  }

  @Override
  public void end(boolean interrupted) {
    for (SwerveModule module : swerveSubsystem.swerveMods) {
        module.isTuning = false;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}