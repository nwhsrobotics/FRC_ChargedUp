// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPoses;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MoveToPoseCmd extends CommandBase {
  /** Creates a new MoveToPoseCmd. */
  private ShoulderSubsystem m_shoulder;
  private ExtendArmSubsystem m_arm;
  private WristSubsystem m_wrist;
  private int m_targetPose;

  static class PoseData {
    private double shoulder_deg;
    private double arm_inches;
    private double pitch_deg;

    public PoseData(
      double shoulder_deg, double arm_inches, double pitch_deg
    ) {
      this.shoulder_deg = shoulder_deg;
      this.arm_inches = arm_inches;
      this.pitch_deg = pitch_deg;
    }
  }

  private static final PoseData[] POSES = {
    null,
    new PoseData(-100.0, 0.0, 100.0), // in_robot, wrist level
    new PoseData(-70.0, 0.0, 70.0 + 15.0), // over bumper, wrist pitched up 15 deg
    new PoseData(-60.0, 6.0, 60.0), // ground,
    new PoseData(-30.0, 10.0, 30.0), // medium
    new PoseData(0, 20.0, 0.00), // high
    new PoseData(20.0, 10.0, -20.0), // shelf
  };

  public MoveToPoseCmd(
    ShoulderSubsystem shoulderSubsystem, 
    ExtendArmSubsystem extendArmSubsystem,
    WristSubsystem wristSubsystem,
    int targetPose
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulderSubsystem, extendArmSubsystem, wristSubsystem);
    m_shoulder = shoulderSubsystem;
    m_arm = extendArmSubsystem;
    m_wrist = wristSubsystem;
    m_targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Construct sequential command to perform component moves
    SequentialCommandGroup command = new SequentialCommandGroup();
    int currentPose = m_shoulder.getCurrentPose();

    if (m_targetPose == ArmPoses.POSE_1_IN_ROBOT || currentPose == ArmPoses.POSE_1_IN_ROBOT) {
      PoseData poseData = POSES[ArmPoses.POSE_1_IN_ROBOT];

      ParallelCommandGroup toBumper = new ParallelCommandGroup(
        new ShoulderCmd(m_shoulder, poseData.shoulder_deg),
        new ExtendArmCmd(m_arm, poseData.arm_inches),
        new WristPitchRollCmd(m_wrist, poseData.pitch_deg, 0.0)
      );
      command.addCommands(toBumper);
    }
    
    final PoseData targetPoseData = POSES[m_targetPose];
    ParallelCommandGroup toDestination = new ParallelCommandGroup(
      new ShoulderCmd(m_shoulder, targetPoseData.shoulder_deg),
      new ExtendArmCmd(m_arm, targetPoseData.arm_inches),
      new WristPitchRollCmd(m_wrist, targetPoseData.pitch_deg, 0.0)
    );
    command.addCommands(toDestination);
    command.addCommands(new ShoulderUpdatePoseCmd(m_shoulder, m_targetPose));
    command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
