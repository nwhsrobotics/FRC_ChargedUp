package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

public class ExtendArmSubsystem extends SubsystemBase {
  private XboxController xboxController;
  private CANSparkMax m_extendArmMotor1 = null;
  private SparkMaxPIDController m_pidController1 = null;
  private RelativeEncoder m_extendArmEncoder1 = null;
  private static double m_currentPos = 0.0;
  private static double m_desiredPos = 0.0;
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  private static final double TOTAL_DISTANCE = 25.0; // Revisit this value!!! 100 would be best for 2.4 seconds and 50 would be 4.8 seconds 150 for 1.6 seconds
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
  private double m_gearRatio = 18.9;
  private double m_oneRotationLength = 1.504; // in inches
  private boolean m_enabled = false;
  private double currentLimit = 0.0;

  /** Creates a new ExtendArmSubsystem. */
  public ExtendArmSubsystem(XboxController m_controller) {
    xboxController = m_controller;
    m_extendArmMotor1 = new CANSparkMax(ExtendArmConstants.ExtendArmCanID24, CANSparkMax.MotorType.kBrushless);

    while(m_extendArmMotor1.getOutputCurrent() < currentLimit) {
      m_extendArmMotor1.set(-0.2);
    }
    if(m_extendArmMotor1.getOutputCurrent() >= currentLimit) {
          m_extendArmMotor1.stopMotor();
    }

    if (m_extendArmMotor1 != null) {
      m_extendArmMotor1.setSmartCurrentLimit(25);
      m_pidController1 = m_extendArmMotor1.getPIDController();
      m_extendArmEncoder1 = m_extendArmMotor1.getEncoder();
      m_extendArmEncoder1.setPosition(0);

      m_pidController1.setP(ExtendArmConstants.kp);
      m_pidController1.setI(ExtendArmConstants.ki);
      m_pidController1.setD(ExtendArmConstants.kd);
      m_pidController1.setIZone(ExtendArmConstants.kIz);
      m_pidController1.setFF(ExtendArmConstants.kFFz);

      m_pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      m_pidController1.setReference(0.0, ControlType.kPosition);
      System.out.println("ExtendArmMotor1 initialized");
      m_enabled = true;
    }
  }

  public void setPos(double p_distInches) {
    m_desiredPos = (((p_distInches / 2) / m_oneRotationLength) * m_gearRatio);      //basically takes the user input for inches and converts it to the first part of arm rotations because the caluclations are relative to the first part of the arm because the third part moves relatively to the first part and moves twice the distance of the first part of the arm with the same motor
  }

  @Override
  public void periodic() {
    if (m_enabled == true) {
      if (xboxController.getPOV() == 90) {
        setPos(m_desiredPos + (((4 / 2) / m_oneRotationLength) * m_gearRatio));     //take the arm exactly by 4 inches forward when RIGHT D-Pad button pressed
      } else if (xboxController.getPOV() == 270) {
        setPos(m_desiredPos - (((4 / 2) / m_oneRotationLength) * m_gearRatio));     //take the arm exactly by 4 inches backwars when LEFT D-Pad button pressed
      }
      if ((m_desiredPos > (((38 / 2) / m_oneRotationLength) * m_gearRatio))) {      //if desired pos for arm is greater than 38 make it 38 and if less than 0 inches make it 0
        m_desiredPos = (((38 / 2) / m_oneRotationLength) * m_gearRatio);
      } else if (m_desiredPos < (((0 / 2) / m_oneRotationLength) * m_gearRatio)) {
        m_desiredPos = (((0 / 2) / m_oneRotationLength) * m_gearRatio);
      }

      double distance = (m_desiredPos - m_currentPos);
      double delta = distance;

      if (delta > SPEED_ROT_PER_TICK) {
        delta = SPEED_ROT_PER_TICK;
      } else if (delta < -SPEED_ROT_PER_TICK) {
        delta = -SPEED_ROT_PER_TICK;
      }

      m_currentPos += delta;

      m_pidController1.setReference(m_currentPos, ControlType.kPosition);


      Logger logger = Logger.getInstance();

      logger.recordOutput("Arm Motor Rotations", m_extendArmEncoder1.getPosition());
      logger.recordOutput("Arm Inches Extended", (m_extendArmEncoder1.getPosition() / m_gearRatio)* m_oneRotationLength * 2);
      //SmartDashboard.putNumber("Arm Motor Rotations", m_extendArmEncoder1.getPosition());
      //SmartDashboard.putNumber("Arm Inches Extended", (m_extendArmEncoder1.getPosition() / m_gearRatio)* m_oneRotationLength * 2);
    } else {
      return;
    }
  }
}
