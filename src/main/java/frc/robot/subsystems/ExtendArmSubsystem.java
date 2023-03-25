package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

public class ExtendArmSubsystem extends SubsystemBase {
  public CANSparkMax m_extendArmMotor1;
  private SparkMaxPIDController m_pidController1;
  private RelativeEncoder m_extendArmEncoder1;
  public DigitalInput input;
  private double m_currentPos_inch = 0.0;
  private double m_desiredPos_inch = 0.0;
  private double m_current_vel_ips = 0.0;
  private static final double GEAR_RATIO = 5.23 * 5.23 * 2.89;
  public static final double INCHES_PER_ROT = 2.0 * 24.0 * 5.0 / (25.4 * GEAR_RATIO); // stages * pulley teeth * mm per
                                                                                      // tooth / (mm per inch * gear
                                                                                      // ratio)
  private static final int HOMING_MAX_TICKS = 150;
  private static final double HOMING_POWER = 0.6;
  public boolean m_homed = false;
  private boolean m_enabled = false;
  private Logger logger = Logger.getInstance();
  private int m_homing_ticks;
  private boolean m_moving = false;
  private final ShoulderSubsystem m_shoulderSubsystem;

  public ExtendArmSubsystem(ShoulderSubsystem shoulder) {
    // TODO CHANGE SPARKMAX CANID TO USE CONSTANTS
    m_extendArmMotor1 = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless);
    m_shoulderSubsystem = shoulder;

    input = new DigitalInput(4);

    if (m_extendArmMotor1 != null) {
      m_extendArmMotor1.setSmartCurrentLimit(10);
      m_pidController1 = m_extendArmMotor1.getPIDController();
      m_extendArmEncoder1 = m_extendArmMotor1.getEncoder();
      // m_extendArmEncoder1.setPosition(2.0 / INCHES_PER_ROT);

      m_pidController1.setP(ExtendArmConstants.kp);

      m_pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      // m_pidController1.setReference(0.0, ControlType.kPosition);
      m_enabled = true;
      System.out.println("YES EXTEND ARM MOTOR");
    }

    else {
      System.out.println("NO EXTEND ARM MOTOR");
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM CURRENT", m_extendArmMotor1.getOutputCurrent());
    SmartDashboard.putBoolean("ARM @ LIMIT SWITCH", input.get());
    if (m_enabled == true) {
      if (m_homed == true) {
        double target_pos = m_desiredPos_inch;
        if(target_pos > m_shoulderSubsystem.getMaxArmExtension()){
          target_pos = m_shoulderSubsystem.getMaxArmExtension();
        }
        
        if (target_pos > ExtendArmConstants.MAX_EXTEND_INCH) { 
          target_pos = ExtendArmConstants.MAX_EXTEND_INCH;
        } else if (target_pos < 0) {
          target_pos = 0;
        }

        double delta = (target_pos - m_currentPos_inch);
        double v = m_current_vel_ips;

        boolean invert = false;

        if (delta < 0.0) {
          invert = true;
          delta = -delta;
          v = -v;
        }

        double t = Math.sqrt(2.0 * delta / ExtendArmConstants.ACCEL_MAX_A_IPS2);

        double speed_limit = ExtendArmConstants.ACCEL_MAX_A_IPS2 * (t - ExtendArmConstants.SECONDS_PER_TICK);

        if (speed_limit > ExtendArmConstants.ACCEL_MAX_V_IPS) {
          speed_limit = ExtendArmConstants.ACCEL_MAX_V_IPS;
        }
        // adjust speed
        if (v >= speed_limit) {
          // slow down
          v -= ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        } else if (v + ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK < speed_limit) {
          // speed up
          v += ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        } else {
          // maintain speed
        }

        double dx = v * ExtendArmConstants.SECONDS_PER_TICK;

        if (invert) {
          m_currentPos_inch += -dx;
          m_current_vel_ips = -v;
        } else {
          m_currentPos_inch += dx;
          m_current_vel_ips = v;
        }

        double newDeltaX = target_pos - m_currentPos_inch;
        if ((Math.abs(m_current_vel_ips) < ExtendArmConstants.MIN_VEL_IPS) &&
            (Math.abs(newDeltaX) < ExtendArmConstants.MIN_X_INCH)) {
          m_currentPos_inch = target_pos;
          m_current_vel_ips = 0;
          m_moving = false;
        } else {
          m_moving = true;
        }

        double position_rot = m_currentPos_inch / INCHES_PER_ROT;
        m_pidController1.setReference(position_rot, ControlType.kPosition);

        logger.recordOutput("arm.rotation", position_rot);
        logger.recordOutput("arm.current", m_extendArmMotor1.getOutputCurrent());
        logger.recordOutput("arm.position.current", m_currentPos_inch);
        logger.recordOutput("arm.position.target", target_pos);
        logger.recordOutput("arm.velocity", m_current_vel_ips);
        logger.recordOutput("arm.homed", m_homed);

        SmartDashboard.putBoolean("ARM HOMED?", m_homed);
        SmartDashboard.putBoolean("ARM MOVING?", m_moving);
      
      } else {
        homing();
        m_moving = true;
      }

    } else {
      m_moving = false;
      return;
    }
  }

  public void setPos_inch(double setPoint) {
    if (m_homed == true) {
      m_desiredPos_inch = setPoint;

      if (m_desiredPos_inch > ExtendArmConstants.MAX_EXTEND_INCH) {
        m_desiredPos_inch = ExtendArmConstants.MAX_EXTEND_INCH;
      }

      if (m_desiredPos_inch < 0.0) {
        m_desiredPos_inch = 0.0;
      }
    } else {
      return;
    }

  }

  public double getDesiredPos_inch() {
    return m_desiredPos_inch;
  }

  public double getCurrentPos_inch() {
    return m_currentPos_inch;
  }

  private void homing() {
    // We are tracking time in homing state.
    m_homing_ticks += 1;

    if (input.get() == false) {
      // REACHED end stop
      stopHoming(0.0);
    } else if (m_homing_ticks > HOMING_MAX_TICKS) {
      // timed out - assume extension is currently at 1 inch
      stopHoming(1.0);
    } else {
      // driving TOWARDS end stop
      m_extendArmMotor1.set(-HOMING_POWER);
    }
  }

  public void startHoming() {
    System.out.println("STARTING HOMING");
    m_homed = false;
    m_homing_ticks = 0;
  }

  public void stopHoming(double position) {
    System.out.println("STOPPING HOMING");
    m_homed = true;
    m_extendArmMotor1.set(0.0);
    m_extendArmEncoder1.setPosition(position / INCHES_PER_ROT);
    m_currentPos_inch = position;
    m_desiredPos_inch = position;
  }

  public boolean isMoving() {
    return m_moving;
  }
}
