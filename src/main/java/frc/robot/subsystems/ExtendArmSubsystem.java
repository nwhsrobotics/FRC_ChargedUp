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
  public static final double INCHES_PER_ROT = 2.0 * 24.0 * 5.0 / (25.4 * GEAR_RATIO); // stages * pulley teeth * mm per tooth / (mm per inch * gear ratio)
  private static final int HOMING_MAX_TICKS = 150;
  private static final double HOMING_POWER = 0.6;
  public boolean m_homed = false;
  private boolean m_enabled = false;
  private Logger logger = Logger.getInstance();
  private int m_homing_ticks;
  private boolean m_moving = false;
  private final ShoulderSubsystem m_shoulderSubsystem;

  public ExtendArmSubsystem(ShoulderSubsystem shoulder) {
    //initialize the motors and PID controllers
    m_extendArmMotor1 = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless);
    m_shoulderSubsystem = shoulder;

    //this DigitalInput switch is there for homing only
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
  // This function runs periodically, updating arm position and velocity
  public void periodic() {
    // Send arm's current and limit switch state to SmartDashboard. (Limit switch is for homing)
    SmartDashboard.putNumber("ARM CURRENT", m_extendArmMotor1.getOutputCurrent());
    SmartDashboard.putBoolean("ARM @ LIMIT SWITCH", input.get());

    // If the arm is enabled...
    if (m_enabled == true) {
      // If the arm has been homed...
      if (m_homed == true) {
        // Set the desired position to the target position
        double target_pos = m_desiredPos_inch;

        // Limit the target position to the maximum arm extension
        if (target_pos > m_shoulderSubsystem.getMaxArmExtension()) {
          target_pos = m_shoulderSubsystem.getMaxArmExtension();
        }

        // Limit the target position to the maximum allowed extension distance
        if (target_pos > ExtendArmConstants.MAX_EXTEND_INCH) {
          target_pos = ExtendArmConstants.MAX_EXTEND_INCH;
        } else if (target_pos < 0) {
          target_pos = 0;
        }

        // Calculate the distance to move
        double delta = (target_pos - m_currentPos_inch);
        // Get the current velocity
        double v = m_current_vel_ips;

        // Determine if the arm needs to be inverted to move in the correct direction
        boolean invert = false;

        if (delta < 0.0) {
          invert = true;
          delta = -delta;
          v = -v;
        }

        // Calculate the time it will take to accelerate to maximum velocity
        double t = Math.sqrt(2.0 * delta / ExtendArmConstants.ACCEL_MAX_A_IPS2);

        // Calculate the maximum speed limit based on acceleration and time
        double speed_limit = ExtendArmConstants.ACCEL_MAX_A_IPS2 * (t - ExtendArmConstants.SECONDS_PER_TICK);

        // Limit the maximum speed to the maximum allowed speed
        if (speed_limit > ExtendArmConstants.ACCEL_MAX_V_IPS) {
          speed_limit = ExtendArmConstants.ACCEL_MAX_V_IPS;
        }

        // Adjust the speed based on current velocity and maximum speed limit
        if (v >= speed_limit) {
          // Slow down
          v -= ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        } else if (v + ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK < speed_limit) {
          // Speed up
          v += ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        } else {
          // Maintain speed
        }

        // Calculate the distance to move based on adjusted velocity
        double dx = v * ExtendArmConstants.SECONDS_PER_TICK;

        // Update current position and velocity based on distance moved and direction
        if (invert) {
          m_currentPos_inch += -dx;
          m_current_vel_ips = -v;
        } else {
          m_currentPos_inch += dx;
          m_current_vel_ips = v;
        }

        // Check if the arm has reached the target position and velocity
        double newDeltaX = target_pos - m_currentPos_inch;
        if ((Math.abs(m_current_vel_ips) < ExtendArmConstants.MIN_VEL_IPS) &&
            (Math.abs(newDeltaX) < ExtendArmConstants.MIN_X_INCH)) {
          // Arm has reached target position and velocity, so stop moving
          m_currentPos_inch = target_pos;
          m_current_vel_ips = 0;
          m_moving = false;
        } else {
          // Arm is still moving to target position and velocity
          m_moving = true;
        }

        double position_rot = m_currentPos_inch / INCHES_PER_ROT; // calculate the position in rotations
        m_pidController1.setReference(position_rot, ControlType.kPosition); // set the position reference for the PID controller

        // record output to logger and SmartDashboard
        logger.recordOutput("arm.rotation", position_rot);
        logger.recordOutput("arm.current", m_extendArmMotor1.getOutputCurrent());
        logger.recordOutput("arm.position.current", m_currentPos_inch);
        logger.recordOutput("arm.position.target", target_pos);
        logger.recordOutput("arm.velocity", m_current_vel_ips);
        logger.recordOutput("arm.homed", m_homed);
        SmartDashboard.putBoolean("ARM HOMED?", m_homed);
        SmartDashboard.putBoolean("ARM MOVING?", m_moving);

        // if the arm is not homed, initiate homing and set the arm to moving
      } else {
        homing();
        m_moving = true;
      }

      // if the arm is not enabled i.e. not intialized, set the arm to not moving and return
    } else {
      m_moving = false;
      return;
    }
  }

  public void setPos_inch(double setPoint) {
    // Only allow setting the position if the arm is homed
    if (m_homed == true) {
      // Set the desired position to the new set point
      m_desiredPos_inch = setPoint;

      // Ensure the desired position is within the allowable range
      if (m_desiredPos_inch > ExtendArmConstants.MAX_EXTEND_INCH) {
        m_desiredPos_inch = ExtendArmConstants.MAX_EXTEND_INCH;
      }

      if (m_desiredPos_inch < 0.0) {
        m_desiredPos_inch = 0.0;
      }
    } else {
      // If the arm is not homed, do nothing
      return;
    }
  }

  public double getDesiredPos_inch() {
    // Return the desired position of the arm in inches
    return m_desiredPos_inch;
  }

  public double getCurrentPos_inch() {
    // Return the current position of the arm in inches
    return m_currentPos_inch;
  }

  private void homing() {
    // Increment the homing tick counter
    m_homing_ticks += 1;

    if (input.get() == false) {
      // If the end stop has been reached, stop homing
      stopHoming(0.0);
    } else if (m_homing_ticks > HOMING_MAX_TICKS) {
      // If the maximum homing time has elapsed, assume the arm is at 1 inch and stop homing
      stopHoming(1.0);
    } else {
      // If the end stop has not been reached, drive towards it at a specified power level
      m_extendArmMotor1.set(-HOMING_POWER);
    }
  }

  public void startHoming() {
    // Print a message indicating homing has started
    System.out.println("STARTING HOMING");

    // Mark the arm as not homed and reset the homing tick counter
    m_homed = false;
    m_homing_ticks = 0;
  }

  public void stopHoming(double position) {
    // Print a message indicating homing has stopped
    System.out.println("STOPPING HOMING");

    // Mark the arm as homed, stop the motor, and set the encoder position to the specified position
    m_homed = true;
    m_extendArmMotor1.set(0.0);
    m_extendArmEncoder1.setPosition(position / INCHES_PER_ROT);
    m_currentPos_inch = position;
    m_desiredPos_inch = position;
  }

  public boolean isMoving() {
    // Return whether or not the arm is currently moving
    return m_moving;
  }
}
