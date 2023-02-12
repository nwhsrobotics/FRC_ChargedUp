package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

// ExtendArmSubsystem initializes and sets up two brushless motors and their associated encoders and PID controllers
public class ExtendArmSubsystem extends SubsystemBase {

  // Declare two instances of the CANSparkMax motor controller class
  private CANSparkMax extendArmMotor1 = null;
  // Declare two instances of the SparkMaxPIDController class
  private SparkMaxPIDController pidController1 = null;
  // Declare two instances of the RelativeEncoder class
  private RelativeEncoder extendArmEncoder1 = null; 
  // Set the current position for the shoulder motors
  public static double currentPos = 0.0;
  // Set the desired position that user wants to go to for the shoulder motors
  public static double desiredPos = 0.0;

  // Set the number of ticks per second
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  // Rotations it can make (Total_Distance) it can travel in one second
  private static final double TOTAL_DISTANCE = 25.0; // Revisit this value!!!
  // Set the time it needs to reach the destination
  private static double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  // Calculate the speed of rotation per tick (distance traveled per tick )
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);

  private double oldPos = 0.0;
  private double oldDelta = 0.0;
  private double oldCurrentDistance = 0.0;
  private double oldDesiredDistance = 0.0;

  private double gear_ratio = 48.0;

  private double one_rotation_length = 1.0; // Revisit this values!!!

  private boolean m_enabled = false;

  /** Creates a new ExtendArmSubsystem. */
  public ExtendArmSubsystem() {
    // Initialize the first motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID20
    extendArmMotor1 = new CANSparkMax(ExtendArmConstants.ExtendArmCanID24, CANSparkMax.MotorType.kBrushless);
    // checking if the shoulder motor instance is not null
    if (extendArmMotor1 != null) {
      // getting PIDController instance from the shoulder motor
      pidController1 = extendArmMotor1.getPIDController();
      // getting the encoder instance from the shoulder motor
      extendArmEncoder1 = extendArmMotor1.getEncoder();
      // setting the encoder position to zero
      extendArmEncoder1.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      pidController1.setP(ExtendArmConstants.kp);
      pidController1.setI(ExtendArmConstants.ki);
      pidController1.setD(ExtendArmConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      pidController1.setIZone(ExtendArmConstants.kIz);
      pidController1.setFF(ExtendArmConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController1.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 1
      System.out.println("ShoulderMotor1 initialized");
      m_enabled = true;
    }

  }

  public void setPos(double position) {
    desiredPos = ((position / one_rotation_length) * gear_ratio);

    System.out.println("desiredPos: " + desiredPos);
  }

  @Override
  public void periodic() {
    if (m_enabled == true) {
      // This method is called once per scheduler run. It is used to periodically update the motor position to match the desired position.

      // Calculate the difference between the desired position and the current position
      double distance = (desiredPos - currentPos);

      // Store the difference in a variable named delta
      double delta = distance;

      // Check if delta is greater than the maximum speed(max distance it can travel in a tick) (SPEED_ROT_PER_TICK)
      if (delta > SPEED_ROT_PER_TICK) {
        // If delta is greater than the maximum speed(max distance it can travel in a tick), set delta to the maximum speed(max distance it can travel in a tick)
        delta = SPEED_ROT_PER_TICK;
      }

      // Check if delta is less than the negative of the maximum speed(max distance it can travel in a tick)
      if (delta < -SPEED_ROT_PER_TICK) {
        // If delta is less than the negative of the maximum speed(max rotation it can travel in a tick), set delta to the negative of the maximum speed(max distance it can travel in a tick)
        delta = -SPEED_ROT_PER_TICK;
      }

      // Update the current position by adding delta
      currentPos += delta;

      // Set the reference position for the 2 PID controllers in two opposite directions
      pidController1.setReference(currentPos, ControlType.kPosition);

      if ((oldPos != currentPos) || (oldDelta != delta) || (oldDesiredDistance != desiredPos) || (oldCurrentDistance != distance)) {
        System.out.println("Current position: " + currentPos);
        System.out.println("Delta: " + delta);
        System.out.println("Distance: " + distance);
        System.out.println("Desired position: " + desiredPos);
      }

      oldPos = currentPos;
      oldDesiredDistance = desiredPos;
      oldDelta = delta;
      oldCurrentDistance = distance;
    } 
    else
    {
      return;
    }
  }
}
