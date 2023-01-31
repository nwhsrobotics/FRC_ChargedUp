package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    public final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder; // built in NEO encoder (steering)

    public final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPosition(getAbsoluteEncoderRad());

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
 
        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, 0);
        turningPidController.setTolerance(ModuleConstants.kPTolerance);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); //the wheels can rotate in a full circle
        resetEncoders();
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), 0));

    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadRaw() {
        double angle = absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.0); // convert degrees to radians
        return angle; //shorthand for if the encoder is reversed, multiply by -1, else do nothing (multiply by 1)
    }

    public double getAbsoluteEncoderRad() {
        double angle = this.getAbsoluteEncoderRadRaw();
        angle -= absoluteEncoderOffsetRad;
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);

        if (angle > 2*Math.PI) {
            angle -= 2 *Math.PI;
        }

        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad()); //reset turning encoders to the previous value of the absolute encoders
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition())); //SwerveModuleState takes the velocity and the angle of the module for params
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
