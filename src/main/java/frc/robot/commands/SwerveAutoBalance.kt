package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.SwerveModule
import frc.robot.subsystems.SwerveSubsystem

class SwerveAutoBalance(private val swerveSubsystem: SwerveSubsystem) : CommandBase() {
    private val imu get() = swerveSubsystem.m_gyro
    private val controller = PIDController(0.5, 0.0, 0.0) // TODO calibrate

    override fun execute() {
        val adjustment = controller.calculate(imu.pitch.toDouble(), 0.0)

        forEachSwerveModule {
            it.resetTurnEncoder()
            it.driveMotor.set((adjustment / 2.0).coerceIn(-1.0, 1.0))
        }
    }

    private fun forEachSwerveModule(f: (SwerveModule) -> Unit) {
        f(swerveSubsystem.frontLeft)
        f(swerveSubsystem.frontRight)
        f(swerveSubsystem.backLeft)
        f(swerveSubsystem.backRight)
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stopModules()
    }

    override fun isFinished() = false
}
