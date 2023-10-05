package frc.robot.subsystems

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.SingletonCommandXboxController
import frc.robot.commands.swerve.TeleopDrive
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import kotlin.math.pow

object SwerveSubsystem : SubsystemBase() {

    private val swerve: SwerveDrive
    val directory: File = File(Filesystem.getDeployDirectory(), "swerve")

    var autoBuilder: SwerveAutoBuilder? = null

    val driverXbox = SingletonCommandXboxController


    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        try {
            swerve = SwerveParser(directory).createSwerveDrive()
        } catch (e: Exception) {
            throw RuntimeException("Failed to create swerve drive", e)
        }

    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean = false, isOpenLoop: Boolean = false) {
        swerve.drive(translation, rotation, fieldRelative, isOpenLoop)
    }

    fun getKinematics(): SwerveDriveKinematics {
        return swerve.kinematics
    }

    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerve.resetOdometry(initialHolonomicPose)
    }

    fun setChassisSpeeds(speeds: ChassisSpeeds) {
        swerve.setChassisSpeeds(speeds)
    }

    fun postTrajectory(trajectory: Trajectory) {
        swerve.postTrajectory(trajectory)
    }

    fun zeroGyro() {
        swerve.zeroGyro()
    }

    fun setMotorBreak(breakMode: Boolean) {
        swerve.setMotorIdleMode(breakMode)
    }

    fun getHeading(): Rotation2d {
        return swerve.getYaw()
    }

    fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return swerve.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().radians)
    }

    fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return swerve.swerveController.getTargetSpeeds(xInput, yInput, angle.radians, getHeading().radians)
    }

    fun getFieldVelocity(): ChassisSpeeds {
        return swerve.getFieldVelocity()
    }

    fun getRobotVelocity(): ChassisSpeeds {
        return swerve.getRobotVelocity()
    }

    fun getSwerveController(): SwerveController {
        return swerve.swerveController
    }

    fun getSwerveDriveConfiguration(): SwerveDriveConfiguration {
        return swerve.swerveDriveConfiguration
    }

    fun lockWheels() {
        swerve.lockPose()
    }

    fun getPitch(): Rotation2d {
        return swerve.getPitch()
    }

    fun createPathPlannerCommand(path: String?, constraints: PathConstraints?, eventMap: Map<String?, Command?>?,
                                 translation: PIDConstants?, rotation: PIDConstants?, useAllianceColor: Boolean): Command {
        val pathGroup = PathPlanner.loadPathGroup(path, constraints)
        if (autoBuilder == null) {
            autoBuilder = SwerveAutoBuilder({ swerve.getPose() }, { pose: Pose2d? -> swerve.resetOdometry(pose) },
                    translation,
                    rotation, { chassisSpeeds: ChassisSpeeds? -> swerve.setChassisSpeeds(chassisSpeeds) },
                    eventMap,
                    useAllianceColor,
                    this
            )
        }
        return autoBuilder!!.fullAuto(pathGroup)
    }

    override fun getDefaultCommand(): Command? {
        val defaultCommand: Command = TeleopDrive(this, { driverXbox.leftX }, { driverXbox.leftY },
                { -driverXbox.rightX },
                { driverXbox.hid.leftBumper },
                false,
                true
        )
        return null
    }




}