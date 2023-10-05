package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object SingletonCommandXboxController: CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
}