from Constants import Constants
import commands2
import commands2.cmd
import commands2.button
from Constants import Constants
from commands2 import (
    Command,
)
from Subsytem import (
    intake,
    elevator,
)


class RobotContainer:
    def __init__(self):
        # Subsystems
        self.intake = intake.intake()
        self.elevator = elevator.elevator()

        # controller
        self.driverController = commands2.button.CommandXboxController(
            Constants.OIConstants.kDriverControllerPort
        )
        self.configure_button_bindings()
        pass

    def configure_button_bindings(self):
        # connecting the button press to the subsystem that was made
        self.driverController.Y().onTrue(  # If the Y buttons is pressed execute ONCE the function inside of the lambda
            commands2.cmd.runOnce(lambda: self.intake.set_motors(0.7), self.intake)
        )
        self.driverController.Y().onFalse(  # If the Y button is no pressed execute ONCE the function inside of the lambda
            commands2.cmd.runOnce(lambda: self.intake.set_motors(0), self.intake)
        )

        self.driverController.A().onTrue(  # if the A button is pressed execute the function inside of the lambda as much as it can
            commands2.cmd.runOnce(lambda: self.elevator.move(0.6, True), self.elevator)
        )
        self.driverController.A().onFalse(  # if the A button is NOT pressed execute ONCE the function inside of the lambda
            commands2.cmd.runOnce(lambda: self.elevator.move(0, True), self.elevator)
        )

        self.driverController.B().onTrue(
            commands2.cmd.runOnce(lambda: self.elevator.move(-0.6, True), self.elevator)
        )
        self.driverController.B().onFalse(
            commands2.cmd.runOnce(lambda: self.elevator.move(0, True), self.elevator)
        )

    def get_autonomous_command(self) -> Command:
        return None


if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
