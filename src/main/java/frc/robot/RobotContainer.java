// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_DualActionStick;
import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.utils.SN_InstantCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AimState;
import frc.robot.Constants.CargoState;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefPreset;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.DiscardCargo;
import frc.robot.commands.Cargo.ShootCargo;
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Turret.MoveTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class RobotContainer {

  // Controllers
  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER_CONTROLLER);
  private final SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(
      prefDrivetrain.driveSlewRateLimit.getValue());
  private final SN_DualActionStick conOperator = new SN_DualActionStick(mapControllers.OPERATOR_CONTROLLER);

  // Subsystems
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Hood subHood = new Hood();
  private final Intake subIntake = new Intake();
  private final Shooter subShooter = new Shooter();
  private final Transfer subTransfer = new Transfer();
  private final Turret subTurret = new Turret();
  private final Climber subClimber = new Climber();

  // Commands
  private final ShootCargo comShootCargo = new ShootCargo(subShooter, subTransfer);
  private final CollectCargo comCollectCargo = new CollectCargo(subIntake, subTransfer);
  private final DiscardCargo comDiscardCargo = new DiscardCargo(subIntake, subTransfer);

  private final MoveTurret comMoveTurret = new MoveTurret(subTurret, conOperator);
  private final MoveClimber comMoveClimber = new MoveClimber(subClimber, subTurret, conDriver);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static CargoState cargoState;
  public static AimState aimState;

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new RunCommand(
            () -> subDrivetrain.arcadeDrive(
                driveSlewRateLimiter.calculate(conDriver.getArcadeMove()), conDriver.getArcadeRotate()),
            subDrivetrain));

    subClimber.setDefaultCommand(comMoveClimber);

    cargoState = CargoState.NONE;
    aimState = AimState.NONE;

    configureButtonBindings();
    configureDashboardButtons();
  }

  private void configureButtonBindings() {

    // Driver Commands

    // Driving
    conDriver.btn_LBump
        .onTrue(Commands.runOnce(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedLow)))
        .onFalse(
            Commands.runOnce(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid)));
    conDriver.btn_RBump
        .onTrue(
            Commands.runOnce(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedHigh)))
        .onFalse(
            Commands.runOnce(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid)));

    // Prep Climb
    conDriver.btn_Back
        .onTrue(Commands.runOnce(() -> subShooter.neutralOutput()))
        .onTrue(Commands.runOnce(() -> subTurret.setAngle(prefTurret.turretMinDegrees)))
        .onTrue(Commands.runOnce(() -> subHood.neutralOutput()));

    // Pose resetting
    conDriver.POV_North.onTrue(Commands.runOnce(
        () -> subDrivetrain
            .resetPose(constField.LEFT_FENDER_POSITION_FRONT)));
    conDriver.POV_South.onTrue(Commands.runOnce(
        () -> subDrivetrain
            .resetPose(constField.LEFT_FENDER_POSITION_BACK)));
    conDriver.POV_West.onTrue(Commands.runOnce(
        () -> subDrivetrain
            .resetPose(constField.RIGHT_FENDER_POSITION_FRONT)));
    conDriver.POV_East.onTrue(Commands.runOnce(
        () -> subDrivetrain
            .resetPose(constField.RIGHT_FENDER_POSITION_BACK)));

    // Operator Commands

    // Shooting
    conOperator.btn_RTrig
        .whileTrue(comShootCargo);

    conOperator.btn_RBump.onTrue(new RunCommand(() -> subShooter.setMotorSpeedToGoalSpeed(), subShooter));
    conOperator.btn_Start.onTrue(new InstantCommand(() -> subShooter.neutralOutput(), subShooter));

    // Turret
    conOperator.btn_LBump.whileTrue(comMoveTurret);
    conOperator.btn_LStick
        .onTrue(Commands.runOnce(() -> subTurret.setAngle(prefTurret.turretFacingTowardsIntakeDegrees)));
    conOperator.btn_RStick
        .onTrue(Commands.runOnce(() -> subTurret.setAngle(prefTurret.turretFacingAwayFromIntakeDegrees)));

    // Intake

    conOperator.btn_LTrig.whileTrue(comCollectCargo);
    conOperator.btn_B.whileTrue(comDiscardCargo);

    // Presets
    conOperator.POV_North
        .onTrue(Commands.runOnce(() -> subShooter.setGoalSpeed(prefPreset.presetFenderShooterSpeed)))
        .onTrue(Commands.runOnce(() -> subHood.setAngle(prefPreset.presetFenderHoodDegrees)));

    conOperator.POV_South
        .onTrue(Commands.runOnce(() -> subShooter.setGoalSpeed(prefPreset.presetLaunchpadShooterSpeed)))
        .onTrue(Commands.runOnce(() -> subHood.setAngle(prefPreset.presetLaunchpadHoodDegrees)));

    conOperator.POV_West
        .onTrue(Commands.runOnce(() -> subShooter.setGoalSpeed(prefPreset.presetTarmacShooterSpeed)))
        .onTrue(Commands.runOnce(() -> subHood.setAngle(prefPreset.presetTarmacHoodDegrees)));
  }

  private void configureDashboardButtons() {

    SmartDashboard.putData(
        "Configure Climber", new SN_InstantCommand(subClimber::configure, true, subClimber));
    SmartDashboard.putData(
        "Configure Drivetrain", new SN_InstantCommand(subDrivetrain::configure, true, subDrivetrain));
    SmartDashboard.putData(
        "Configure Hood", new SN_InstantCommand(subHood::configure, true, subHood));
    SmartDashboard.putData(
        "Configure Intake", new SN_InstantCommand(subIntake::configure, true, subIntake));
    SmartDashboard.putData(
        "Configure Shooter", new SN_InstantCommand(subShooter::configure, true, subShooter));
    SmartDashboard.putData(
        "Configure Transfer", new SN_InstantCommand(subTransfer::configure, true, subTransfer));
    SmartDashboard.putData(
        "Configure Turret", new SN_InstantCommand(subTurret::configure, true, subTurret));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
