// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

public class Drivetrain extends SubsystemBase {
  CANSparkMax leftLead;
  CANSparkMax leftFollow;
  CANSparkMax rightLead;
  CANSparkMax rightFollow;

  AHRS navx;

  Field2d field = new Field2d();

  SparkPIDController config;

  double arcadeDriveSpeedMultiplier;
  double arcadeDriveTurnMultiplier;
  boolean displayOnDashboard;

  Trajectory TRAJ_T1toB1thenB2; // tarmac 1 to ball 1 then ball 2
  Trajectory TRAJ_B2toB4andB5; // ball 2 to ball 4 and ball 5 (ball 2 to terminal)
  Trajectory TRAJ_B4toB2; // ball 4 to ball 2 (terminal to ball 2)
  Trajectory TRAJ_F1toB1; // fender 1 to ball 1
  Trajectory TRAJ_T4toB3; // tarmac 4 to ball 3
  Trajectory TRAJ_B3toRB3toB3; // ball 3 to red ball 3 to ball 3 (in a circle)
  Trajectory TRAJ_TestAuto; // straight forward for two meters
  Trajectory TRAJ_T3toTaxi; // tarmac 3 with simple taxi
  Trajectory TRAJ_T2toB2;
  Trajectory TRAJ_FendertoB2;
  Trajectory TRAJ_B2toFender;

  public Drivetrain() {

    leftLead = new CANSparkMax(mapDrivetrain.LEFT_LEAD_MOTOR_CAN, MotorType.kBrushless);
    leftFollow = new CANSparkMax(mapDrivetrain.LEFT_FOLLOW_MOTOR_CAN, MotorType.kBrushless);
    rightLead = new CANSparkMax(mapDrivetrain.RIGHT_LEAD_MOTOR_CAN, MotorType.kBrushless);
    rightFollow = new CANSparkMax(mapDrivetrain.RIGHT_FOLLOW_MOTOR_CAN, MotorType.kBrushless);

    navx = new AHRS();

    arcadeDriveSpeedMultiplier = prefDrivetrain.driveArcadeSpeedMid.getValue();
    arcadeDriveTurnMultiplier = prefDrivetrain.driveArcadeTurnMid.getValue();

    displayOnDashboard = true;

    field = new Field2d();

    config = leftLead.getPIDController();
    config = leftFollow.getPIDController();
    config = rightLead.getPIDController();
    config = rightFollow.getPIDController();

    configure();
    loadTrajectories();
  }

  public void configure() {
    leftLead.restoreFactoryDefaults();
    leftFollow.restoreFactoryDefaults();
    rightLead.restoreFactoryDefaults();
    rightFollow.restoreFactoryDefaults();

    config.setP(prefDrivetrain.driveP.getValue());
    config.setI(prefDrivetrain.driveI.getValue());
    config.setD(prefDrivetrain.driveD.getValue());

    leftLead.setInverted(constDrivetrain.LEFT_INVERTED);
    leftFollow.setInverted(constDrivetrain.LEFT_INVERTED);
    rightLead.setInverted(constDrivetrain.RIGHT_INVERTED);
    rightFollow.setInverted(constDrivetrain.RIGHT_INVERTED);

    leftLead.setIdleMode(IdleMode.kBrake);
    leftFollow.setIdleMode(IdleMode.kBrake);
    rightLead.setIdleMode(IdleMode.kBrake);
    rightFollow.setIdleMode(IdleMode.kBrake);

    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);

  }

  public double getLeftMeters() {
    double falconCounts = leftLead.getEncoder().getPosition();
    double falconRotations = falconCounts / SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT;
    double wheelRotations = falconRotations / constDrivetrain.GEAR_RATIO;
    double meters = wheelRotations * constDrivetrain.WHEEL_CIRCUMFERENCE;

    return meters;
  }

  public double getRightMeters() {
    double falconCounts = rightLead.getEncoder().getPosition();
    double falconRotations = falconCounts / SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT;
    double wheelRotations = falconRotations / constDrivetrain.GEAR_RATIO;
    double meters = wheelRotations * constDrivetrain.WHEEL_CIRCUMFERENCE;

    return meters;
  }

  public void resetEncoderCounts() {
    leftLead.getEncoder().setPosition(0);
    rightLead.getEncoder().setPosition(0);
  }

  public void resetPose(Pose2d pose) {
    resetEncoderCounts();
  }

  public void setArcadeDriveSpeedMultiplier(SN_DoublePreference multiplier) {
    arcadeDriveSpeedMultiplier = multiplier.getValue();
  }

  public void setArcadeDriveTurnMultiplier(SN_DoublePreference multiplier) {
    arcadeDriveTurnMultiplier = multiplier.getValue();
  }

  public void arcadeDrive(double speed, double turn) {
    leftLead.set((speed * arcadeDriveSpeedMultiplier) - (+turn * arcadeDriveTurnMultiplier));
    rightLead.set((speed * arcadeDriveSpeedMultiplier) - (-turn * arcadeDriveTurnMultiplier));
  }

  public void driveSpeed(double leftVelocity, double rightVelocity) {
    leftLead.set(leftVelocity);
    rightLead.set(rightVelocity);

  }

  private void loadTrajectories() {
    try {

      TRAJ_T1toB1thenB2 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_T1toB1thenB2);
      TRAJ_B2toB4andB5 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_B2toB4andB5);
      TRAJ_B4toB2 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_B4toB2);
      TRAJ_F1toB1 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_F1toB1);
      TRAJ_T4toB3 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_T4toB3);
      TRAJ_B3toRB3toB3 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_B3toRB3toB3);
      TRAJ_TestAuto = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_TESTAUTO);
      TRAJ_T3toTaxi = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_T3toTaxi);
      TRAJ_T2toB2 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_T2toB2);
      TRAJ_B2toFender = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_B2toFender);
      TRAJ_FendertoB2 = TrajectoryUtil.fromPathweaverJson(constDrivetrain.PATH_FendertoB2);

    } catch (Exception e) {

      DriverStation.reportError("Unable to open trajectory: ", e.getStackTrace());

    }
  }

  public enum AutoPath {
    T1toB1thenB2,
    B2toB4andB5,
    B4toB2,
    F1toB1,
    T4toB3,
    B3toRB3toB3,
    TestAuto,
    T3toTaxi,
    T2toB2,
    FendertoB2,
    B2toFender
  }

  public Trajectory getTrajectory(AutoPath trajectory) {
    switch (trajectory) {

      case T1toB1thenB2:
        return TRAJ_T1toB1thenB2;
      case B2toB4andB5:
        return TRAJ_B2toB4andB5;
      case B4toB2:
        return TRAJ_B4toB2;
      case F1toB1:
        return TRAJ_F1toB1;
      case T4toB3:
        return TRAJ_T4toB3;
      case B3toRB3toB3:
        return TRAJ_B3toRB3toB3;
      case TestAuto:
        return TRAJ_TestAuto;
      case T3toTaxi:
        return TRAJ_T3toTaxi;
      case T2toB2:
        return TRAJ_T2toB2;
      case FendertoB2:
        return TRAJ_FendertoB2;
      case B2toFender:
        return TRAJ_B2toFender;

      default:
        return null;
    }
  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {

    if (displayOnDashboard) {
      SmartDashboard.putData(field);

      SmartDashboard.putNumber("Drivetrain Left Encoder", leftLead.getEncoder().getPosition());
      SmartDashboard.putNumber("Drivetrain Right Encoder", rightLead.getEncoder().getPosition());

      SmartDashboard.putNumber("Drivetrain Left Meters", getLeftMeters());
      SmartDashboard.putNumber("Drivetrain Right Meters", getRightMeters());

      SmartDashboard.putNumber("Drivetrain Heading Degrees", navx.getRotation2d().getDegrees());

    }
  }
}
