// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.preferences.SN_DoublePreference;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {

  CANSparkMax turretMotor;
  SparkPIDController turretPIDController;

  boolean displayOnDashboard;
  final double conversionFactor = 360 / constTurret.GEAR_RATIO;

  public Turret() {

    turretMotor = new CANSparkMax(mapTurret.TURRET_MOTOR_CAN, MotorType.kBrushless);
    turretPIDController = turretMotor.getPIDController();

    displayOnDashboard = true;

    configure();
  }

  public void configure() {

    turretPIDController.setP(prefTurret.turretP.getValue());
    turretPIDController.setI(prefTurret.turretI.getValue());
    turretPIDController.setD(prefTurret.turretD.getValue());

    turretMotor.restoreFactoryDefaults();

    turretMotor.setInverted(constTurret.INVERTED);

    turretMotor.getEncoder().setPositionConversionFactor(conversionFactor);

    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward,
        (float) prefTurret.turretMaxDegrees.getValue());

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse,
        (float) prefTurret.turretMinDegrees.getValue());

    turretMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the turret angle in degrees. Uses positional closed loop control.
   * 
   * @param degrees Degree count to set turret to
   */
  public void setAngle(double degrees) {
    turretPIDController.setReference(degrees, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Sets the turret angle in degrees. Uses positional closed loop control.
   * 
   * @param degrees Degree count to set turret to
   */
  public void setAngle(SN_DoublePreference degrees) {
    setAngle(degrees.getValue());
  }

  /**
   * Gets the turret angle in degrees
   * 
   * @return Turret angle in degrees
   */
  public double getAngleDegrees() {
    return turretMotor.getEncoder().getPosition();
  }

  public double getRawAngle() {
    return turretMotor.getEncoder().getPosition();
  }

  public void setSpeed(double speed) {
    turretMotor.set(speed);
  }

  public void resetEncoderCounts() {
    turretMotor.getEncoder().setPosition(prefTurret.turretFacingTowardsIntakeDegrees.getValue());
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

      SmartDashboard.putNumber("Turret Angle Degrees", getAngleDegrees());
      SmartDashboard.putNumber("Turret Raw Angle", getRawAngle());

    }

  }
}
