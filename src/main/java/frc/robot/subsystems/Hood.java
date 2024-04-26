// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constHood;
import frc.robot.RobotMap.mapHood;
import frc.robot.RobotPreferences.prefHood;

public class Hood extends SubsystemBase {

  CANSparkMax hoodMotor;
  DigitalInput bottomSwitch;
  SparkPIDController hoodPIDController;

  boolean displayOnDashboard;

  public Hood() {

    hoodMotor = new CANSparkMax(mapHood.HOOD_MOTOR_CAN, MotorType.kBrushless);
    bottomSwitch = new DigitalInput(mapHood.HOOD_BOTTOM_SWITCH_DIO);
    hoodPIDController = hoodMotor.getPIDController();

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    hoodMotor.restoreFactoryDefaults();

    hoodPIDController.setP(prefHood.hoodP.getValue());
    hoodPIDController.setI(prefHood.hoodI.getValue());
    hoodPIDController.setD(prefHood.hoodD.getValue());

    hoodMotor.setInverted(constHood.INVERTED);
    hoodMotor.setIdleMode(IdleMode.kCoast);

    hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward,
        (float) SN_Math.degreesToFalcon(prefHood.hoodMaxDegrees.getValue(), constHood.GEAR_RATIO));

    hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse,
        (float) SN_Math.degreesToFalcon(prefHood.hoodMinDegrees.getValue(), constHood.GEAR_RATIO));

    resetAngleToBottom();
  }

  public double getAngleDegrees() {
    return (hoodMotor.getEncoder().getPosition()) * 360;
  }

  public double getRawAngle() {
    return hoodMotor.getEncoder().getPosition();
  }

  public void setAngle(double a_degrees) {

    double degrees = MathUtil.clamp(a_degrees, prefHood.hoodMinDegrees.getValue(), prefHood.hoodMaxDegrees.getValue());

    hoodPIDController.setReference((degrees / 360), CANSparkMax.ControlType.kPosition);
  }

  public void setAngle(SN_DoublePreference degrees) {
    setAngle(degrees.getValue());
  }

  public boolean isBottomSwitch() {
    return !bottomSwitch.get();
  }

  public void resetAngleToBottom() {
    hoodMotor.getEncoder().setPosition((prefHood.hoodMinDegrees.getValue()) / 360);
  }

  public void neutralOutput() {
    hoodMotor.set(0);
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

      SmartDashboard.putNumber("Hood Angle Degrees", getAngleDegrees());
      SmartDashboard.putNumber("Hood Raw Angle", getRawAngle());
      SmartDashboard.putBoolean("Hood Is Bottom Switch", isBottomSwitch());

    }

    if (isBottomSwitch()) {
      resetAngleToBottom();
    }

  }
}
