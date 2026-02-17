// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooterSubsystem extends SubsystemBase {

  // public enum State{
  //   SHOOTING,
  //   IDLE,
  //   BROKEN
  // }

  // private 


  
  private SparkMax shooterSpark;
  private SparkMax Spark2;
  private SparkMax SparkTransfer;

  private SparkMaxConfig shooterConfig;
  private SparkMaxConfig Spark2Config;
  private SparkMaxConfig SparkTransferConfig;

  
  double output = 0.0;
  
  /** Creates a new shooterSubsystem. */
  public shooterSubsystem() {
    shooterSpark = new SparkMax(ShooterConstants.kShooterId, MotorType.kBrushless);
    shooterConfig = new SparkMaxConfig();
    

    Spark2 = new SparkMax(2, MotorType.kBrushless);
    Spark2Config = new SparkMaxConfig();

    SparkTransfer = new SparkMax(3, MotorType.kBrushless);
    SparkTransferConfig = new SparkMaxConfig();
    
    SparkTransferConfig.inverted(false).smartCurrentLimit(50);

    Spark2Config.inverted(false).smartCurrentLimit(50);
  
    shooterConfig
            .inverted(true)
            .smartCurrentLimit(50);

    
    shooterSpark.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Spark2.configure(Spark2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public void setShootingPower(double output){

    this.output = output;

    shooterSpark.set(-output);
    Spark2.set(output);
  
  }

  public void activateTransfer(double output){
    SparkTransfer.set(output);
  }

  public void reverseShoot(){
    ShooterConstants.kShooterReversed = !ShooterConstants.kShooterReversed;
  }

  @Override
  public void periodic() {
    SparkTransfer.set(0.0);
    SmartDashboard.putNumber("Shooting speed", output);
  }
}
