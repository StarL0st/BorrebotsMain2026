// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.shooter.shooterSubsystem;

/** Add your docs here. */
public class RobotContainer {

    DriveSubsystem Chassis = new DriveSubsystem();
    shooterSubsystem shooter = new shooterSubsystem();
    Limelight limelight = new Limelight(Chassis);
//  private final SendableChooser<Command> autoChooser;

    CommandXboxController m_Controller = new CommandXboxController(0);
    boolean isCompetition = false;

    public RobotContainer(){

        configureButtons();

        Chassis.setDefaultCommand(new RunCommand(()-> Chassis.drive(
        MathUtil.applyDeadband(-m_Controller.getLeftX()*0.2, ControllerConstants.controlDeadband),
        MathUtil.applyDeadband(m_Controller.getLeftY()*0.2, ControllerConstants.controlDeadband),
        MathUtil.applyDeadband(-m_Controller.getRightX()*0.2, ControllerConstants.controlDeadband),
        DriveConstants.kfieldRelative), Chassis));
 

        shooter.setDefaultCommand(new RunCommand(()-> shooter.setShootingPower(m_Controller.getRightTriggerAxis()), shooter));

   

 
        //   autoChooser = AutoBuilder.buildAutoChooser();

        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtons(){

        m_Controller.start().onTrue(Chassis.changeDrivingMode());

        m_Controller.leftBumper().onChange(Chassis.changeSpeed());

        // m_Controller.rightBumper().whileTrue(limelight.AllignXAxis());

        m_Controller.leftTrigger(0.05).whileTrue(new RunCommand(()->shooter.activateTransfer(m_Controller.getLeftTriggerAxis()), shooter)) ;
    }

    public Command getAutonomousCommand(){


        return null;
    }

    public void shuffleboardData(){
    
    }

}
