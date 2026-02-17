// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstats;
import frc.robot.subsystems.drive.DriveSubsystem;


/** Add your docs here. */
public class Limelight extends SubsystemBase{
    public String LL = "limelight-ll" ;
    
    DriveSubsystem driveSubsystem;
    PIDController zPidController;
    PIDController yPidController;
    PIDController xPidController;
    
    // double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL); // [0] x, [2] y, [4], z/rot





    public Limelight(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        zPidController = new PIDController(0.0125, 0.0, 0.0);
        yPidController = new PIDController(0.035, 0.0, 0.0);
        xPidController = new PIDController(0.035, 0.0, 0.0);

        // xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
        xPidController.setTolerance(LimelightConstats.xPosTolerance);

        xPidController.setSetpoint(LimelightConstats.xPosSetpoint);
        yPidController.setSetpoint(LimelightConstats.xPosSetpoint);

        yPidController.setTolerance(LimelightConstats.yPosTolerance);

        zPidController.setSetpoint(LimelightConstats.rotPosTolerance);
        // zPidController.setTolerance(limelightConstants.rotReefTolerance);
      
        zPidController.enableContinuousInput(-180.0, 180);

        

    }

    @Override
   public void periodic(){
    SmartDashboard.putNumber("Ty", getTx());
SmartDashboard.putNumber("Tx", getTy());
SmartDashboard.putBoolean("HasTarget", hasTarget());
SmartDashboard.putBoolean("isAlligned", isAlligned());
SmartDashboard.putNumber("yError", yPidController.getError());
   } 

    public double getTy(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[0]*10;
    }

    
    public double getTx(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[2]*10;
    }

    public double getRy(){
        double[] positions = LimelightHelpers.getBotPose_TargetSpace(LL);
        return positions[4];
    }
    public double getTa(){
        return LimelightHelpers.getTA(LL);
    }

    public double getTxnc(){
        return NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("txnc").getDouble(0);
    }
    
    public boolean hasTarget(){
        return LimelightHelpers.getTV(LL);
    }

    public int getId() {
        return (int)LimelightHelpers.getFiducialID(LL);
    }
    
    public Command stopCommand(){
        return Commands.run(()->{

            driveSubsystem.drive(0, 0, 0, false);
        }, driveSubsystem);
    }


  
    public void ResetPids(){
        xPidController.reset();
        yPidController.reset();
        zPidController.reset();
    }

    
    public boolean isAlligned(){
        if(xPidController.atSetpoint()
            ){
            return true;
         }else{
                return false;
         }
    }

    public Command AllignXAxis(){

        
        return Commands.run(()->{
            if(hasTarget()){
            double youtput = yPidController.calculate(getTy(), LimelightConstats.xPosSetpoint);
        
            driveSubsystem.drive(0, youtput*0.2 , 0, false);


        }}, driveSubsystem);
    }

}