// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {

  }

  public Optional<LimelightHelpers.PoseEstimate> getBotPose() {
    //get the latest estimate from the limelight

    if (Robot.isSimulation()) return Optional.empty();
    // return Optional.empty();

    Optional<LimelightHelpers.PoseEstimate> optinalEst = LimelightHelpers
        .getBotPoseEstimate_wpiBlue(Constants.VisionConstants.LIMELIGHT_NAME);
    
    if (optinalEst.isEmpty()) return Optional.empty();

    LimelightHelpers.PoseEstimate est = optinalEst.get();

    boolean doRejectUpdate = false;
    //cases when we should reject the latest update 
    if (est.tagCount == 1 && est.rawFiducials.length == 1) {
      SmartDashboard.putNumber("tag0 dist", est.rawFiducials[0].distToCamera);
      if (est.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (est.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    if (est.tagCount == 0) {
      doRejectUpdate = true;
    }


    // return an empty optional if the update should be rejected 
    // return an optional that contains the update if the update is valid
    Optional<LimelightHelpers.PoseEstimate> sentEstimate = !doRejectUpdate ? Optional.of(est) : Optional.empty();
    return sentEstimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
