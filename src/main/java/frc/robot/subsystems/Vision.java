// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {

  }

  public Optional<LimelightHelpers.PoseEstimate> getBotPose() {
    //get the latest estimate from the limelight
    LimelightHelpers.PoseEstimate est = LimelightHelpers
        .getBotPoseEstimate_wpiBlue(Constants.VisionConstants.LIMELIGHT_NAME);
    
    boolean doRejectUpdate = false;
    //cases when we should reject the latest update 
    if (est.tagCount == 1 && est.rawFiducials.length == 1) {
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
    //return an empty optional if the update should be rejected 
    //return an optional that contains the update if the update is valid
    Optional<LimelightHelpers.PoseEstimate> sentEstimate = !doRejectUpdate ? Optional.of(est) : Optional.empty();
    return sentEstimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
