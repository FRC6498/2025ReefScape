// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ledcontroller;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final ledcontroller LEDsystem = new ledcontroller();
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
          
    for (int i = 5800; i <= 5809; i++) {
      PortForwarder.add(i, "limelight.local", i); // adds the port to access limelight directly
    }

    Pathfinding.setPathfinder(new LocalADStar());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    ;
    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().schedule(LEDsystem.LEDrainbow());
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LEDPattern base = LEDPattern.solid(Color.kMediumTurquoise);
    if (Alliance.Blue == DriverStation.getAlliance().orElse(Alliance.Red)) {
      CommandScheduler.getInstance().schedule(LEDsystem.LedRun(0,0,255));
      base = LEDPattern.solid(Color.kBlue);
      LEDPattern pattern = base.blink(Seconds.of(1));
      pattern.applyTo(LEDsystem.m_ledBuffer);      
    } else {
      CommandScheduler.getInstance().schedule(LEDsystem.LedRun(255,0,0));
      base = LEDPattern.solid(Color.kRed);
      LEDPattern pattern = base.blink(Seconds.of(1));
      pattern.applyTo(LEDsystem.m_ledBuffer);
    }
  }

  @Override
  public void teleopPeriodic() {
     
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
