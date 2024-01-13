// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  public Drivetrain Drivetrain;
  public PowerDistribution Pdp;
  public CommandJoystick DriverController;

  public RobotContainer() {
    Pdp = new PowerDistribution();
    Pdp.resetTotalEnergy();
    Pdp.clearStickyFaults();
    SmartDashboard.putData(Pdp);

    Drivetrain = new Drivetrain(DriveMap.DrivetrainConfig);
    Drivetrain.register();
    SmartDashboard.putData(Drivetrain);
    configureBindings();
  }

  private void configureBindings() {
    DriverController = new CommandJoystick(0);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
