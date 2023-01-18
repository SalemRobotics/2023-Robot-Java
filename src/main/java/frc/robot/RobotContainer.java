// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.FileSystems;
import java.util.Scanner;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}
    
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Method to read constants from a json file and assign them to a SparkMaxPIDController object. 
   * @param groupname The name of the PID constant json object.
   * @param controller The reference variable of the SparkMaxPIDController object.
   */
  public static void readPIDConstants(String objname, SparkMaxPIDController controller) {
    /* 
    TODO: finish file parsing; needs to read all the other constants, 
    and also the other constants need to be defined in the file. 
    */
    try (Scanner scan = new Scanner(new File("constants/PIDConstants.json"))) {
      double p=0, i=0, d=0;
      while (scan.hasNextLine()) {
        String nextLine = scan.nextLine();

        // all numbers 0-9, characters "." and "-"
        String strip = nextLine.replaceAll("[^0-9.-]", "");

        // assign relevant constant values to respective double variables
        if (nextLine.contains("p")) { 
          p = Double.parseDouble(strip); 
        }
        else if (nextLine.contains("i")) { 
          i = Double.parseDouble(strip); 
        }
        else if (nextLine.contains("d")) { 
          d = Double.parseDouble(strip); 
        }
      }

      // assign variables to PID controller configuration
      controller.setP(p);
      controller.setI(i);
      controller.setD(d);
    } catch (Exception e) {
      DriverStation.reportError("File not found", true);
    }
  }
}
