// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialStandByCommandGroup extends SequentialCommandGroup {
  List<Command> standby_commands;

  public SequentialStandByCommandGroup(BooleanSupplier _to_continue,Command... commands) {
    for (Command command : commands) {
      
      addCommands(Commands.race(command,Commands.waitUntil(_to_continue)));
      
    }
  }
}
