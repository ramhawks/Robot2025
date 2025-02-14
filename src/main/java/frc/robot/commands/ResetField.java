package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetField extends Command {
    //private final Object m_RestField;

    public ResetField() {
        //m_gripper = subsystem;
        //addRequirements(m_RestField);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
