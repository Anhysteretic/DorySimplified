package com.team841.dory.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

/**
 * Command that moves the escalator to a specified preset location.
 */
public class MoveCommand extends Command {
    public MoveCommand(Escalator escalator,
                       Escalator.Position position,
                       BooleanSupplier hasCoralSupplier,
                       BooleanSupplier isClear) {
        this.escalator = escalator;
        this.position = position;
        this.hasCoralSupplier = hasCoralSupplier;
        this.isClearSupplier = isClear;

        addRequirements(this.escalator);
        setName("Move");
    }

    Escalator escalator;
    Escalator.Position position;
    BooleanSupplier hasCoralSupplier, isClearSupplier;
    boolean hasCoral, isClear;

    @Override
    public void initialize() {
        this.hasCoral = hasCoralSupplier.getAsBoolean();
        this.isClear = isClearSupplier.getAsBoolean();

        // Make sure the elevator is clear otherwise do nothing.
        if (isClear)
            escalator.setPosition(position, hasCoral);
    }

    @Override
    public void execute() {
        if (isClear)
            this.escalator.setPosition(position, hasCoral);
    }

    /**
     * Determines if the command is finished. If the elevator is not clear, then the command also finishes.
     * @return true if the elevator is not clear or at the position
     */
    @Override
    public boolean isFinished() {
        return this.escalator.atPosition(position) || !this.isClear;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }
}
