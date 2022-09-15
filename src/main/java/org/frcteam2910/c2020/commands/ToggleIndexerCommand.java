package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIndexerCommand extends CommandBase {
    private final Indexer indexer;
    private final boolean state;
    public ToggleIndexerCommand(Indexer indexer, boolean state) {
        this.indexer = indexer;
        this.state = state;
    }

    @Override
    public void initialize() {
        indexer.toggleIndexer(state);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
