package lib.team8592.logging;

import edu.wpi.first.networktables.GenericEntry;

public class LoggerEntry {
    private String key;
    private GenericEntry entry;
    private boolean override = false;

    protected LoggerEntry(String key, GenericEntry entry) {
        this.key = key;
        this.entry = entry;
    }

    public void override(boolean override) {
        this.override = override;
    }

    public boolean override() {
        return this.override;
    }

    public GenericEntry getEntry() {
        return this.entry;
    }

    public String getKey() {
        return this.key;
    }
}