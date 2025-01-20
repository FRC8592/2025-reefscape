package lib.team8592.logging;

import java.util.*;

import edu.wpi.first.networktables.GenericEntry;

public class LoggerDictionary {
    private Dictionary<String, LoggerEntry> cards = new Hashtable<>();

    protected LoggerDictionary() {}

    public boolean contains(String key) {
        return Collections.list(cards.keys()).contains(key);
    }

    public void addEntry(String key, LoggerEntry entry) {
        this.cards.put(key, entry);
    }

    public LoggerEntry getLoggerEntry(String key) {
        return this.cards.get(key);
    }

    public GenericEntry getGenericEntry(String key) {
        return this.getLoggerEntry(key).getEntry();
    }

    public ArrayList<GenericEntry> getAllGenericEntries() {
        ArrayList<GenericEntry> entries = new ArrayList<>();
        for (String key : Collections.list(cards.keys())) {
            entries.add(getGenericEntry(key));
        }
        return entries;
    }

    public ArrayList<GenericEntry> getAllLoggerEntries() {
        ArrayList<GenericEntry> entries = new ArrayList<>();
        for (String key : Collections.list(cards.keys())) {
            entries.add(getGenericEntry(key));
        }
        return entries;
    }
}