package lib.team8592;

public enum MatchMode {
    DISABLED,
    TELEOP,
    AUTONOMOUS,
    TEST;

    public boolean is(MatchMode mode) {
        return this.equals(mode);
    }

    public boolean isAny(MatchMode ... modes) {
        for (MatchMode mode : modes) {
            if (this.equals(mode)) return true;
        }
        return false;
    }
}
