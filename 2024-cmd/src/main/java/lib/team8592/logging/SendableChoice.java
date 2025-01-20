package lib.team8592.logging;

public class SendableChoice<T> {
    private String name;
    private T choice;

    public SendableChoice(String name, T choice) {
        this.name = name;
        this.choice = choice;
    }

    public String getName() {
        return this.name;
    }

    public T get() {
        return this.choice;
    }
}