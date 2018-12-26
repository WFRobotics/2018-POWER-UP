package org.wfrobotics.reuse.config;

import java.util.function.Supplier;

import org.wfrobotics.reuse.utilities.ConsoleLogger;

/**
 * Handles an autonomous option, providing the selected value via callback when the selected value changes
 * @author STEM Alliance of Fargo Moorhead
 */
public abstract class AutoSelection<T> implements Supplier<T>
{
    /** Listens for AutoSelection updates */
    public interface SelectionListener
    {
        /** Inform registered listeners to update based on an {@AutoSelection}'s new value */
        public void onSelectionChanged();
    }

    private int selectedIndex = 0;
    private SelectionListener callback = null;

    protected abstract T[] options();
    protected abstract void apply();

    public void registerListener(SelectionListener callback)
    {
        this.callback = callback;
    }

    public T get()
    {
        return options()[selectedIndex];
    }

    public void increment()
    {
        if (callback == null)
        {
            ConsoleLogger.warning("Forget to register Selection callback");
            return;
        }
        selectedIndex = next(selectedIndex, options().length - 1);
        callback.onSelectionChanged();
    }

    private int next(int indexCurrent, int indexMax)
    {
        return (indexCurrent < indexMax) ? indexCurrent + 1 : 0;
    }

    public String toString()
    {
        String name = getClass().getSimpleName().replaceFirst("Select.*", "");
        return String.format("%s: %s", name, get().toString());
    }
}
