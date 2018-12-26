package org.wfrobotics.reuse.utilities;

import org.wfrobotics.reuse.subsystems.EnhancedSubsystem;

/**
 * Includes self-testing
 * @author STEM Alliance of Fargo Moorhead
 */
public interface Testable
{
    /**
     * Run <b>automated, diagnostic tests</b> to check if {@link EnhancedSubsystem} or other
     * {@link Testable} is working on the robot. This helps us rapidly check for <i>every
     * issue we have a test for</i> before each competition match!
     */
    public TestReport runFunctionalTest();

    /** Executes {@link runFunctionalTest} for {@link Testable}. Wraps test in standard log format. */
    public static TestReport run(Testable systemUnderTest)
    {
        String name = systemUnderTest.getClass().getSimpleName();
        TestReport result;

        System.out.println(String.format("--- %s Test ---", name));
        if (systemUnderTest instanceof EnhancedSubsystem)
        {
            System.out.print(String.format("%s default command: %s\n", name, ((EnhancedSubsystem) systemUnderTest).getDefaultCommandName()));
        }
        result = systemUnderTest.runFunctionalTest();
        System.out.println(String.format("%s Test: %s", name, (result.allPassed()) ? "SUCCESS" : "FAILURE"));
        return result;
    }

    /** Tracks the number of tests runs for an {@link Testable} */
    public static class TestReport
    {
        private int successes;
        private int failures;

        public TestReport()
        {
            successes = 0;
            failures = 0;
        }

        /** Count this {@link Testable} in this Subsystem's tests */
        public void add(boolean singleResult)
        {
            add(singleResult, true);
        }

        /** Count this {@link Testable} in this Subsystem's tests */
        public void add(boolean singleResult, String whatTest)
        {
            System.out.println(String.format("%s %s", whatTest, (singleResult) ? "okay" : "failed"));
            add(singleResult, true);
        }

        /** Count this {@link Testable} in this Subsystem's tests. Can specify an expected failure. */
        public void add(boolean singleResult, boolean expectedResult)
        {
            if (singleResult == expectedResult)
            {
                successes++;
            }
            else
            {
                failures++;
            }
        }

        public void add(TestReport other)
        {
            successes += other.successes;
            failures += other.failures;
        }

        /** Take credit for running a test where the result is manually determined */
        public void addManualTest(String whatTest)
        {
            System.out.println(whatTest);
            successes++;
        }

        public boolean allPassed()
        {
            return failures == 0;
        }

        public int numTestsRan()
        {
            return successes + failures;
        }

        public String toString()
        {
            return String.format("%d Passed, %d Failed", successes, failures);
        }
    }
}
