package org.team100.lib.targeting;

public class RangeTest {
    private static final boolean DEBUG = false;

    /**
     * Is it worth caching?
     * 
     * Without caching, a range solution takes about 50 us on my machine.
     * Wih caching, it takes much much less, maybe 0.1 us.
     * 
     * The range solver is inside the inner loop of a Newton solver which takes 5 or
     * 10 iterations to converge, so uncached, up to 0.5 ms on my machine, or maybe
     * 2 or 3 ms on the RoboRIO -- a substantial fraction of the time budget.
     * 
     * So caching seems like a good idea.
     */
    // disable to speed up tests
    // @Test
    void testPerformance() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        RangeSolver rangeSolver = new RangeSolver(d, 0);

        double uncachedETperCall;
        {
            int iterations = 10000;
            long startTime = System.currentTimeMillis();
            for (int i = 0; i < iterations; ++i) {
                rangeSolver.getSolution(10, 1, 1);
            }
            long finishTime = System.currentTimeMillis();
            if (DEBUG) {
                System.out.printf("Uncached ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
                uncachedETperCall = 1000000 * ((double) finishTime - startTime) / iterations;
                System.out.printf("Uncached ET/call (ns): %6.3f\n ", uncachedETperCall);
            }
        }
        double cachedETperCall;
        {
            RangeCache r = new RangeCache(rangeSolver, 10, 1);
            int iterations = 10000000;
            long startTime = System.currentTimeMillis();
            for (int i = 0; i < iterations; ++i) {
                r.get(1);
            }
            long finishTime = System.currentTimeMillis();
            if (DEBUG) {
                System.out.printf("Cached ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
                cachedETperCall = 1000000 * ((double) finishTime - startTime) / iterations;
                System.out.printf("Cached ET/call (ns): %6.3f\n ", cachedETperCall);
            }
        }
        if (DEBUG)
            System.out.printf("cache benefit %f\n", uncachedETperCall / cachedETperCall);
    }

}
