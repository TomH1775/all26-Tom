package org.team100.lib.util;

/**
 * A wrapper that makes it easier to find all the can IDs.
 * 
 * Use "find references" to the constructor to find them all, or search for a
 * specific one, e.g. "CanId(10)"
 */
public class CanId {
    public final int id;

    public CanId(int id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return "CanId [id=" + id + "]";
    }
}
