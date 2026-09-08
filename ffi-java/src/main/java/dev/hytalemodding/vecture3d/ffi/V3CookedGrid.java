package dev.hytalemodding.vecture3d.ffi;

import java.lang.foreign.MemorySegment;
import java.util.Objects;

import dev.hytalemodding.vecture3d.ffi.generated.V3Abi;

/**
 * A handle on immutable cooked BlockGrid geometry.
 *
 * <p>The handle owns exactly one native reference. It is independent of any world, so the same
 * handle may be attached to bodies in several worlds. Attachment and replacement retain their own
 * reference, so the caller may close this handle immediately after either operation. Geometry stays
 * alive while any shape or world holds it. Closing is idempotent. Instances are confined to the
 * thread that cooked them.
 */
public final class V3CookedGrid implements AutoCloseable {
    private final Thread ownerThread;
    private MemorySegment handle;
    private boolean closed;

    V3CookedGrid(MemorySegment handle) {
        this.handle = Objects.requireNonNull(handle, "handle");
        ownerThread = Thread.currentThread();
    }

    MemorySegment handle() {
        requireOpenOwner();
        return handle;
    }

    /** Whether this handle has been closed. */
    public boolean isClosed() {
        return closed;
    }

    @Override
    public void close() {
        requireOwnerThread();
        if (closed) {
            return;
        }
        try {
            V3Abi.v3_destroy_cooked_grid(handle);
        } finally {
            handle = MemorySegment.NULL;
            closed = true;
        }
    }

    private void requireOpenOwner() {
        requireOwnerThread();
        if (closed) {
            throw new IllegalStateException("V3CookedGrid is closed");
        }
    }

    private void requireOwnerThread() {
        if (Thread.currentThread() != ownerThread) {
            throw new IllegalStateException("V3CookedGrid is confined to its creating thread");
        }
    }
}
