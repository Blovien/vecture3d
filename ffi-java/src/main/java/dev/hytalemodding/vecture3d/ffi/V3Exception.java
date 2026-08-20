package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

public final class V3Exception extends RuntimeException {
    private final Kind kind;
    private final String operation;
    private final int status;
    private final long detail;

    V3Exception(Kind kind, String operation, int status, long detail) {
        this(validate(kind, operation, status, detail));
    }

    private V3Exception(Details details) {
        super(details.message());
        this.kind = details.kind();
        this.operation = details.operation();
        this.status = details.status();
        this.detail = details.detail();
    }

    private static Details validate(Kind kind, String operation, int status, long detail) {
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(operation, "operation");
        if (operation.isBlank()) {
            throw new IllegalArgumentException("operation cannot be blank");
        }
        if (status < 0) {
            throw new IllegalArgumentException("status must be non-negative");
        }
        return new Details(kind, operation, status, detail);
    }

    public Kind kind() {
        return kind;
    }

    public String operation() {
        return operation;
    }

    public int status() {
        return status;
    }

    public long detail() {
        return detail;
    }

    private record Details(Kind kind, String operation, int status, long detail) {
        private String message() {
            return operation + " failed with " + kind + " (status=" + status + ", detail=" + detail + ')';
        }
    }

    public enum Kind {
        ABI_MISMATCH,
        NATIVE_STATUS
    }
}
