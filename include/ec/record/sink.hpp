#pragma once
/// @file
/// Where records go: the one interface every recorder, file writer or viewer
/// implements.

namespace ec::record {

/// Receives records of one type, in order, one call per record. An
/// implementation stores, shows or forwards them: a raw file, an HDF5 file, a
/// viewer. A sink for several record types implements Sink<T> once per type.
///
/// write() does not throw: a sink that fails keeps its own error, for its owner
/// to check, and drops what follows. Whether a sink may be called from several
/// threads is up to the implementation.
template <typename Record>
class Sink {
public:
    virtual ~Sink() = default;
    virtual void write(const Record& record) = 0;

protected:
    Sink() = default;
    Sink(const Sink&) = default;
    Sink& operator=(const Sink&) = default;
    Sink(Sink&&) = default;
    Sink& operator=(Sink&&) = default;
};

}  // namespace ec::record
