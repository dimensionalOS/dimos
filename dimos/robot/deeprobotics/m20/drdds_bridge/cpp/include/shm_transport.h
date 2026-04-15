// shm_transport.h - Lock-free SPSC ring buffer over POSIX shared memory
// Used by drdds_recv (writer) and ros2_pub (reader) to bridge point cloud data
// without linking both FastDDS versions in one process.

#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <string>
#include <stdexcept>

namespace drdds_bridge {

// Ring buffer layout in shared memory:
//   [Header: 64 bytes]
//   [Slot 0: SlotHeader + data bytes]
//   [Slot 1: SlotHeader + data bytes]
//   ...
//   [Slot N-1: SlotHeader + data bytes]

static_assert(std::atomic<uint64_t>::is_always_lock_free,
    "atomic<uint64_t> must be lock-free for cross-process SHM on ARM64");

struct ShmHeader {
    std::atomic<uint64_t> write_idx{0};  // monotonically increasing write index
    std::atomic<uint64_t> read_idx{0};   // monotonically increasing read index
    uint32_t num_slots;                   // number of slots in ring buffer
    uint32_t slot_size;                   // max bytes per slot (including SlotHeader)
    uint32_t ready;                       // set to 0xDEADBEEF when initialized
    char padding[28];                     // pad to 64 bytes
};

struct SlotHeader {
    uint32_t data_size;       // actual payload bytes
    uint32_t msg_type;        // 0 = PointCloud2, 1 = Imu
    int32_t  stamp_sec;       // ROS timestamp seconds
    uint32_t stamp_nsec;      // ROS timestamp nanoseconds
    uint32_t height;          // PointCloud2 height
    uint32_t width;           // PointCloud2 width
    uint32_t point_step;      // bytes per point
    uint32_t row_step;        // bytes per row
    uint8_t  is_dense;        // PointCloud2 is_dense
    uint8_t  is_bigendian;    // PointCloud2 is_bigendian
    uint8_t  _pad1[2];
    uint32_t fields_size;     // size of serialized PointFieldArray following data
    uint8_t  _pad2[24];       // pad to 64 bytes (ARM64 cache line alignment)
    // followed by: [data_size bytes of point data]
};
static_assert(sizeof(SlotHeader) == 64, "SlotHeader must be cache-line aligned");

static constexpr uint32_t SHM_READY_MAGIC = 0xDEADBEEF;
static constexpr const char* SHM_LIDAR_NAME = "/drdds_bridge_lidar";
static constexpr const char* SHM_IMU_NAME = "/drdds_bridge_imu";
static constexpr const char* SHM_NOTIFY_NAME = "/drdds_bridge_notify";

// Slot size: 4MB per lidar message (M20 dual RSAIRY lidars merged = ~3MB per scan)
// 16 slots × 4MB = 64MB total for lidar ring buffer.
// Needs headroom because ROS2 publish() can stall 50-400ms on ARM64.
// At 10Hz lidar, 16 slots = 1.6s buffer before frame drops.
static constexpr uint32_t LIDAR_SLOT_SIZE = 4 * 1024 * 1024;
static constexpr uint32_t LIDAR_NUM_SLOTS = 16;

// IMU: 256 bytes per message, 64 slots (covers 320ms at 200Hz)
static constexpr uint32_t IMU_SLOT_SIZE = 256;
static constexpr uint32_t IMU_NUM_SLOTS = 64;

inline size_t shm_total_size(uint32_t num_slots, uint32_t slot_size) {
    return sizeof(ShmHeader) + (size_t)num_slots * slot_size;
}

// Writer side (used by drdds_recv)
class ShmWriter {
public:
    ShmWriter(const std::string& name, uint32_t num_slots, uint32_t slot_size) {
        size_t total = shm_total_size(num_slots, slot_size);
        int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0) throw std::runtime_error("shm_open failed: " + name);
        ftruncate(fd, total);
        mem_ = (uint8_t*)mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (mem_ == MAP_FAILED) throw std::runtime_error("mmap failed");

        hdr_ = reinterpret_cast<ShmHeader*>(mem_);
        hdr_->num_slots = num_slots;
        hdr_->slot_size = slot_size;
        hdr_->write_idx.store(0, std::memory_order_release);
        hdr_->read_idx.store(0, std::memory_order_release);
        hdr_->ready = SHM_READY_MAGIC;

        total_size_ = total;
        name_ = name;
    }

    ~ShmWriter() {
        if (mem_ && mem_ != MAP_FAILED) munmap(mem_, total_size_);
    }

    ShmWriter(const ShmWriter&) = delete;
    ShmWriter& operator=(const ShmWriter&) = delete;

    // Returns pointer to current write slot's SlotHeader. Caller fills in header + data.
    // Call commit() after writing to advance the write index.
    SlotHeader* acquire() {
        uint64_t wi = hdr_->write_idx.load(std::memory_order_relaxed);
        uint32_t slot_idx = wi % hdr_->num_slots;
        return reinterpret_cast<SlotHeader*>(mem_ + sizeof(ShmHeader) + (size_t)slot_idx * hdr_->slot_size);
    }

    uint8_t* slot_data(SlotHeader* slot) {
        return reinterpret_cast<uint8_t*>(slot) + sizeof(SlotHeader);
    }

    void commit() {
        hdr_->write_idx.fetch_add(1, std::memory_order_release);
    }

private:
    uint8_t* mem_ = nullptr;
    ShmHeader* hdr_ = nullptr;
    size_t total_size_ = 0;
    std::string name_;
};

// Reader side (used by ros2_pub)
class ShmReader {
public:
    ShmReader(const std::string& name) : name_(name) {}

    bool try_open() {
        int fd = shm_open(name_.c_str(), O_RDONLY, 0666);
        if (fd < 0) return false;

        // Map just the header first to read sizes safely (avoids read() on atomics)
        void* hdr_map = mmap(nullptr, sizeof(ShmHeader), PROT_READ, MAP_SHARED, fd, 0);
        if (hdr_map == MAP_FAILED) { close(fd); return false; }
        auto* peek = reinterpret_cast<const ShmHeader*>(hdr_map);
        if (peek->ready != SHM_READY_MAGIC) { munmap(hdr_map, sizeof(ShmHeader)); close(fd); return false; }
        uint32_t num_slots = peek->num_slots;
        uint32_t slot_size = peek->slot_size;
        munmap(hdr_map, sizeof(ShmHeader));

        // Now map the full region read-only
        size_t total = shm_total_size(num_slots, slot_size);
        mem_ = (uint8_t*)mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (mem_ == MAP_FAILED) { mem_ = nullptr; return false; }

        hdr_ = reinterpret_cast<ShmHeader*>(mem_);
        total_size_ = total;
        last_read_ = hdr_->write_idx.load(std::memory_order_acquire); // start from current
        return true;
    }

    // Returns next unread slot, or nullptr if no new data
    const SlotHeader* poll() {
        if (!hdr_) return nullptr;
        uint64_t wi = hdr_->write_idx.load(std::memory_order_acquire);
        if (last_read_ >= wi) return nullptr;

        // Skip if we fell behind (overwritten slots)
        if (wi - last_read_ > hdr_->num_slots) {
            last_read_ = wi - 1; // jump to latest
        }

        uint32_t slot_idx = last_read_ % hdr_->num_slots;
        last_read_++;
        return reinterpret_cast<const SlotHeader*>(mem_ + sizeof(ShmHeader) + (size_t)slot_idx * hdr_->slot_size);
    }

    const uint8_t* slot_data(const SlotHeader* slot) {
        return reinterpret_cast<const uint8_t*>(slot) + sizeof(SlotHeader);
    }

    bool is_open() const { return hdr_ != nullptr; }

    ~ShmReader() {
        if (mem_ && mem_ != MAP_FAILED) munmap(mem_, total_size_);
    }

    ShmReader(const ShmReader&) = delete;
    ShmReader& operator=(const ShmReader&) = delete;

private:
    uint8_t* mem_ = nullptr;
    ShmHeader* hdr_ = nullptr;
    size_t total_size_ = 0;
    uint64_t last_read_ = 0;
    std::string name_;
};

} // namespace drdds_bridge
