#include <algorithm>
#include <cstdint>
#include <expected>
#include <map>
#include <optional>
#include <queue>
#include <span>

#include <absl/container/flat_hash_map.h>

#include "interval_tree.h"

class Type {
    /// Encoded in a bitset (0x0 is void, 0xFFFF is invalid):
    /// +---------------+-----+-------+-----+--------+-------------+
    /// | 0-2           | 3   | 4     | 5   | 6      | 7-9         |
    /// +---------------+-----+-------+-----+--------+-------------+
    /// | log2(bitsize) | int | float | ptr | vector | log2(lanes) |
    /// +---------------+-----+-------+-----+--------+-------------+

    enum class Base {
        Void = 0,
        Int = 1,
        Float = 2,
        Ptr = 3,
        Vector = 4,
    };

    enum class Size {
        B8 = 0,
        B16 = 1,
        B32 = 2,
        B64 = 3,
        B128 = 4,
        B256 = 5,
        B512 = 6,
        B1024 = 7,
    };

    enum class Lane {
        L1 = 0,
        L2 = 1,
        L4 = 2,
        L8 = 3,
        L16 = 4,
        L32 = 5,
        L64 = 6,
        L128 = 7,
    };

public:
    Type(Base base, Size size, Lane lane)
        : bits_{static_cast<uint16_t>(base) | (static_cast<uint16_t>(size) << 3) | (static_cast<uint16_t>(lane) << 7)} {
    }

    Type() : bits_{0} {}

    Base base() const { return static_cast<Base>(bits_ & 0x7); }

    size_t size_bytes() { return static_cast<size_t>(1) << static_cast<size_t>(size()); }

    Size size() const { return static_cast<Size>((bits_ >> 3) & 0x7); }

    Lane lane() const { return static_cast<Lane>((bits_ >> 7) & 0x7); }

    bool is_void() const { return bits_ == 0; }

    bool is_int() const { return base() == Base::Int; }

    bool is_float() const { return base() == Base::Float; }

    bool is_ptr() const { return base() == Base::Ptr; }

    bool is_vector() const { return base() == Base::Vector; }

    bool operator==(Type const &other) const { return bits_ == other.bits_; }

    bool operator!=(Type const &other) const { return bits_ != other.bits_; }

private:
    uint16_t bits_;
};

struct VirtualReg {
    uint32_t vreg;
    Type type;

    auto operator<=>(VirtualReg const &other) const = default;
};

struct LiveRange;

enum class RegClass {
    Int,
    Float,
    Vector,
};

struct Register {
    RegClass type;
    uint8_t encoding;

    auto operator<=>(Register const &other) const = default;
};

class Allocation {
public:
    Allocation() : bits_(kNull) {}

    explicit Allocation(Register reg) : bits_(kReg | (static_cast<uint16_t>(reg.type) << 2) | reg.encoding) {}

    Allocation(Register reg, RegClass reg_class)
        : bits_(kReg | (static_cast<uint16_t>(reg_class) << 2) | reg.encoding) {}

    explicit Allocation(uint16_t spill_slot) : bits_(kSpill | (spill_slot & 0x0FFF)) {}

    static Allocation reg(Register reg) { return Allocation{reg}; }

    static Allocation spill(uint16_t slot) { return Allocation{slot}; }

    static Allocation spill() { return spill(static_cast<uint16_t>(-1)); }

    static Allocation null() { return Allocation{}; }

    bool is_null() const { return bits_ == kNull; }
    bool is_reg() const { return (bits_ & 0x3) == kReg; }
    bool is_spill() const { return (bits_ & 0x3) == kSpill; }
    bool is_nullspill() const { return is_spill() && spill_slot() == static_cast<uint16_t>(-1); }

    RegClass reg_class() const {
        assert(is_reg());
        return static_cast<RegClass>((bits_ >> 2) & 0x3);
    }

    Register reg() const {
        assert(is_reg());
        return Register{
                reg_class(),
                static_cast<uint8_t>(bits_ & 0xFF),
        };
    }

    uint16_t spill_slot() const {
        assert(is_spill());
        return bits_ & 0x0FFF;
    }

    Allocation &operator=(Allocation const &other) = default;

    bool operator==(Allocation const &other) const { return bits_ == other.bits_; }

    bool operator!=(Allocation const &other) const { return bits_ != other.bits_; }

private:
    // bits 0-1 : 0b00 -> invalid, 0b01 -> null, 0b10 -> spill, 0b11 -> reg
    // 2-3 class encoding: 0b00 -> reserved, 0b01 -> int, 0b10 -> float,
    // 0b11 -> vector 4-11: register encoding if reg
    // 4-15 spill slot if spill
    uint16_t bits_;

    static constexpr uint32_t kNull = 0x1;
    static constexpr uint32_t kReg = 0x2;
    static constexpr uint32_t kSpill = 0x3;
};

class LiveBundle;

struct LiveRange {
    CodePoint start;
    CodePoint end;
    LiveBundle *parent;
    size_t spill_cost;
    std::vector<CodePoint> uses;
    VirtualReg vreg;

    Interval live_interval() const { return Interval{start, end}; }

    bool is_minimal() const { return Interval{end, start}.is_minimal(); }

    auto operator<=>(LiveRange const &) const = default;

    LiveRange *clone() { return new LiveRange{start, end, parent, spill_cost, uses, vreg}; }
};

class LiveBundle {
public:
    LiveBundle(std::vector<LiveRange *> ranges, Allocation allocation)
        : ranges_{std::move(ranges)}, allocation_{allocation} {}

    std::optional<LiveBundle> truncated(Interval interval);

    std::optional<LiveBundle> truncated(CodePoint from, CodePoint to) { return truncated({from, to}); }

    std::span<LiveRange *> ranges() { return ranges_; }

    Allocation allocation() const { return allocation_; }

    void set_allocation(Allocation alloc) { allocation_ = alloc; }

    CodePoint start() const { return ranges_.front()->start; }

    CodePoint end() const { return ranges_.back()->end; }

    size_t num_ranges() const { return ranges_.size(); }

    LiveRange *first_range() {
        assert(!ranges_.empty());
        return ranges_.front();
    }

    LiveRange *last_range() {
        assert(!ranges_.empty());
        return ranges_.back();
    }

    LiveRange const *last_range() const {
        assert(!ranges_.empty());
        return ranges_.back();
    }

    LiveRange const *first_range() const {
        assert(!ranges_.empty());
        return ranges_.front();
    }

    bool is_minimal() { return ranges_.size() == 1 && ranges_.front()->is_minimal(); }

private:
    // a set of non-intersecting sorted ranges
    std::vector<LiveRange *> ranges_;
    Allocation allocation_;
};

enum class Error {
    DuplicateRange,
};

struct TargetISA {
    std::map<RegClass, std::vector<Register>> registers;
};

struct PriorityCompare {
    bool operator()(LiveRange const *a, LiveRange const *b) { return a->spill_cost < b->spill_cost; }
};

template<typename V>
class IndexedMap {
public:
    size_t insert(V &&value) {
        map_.insert({counter_, value});
        return counter_++;
    }

    V const &at(size_t key) const { return map_.at(key); }

    V &at(size_t key) { return map_.at(key); }

    void erase(size_t key) { map_.erase(key); }

    size_t size() { return map_.size(); }

    std::vector<V> extract_all() {
        std::vector<V> result;
        result.reserve(map_.size());

        for (auto &[_, value] : map_) {
            result.push_back(std::move(value));
        }

        map_.clear();
        counter_ = 0;

        return result;
    }

    auto begin() { return map_.begin(); }

    auto end() { return map_.end(); }

private:
    std::unordered_map<size_t, V> map_{};
    size_t counter_{0};
};

struct Stitch {
    VirtualReg vreg;
    Register from;
    Register to;
    CodePoint at;
};

struct Output {
    std::vector<LiveRange *> allocations;
    std::vector<Stitch> stitches;

    static Output from_ranges(std::vector<LiveRange *> bundles);
};

class Allocator {
    using AllocationTree = IntervalTree<LiveRange *>;

public:
    explicit Allocator(TargetISA const &isa)
        : trees_{{RegClass::Int, AllocationTree{}},
                  {RegClass::Float, AllocationTree{}},
                  {RegClass::Vector, AllocationTree{}}},
          isa_{isa} {}

    Output run(std::span<LiveBundle> bundles);

private:
    std::priority_queue<LiveRange *, std::vector<LiveRange *>, PriorityCompare> second_chance_;
    std::unordered_map<RegClass, AllocationTree> trees_;
    std::priority_queue<LiveRange *, std::vector<LiveRange *>, PriorityCompare> pending_;
    TargetISA const &isa_;

    std::optional<Register> run_once(LiveRange *range);

    std::optional<Register> get_unused_preg(RegClass type, std::span<LiveRange *> interferences);

    std::flat_hash_map<Register, size_t> calculate_eviction_costs(std::span<LiveRange *> interferences);

    std::optional<Register> try_assign_might_evict(LiveRange *range, std::span<LiveRange *> interferences);

    std::optional<CodePoint> find_split_spot(LiveRange *range, std::span<LiveRange *> interferences);

    bool try_split(LiveRange *range, CodePoint at);

    void evict_for(Register reg, std::span<LiveRange *> interferences);

    std::vector<LiveRange *> extract_ranges();
};
