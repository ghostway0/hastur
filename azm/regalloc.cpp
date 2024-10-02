#include <cassert>
#include <vector>
#include <ranges>

#include <absl/container/flat_hash_map.h>

#include "regalloc.h"

CodePoint const kMaximumInvalidCodePoint{SIZE_MAX};

// spans one instruction
constexpr size_t kMinimalInterval = 2;

std::optional<LiveBundle> LiveBundle::truncated(Interval interval) {
    // this creates a new bundle with the ranges in the range [from, to]
    // it also handles when from or to intersect a range (they split it at the
    // right point, distributing the uses) funnily enough, it probably is faster
    // to go over sequentially than to use binary search

    std::vector<LiveRange *> new_ranges;

    for (LiveRange *range : ranges_) {
        Interval live_in = range->live_interval();

        if (!interval.overlaps_with(live_in)) {
            continue;
        }

        if (live_in.fully_within(interval)) {
            new_ranges.push_back(range);
            continue;
        }

        CodePoint new_start = std::max(range->start, interval.low);
        CodePoint new_end = std::min(range->end, interval.high);

        LiveRange *truncated_range = range->clone();
        truncated_range->start = new_start;
        truncated_range->end = new_end;

        std::erase_if(truncated_range->uses, [&](CodePoint use) { return use < new_start || use > new_end; });

        new_ranges.push_back(truncated_range);
    }

    if (new_ranges.empty()) {
        return std::nullopt;
    }

    return LiveBundle{std::move(new_ranges), allocation_};
}

namespace {

std::optional<CodePoint> find_split_spot(LiveRange *range, std::span<LiveRange *> interferences) {
    CodePoint initial_intersection = kMaximumInvalidCodePoint;

    for (LiveRange *interference : interferences) {
        if (interference->start < initial_intersection) {
            initial_intersection = CodePoint{
                    interference->start.max(range->start),
            };
        }
    }

    if (initial_intersection == kMaximumInvalidCodePoint) {
        return std::nullopt;
    }

    if (initial_intersection != range->start) {
        return initial_intersection;
    }

    if (range->uses.empty() || range->uses.front() == range->end || range->uses.front() == range->start) {
        return range->start.next_inst();
    }

    return range->uses.front();
}

void patch_liveranges(std::vector<LiveRange *> ranges, std::vector<Stitch> &stitches) {
    std::sort(ranges.begin(), ranges.end(), [](LiveRange *lhs, LiveRange *rhs) {
        return lhs->live_interval().low < rhs->live_interval().low;
    });

    std::unordered_map<VirtualReg, LiveRange *> last_used;
    std::unordered_map<VirtualReg, size_t> spill_slot_mapping;
    size_t spill_slot_offset = 0;

    for (LiveRange *range : ranges) {
        auto const &vreg = range->vreg;

        if (auto it = last_used.find(vreg); it != last_used.end()) {
            if (range->parent->allocation() != it->second->parent->allocation()) {
                stitches.push_back(Stitch{
                        .vreg = vreg,
                        .from = it->second->parent->allocation().reg(),
                        .to = range->parent->allocation().reg(),
                        .at = it->second->end.next_inst(),
                });
            }
        }

        if (range->parent->allocation().is_spill()) {
            if (spill_slot_mapping.find(vreg) == spill_slot_mapping.end()) {
                spill_slot_mapping[vreg] = spill_slot_offset;
                spill_slot_offset += range->vreg.type.size_bytes();
            }
            range->parent->set_allocation(Allocation::spill(spill_slot_mapping[vreg]));
        }

        last_used[vreg] = range;
    }
}

} // namespace

Output Allocator::run(std::span<LiveBundle> bundles) {
    for (LiveBundle &bundle : bundles) {
        for (LiveRange *range : bundle.ranges()) {
            pending_.push(range);
        }
    }

    while (!pending_.empty()) {
        LiveRange *range = pending_.top();
        pending_.pop();

        if (auto preg = run_once(range); preg) {
            range->parent.set_allocation(Allocation::reg(preg.value()));
            trees_.at(range->vreg.type).insert(range->live_interval(), range);
        } else {
            second_chance_.push(range);
        }
    }

    // code duplication is not great
    while (!second_chance_.empty()) {
        LiveRange *range = second_chance_.top();
        second_chance_.pop();

        if (auto preg = run_once(range); preg) {
            range->parent.set_allocation(Allocation{preg.value()});
            trees_.at(range->vreg.type).insert(range->live_interval(), range);
        } else {
            range->parent.set_allocation(Allocation::spill());
        }
    }

    return Output::from_ranges(extract_ranges());
}

std::vector<LiveRange *> Allocator::extract_ranges() {
    std::vector<LiveRange *> result;

    for (auto &[_, tree] : trees_) {
        result.insert(result.end(), tree.extract_all().begin(), tree.extract_all().end());
    }

    return result;
}

std::optional<Register> Allocator::run_once(LiveRange *range) {
    std::vector<LiveRange *> interferences{};
    trees_.at(range->vreg.type).overlap(range->live_interval(), interferences);

    // 1. assign naively. where that doesn't work, evict if beneficial.
    if (std::optional<Register> preg = try_assign_might_evict(range, interferences); preg.has_value()) {
        return preg;
    }

    // 2. split ranges; if we can't, push them to second chance queue
    // to be allocated at a later stage

    std::optional<CodePoint> split_spot = find_split_spot(range, interferences);

    if (!split_spot.has_value() || !try_split(range, split_spot.value())) {
        second_chance_.push(range);
        return std::nullopt;
    }

    return std::nullopt;
}

std::optional<Register> Allocator::get_unused_preg(RegClass type, std::span<LiveRange *> interferences) {
    std::vector<bool> used(isa_.registers.at(type).size(), false);

    for (auto [i, interference] : std::views::enumerate(interferences)) {
        Allocation allocation = interference->parent.allocation();

        if (!allocation.is_reg()) {
            continue;
        }

        used[i] = true;
    }

    for (size_t i = 0; i < used.size(); i++) {
        if (!used[i]) {
            return isa_.registers.at(type)[i];
        }
    }

    return std::nullopt;
}

absl::flat_hash_map<Register, size_t> Allocator::calculate_eviction_costs(std::span<LiveRange *> interferences) {
    std::map<Register, size_t> eviction_costs;

    for (LiveRange const *interference : interferences) {
        Allocation const &allocation = interference->parent.allocation();

        if (!allocation.is_reg()) {
            continue;
        }

        eviction_costs[allocation.reg()] += interference->spill_cost;
    }

    return eviction_costs;
}

std::optional<Register> Allocator::try_assign_might_evict(LiveRange *range, std::span<LiveRange *> interferences) {
    if (std::optional<Register> preg = get_unused_preg(range->vreg.type, interferences); preg) {
        return preg;
    }

    size_t min_cost = SIZE_MAX;
    std::optional<Register> best_reg = std::nullopt;

    for (auto [reg, cost] : calculate_eviction_costs(interferences)) {
        if (cost < min_cost) {
            min_cost = cost;
            best_reg = reg;
        }
    }

    if (!best_reg.has_value()) {
        return std::nullopt;
    }

    assert(min_cost != SIZE_MAX);

    if (min_cost < range->spill_cost) {
        evict_for(best_reg.value(), interferences);
        return best_reg;
    }

    return std::nullopt;
}

bool Allocator::try_split(LiveRange *range, CodePoint at) {
    LiveBundle *bundle = range->parent;

    if (bundle->is_minimal()) {
        return false;
    }

    std::optional<LiveBundle> left = bundle.truncated(bundle->start(), at.prev_inst().late());
    std::optional<LiveBundle> right = bundle.truncated(at, bundle->end());

    if (!left.has_value() || !right.has_value()) {
        return false;
    }

    if (left->num_ranges() + right->num_ranges() != bundle->num_ranges()) {
        pending_.push(left->last_range());
        pending_.push(right->first_range());
    }

    delete bundle;

    for (LiveRange *range : left->ranges()) {
        range->parent = left;
    }

    for (LiveRange *range : right->ranges()) {
        range->parent = right;
    }

    // avoid std::expected<std::optional<std::pair<LiveRange, LiveRange>>,
    // Error> for my life
    return true;
}

void Allocator::evict_for(Register reg, std::span<LiveRange *> interferences) {
    for (LiveRange const *interference : interferences) {
        if (interference->parent.allocation().reg() == reg) {
            trees_.at(reg.type).remove({interference->start, interference->end});
        }
    }
}

Output Output::from_ranges(std::vector<LiveRange *> ranges) {
    std::vector<Stitch> stitches;
    patch_liveranges(ranges, stitches);

    return Output{std::move(ranges), stitches};
}
