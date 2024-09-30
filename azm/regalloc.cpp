#include <ranges>

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

std::vector<Stitch> discover_stitches(std::span<LiveRange> allocated_ranges) {
    std::sort(allocated_ranges.begin(), allocated_ranges.end(), [](LiveRange const &lhs, LiveRange const &rhs) {
        return lhs < rhs;
    });

    std::map<Interval, void> intervals;

    for (LiveRange const &range : allocated_ranges) {
        intervals.emplace(range.live_interval(), {});
    }

    std::map<VirtualReg, LiveRange const *> last_used;

    std::vector<Stitch> stitches;

    for (auto const &[interval, _] : intervals) {
        LiveRange const &representative = *interval.ranges().front();

        if (auto last_range = last_used.find(representative.vreg); last_range != last_used.end()) {
            if (interval.allocation() == last_range->second->allocation()) {
                continue;
            }

            stitches.push_back(Stitch{
                    .codepoint = last_range->second->end.next_inst(),
                    .from = last_range->second->allocation(),
                    .to = interval.allocation(),
                    .vreg = representative.vreg,
            });
        }

        last_used[representative.vreg] = &representative;
    }

    return stitches;
}

void assign_spillslots(std::vector<LiveBundle> &bundles, std::vector<Stitch> const &stitches) {
    // assign spill slots based on the stitches and instructions. This basically goes over the instructions, finds the
    // current rsp offset at every point, and when we encounter a spill, we assign it to the next available slot.

    size_t idx = 0;
    size_t delta = 0;

    std::vector<LiveRange *> allocations;

    for (LiveBundle const &bundle : bundles) {
        for (LiveRange *range : bundle.ranges()) {
            allocations.push_back(range);
        }
    }

    std::sort(allocations.begin(), allocations.end(), [](LiveRange *lhs, LiveRange *rhs) {
        return lhs->start < rhs->start;
    });

    std::vector<LiveRange *> active_ranges;
    std::unordered_map<VirtualReg, size_t> active_stitch_mapping;

    CodePoint current{0};

    for (size_t i = 0; i < allocations.size(); i++) {
        LiveRange *range = allocations[i];

        if (range->start > current.late()) {
            for (size_t j = 0; j < active_ranges.size();) {
                if (active_ranges[j]->end < current) {
                    active_ranges.erase(active_ranges.begin() + j);
                } else {
                    j++;
                }
            }
        }

        if (range->start < current.late()) {
            if (auto it = active_stitch_mapping.find(range->vreg); it != active_stitch_mapping.end()) {
                range->set_allocation(Allocation::stack(it->second));
            } else {
                range->set_allocation(Allocation::stack(delta));
                delta += range->type.size_bytes();
            }
        }

        current = current.next_inst();
    }
}

} // namespace

Output Allocator::run(std::span<LiveBundle> bundles) {
    for (LiveBundle &bundle : bundles) {
        for (LiveRange *range : bundle.ranges()) {
            pending_.push(range);
        }

        bundles_.insert(std::move(bundle));
    }

    while (!pending_.empty()) {
        LiveRange *range = pending_.top();
        pending_.pop();

        if (auto preg = run_once(range); preg) {
            bundles_.at(range->parent_id).set_allocation(Allocation::reg(preg.value()));
            trees_.at(range->type).insert(range->live_interval(), range);
        } else {
            second_chance_.push(range);
        }
    }

    // code duplication is not great
    while (!second_chance_.empty()) {
        LiveRange *range = second_chance_.top();
        second_chance_.pop();

        if (auto preg = run_once(range); preg) {
            bundles_.at(range->parent_id).set_allocation(Allocation{preg.value()});
            trees_.at(range->type).insert(range->live_interval(), range);
        } else {
            LiveBundle &bundle = bundles_.at(range->parent_id);
            bundle.set_allocation(Allocation::spill());
        }
    }

    return Output::from_bundles(bundles_.extract_all());
}

std::optional<Register> Allocator::run_once(LiveRange *range) {
    std::vector<LiveRange *> interferences{};
    trees_.at(range->type).overlap(range->live_interval(), interferences);

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
        LiveBundle const &bundle = bundles_.at(interference->parent_id);
        Allocation allocation = bundle.allocation();

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

std::map<Register, size_t> Allocator::calculate_eviction_costs(std::span<LiveRange *> interferences) {
    std::map<Register, size_t> eviction_costs;

    for (LiveRange const *interference : interferences) {
        LiveBundle const &bundle = bundles_.at(interference->parent_id);
        Allocation const &allocation = bundle.allocation();

        if (!allocation.is_reg()) {
            continue;
        }

        eviction_costs[allocation.reg()] += interference->spill_cost;
    }

    return eviction_costs;
}

std::optional<Register> Allocator::try_assign_might_evict(LiveRange *range, std::span<LiveRange *> interferences) {
    if (std::optional<Register> preg = get_unused_preg(range->type, interferences); preg) {
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
    LiveBundle &bundle = bundles_.at(range->parent_id);

    if (bundle.is_minimal()) {
        return false;
    }

    std::optional<LiveBundle> left = bundle.truncated(bundle.start(), at.prev_inst().late());
    std::optional<LiveBundle> right = bundle.truncated(at, bundle.end());

    if (!left.has_value() || !right.has_value()) {
        return false;
    }

    bundles_.erase(range->parent_id);

    if (left->num_ranges() + right->num_ranges() != bundle.num_ranges()) {
        pending_.push(left->last_range());
        pending_.push(right->first_range());
    }

    std::span<LiveRange *> left_ranges = left->ranges();
    std::span<LiveRange *> right_ranges = right->ranges();

    size_t left_id = bundles_.insert(std::move(left.value()));
    size_t right_id = bundles_.insert(std::move(right.value()));

    for (LiveRange *range : left_ranges) {
        range->parent_id = left_id;
    }

    for (LiveRange *range : right_ranges) {
        range->parent_id = right_id;
    }

    // avoid std::expected<std::optional<std::pair<LiveRange, LiveRange>>,
    // Error> for my life
    return true;
}

void Allocator::evict_for(Register reg, std::span<LiveRange *> interferences) {
    for (LiveRange const *interference : interferences) {
        LiveBundle &bundle = bundles_.at(interference->parent_id);

        if (bundle.allocation().reg() == reg) {
            trees_.at(reg.type).remove({interference->start, interference->end});
        }
    }
}

Output Output::from_bundles(std::vector<LiveBundle> bundles) {
    std::vector<Stitch> stitches = discover_stitches(bundles);

    assign_spillslots(bundles, stitches);

    return Output{std::move(bundles), stitches};
}
