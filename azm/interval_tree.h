#include <compare>
#include <cstddef>
#include <vector>

#include <absl/container/btree_map.h>

class CodePoint {
public:
    explicit CodePoint(size_t point) : point_{point} {}

    CodePoint max(CodePoint const &other) const { return CodePoint{std::max(point_, other.point_)}; }

    size_t repr() const { return point_; }

    CodePoint next_inst() const { return CodePoint{early().point_ + 2}; }
    CodePoint prev_inst() const { return CodePoint{early().point_ - 2}; }
    CodePoint early() const { return CodePoint{point_ & (~static_cast<size_t>(1))}; };
    CodePoint late() const { return CodePoint{point_ | static_cast<size_t>(1)}; };

    auto operator<=>(CodePoint const &other) const = default;

private:
    size_t point_;
};

struct Interval {
    CodePoint low;
    CodePoint high;

    Interval(CodePoint start, CodePoint end) : low{start}, high{end} {}

    auto operator<=>(Interval const &) const = default;

    bool overlaps_with(Interval const &other) const { return low <= other.high && high >= other.low; }

    bool is_minimal() const { return high.repr() - low.repr() == kMinimalInterval; }

    bool fully_within(Interval const &other) const { return other.low <= low && other.high >= high; }
};

template<typename T>
class IntervalTree {
public:
    bool insert(Interval const &interval, T value) {
        auto [it, inserted] = btree_.insert({interval, value});
        return inserted;
    }

    class OverlapIterator {
        using Iterator = absl::btree_map<Interval, T>::iterator;

    public:
        OverlapIterator(Iterator begin, Iterator end, Interval const &interval)
            : it_{begin}, end_{end}, interval_{interval} {
            if (it_ != end_ && !it_->first.overlaps_with(interval_)) {
                ++(*this);
            }
        }

        OverlapIterator &begin() { return *this; }

        OverlapIterator end() const { return OverlapIterator{end_, end_, interval_}; }

        OverlapIterator &operator++() {
            ++it_;
            while (it_ != end_ && !it_->first.overlaps_with(interval_)) {
                ++it_;
            }
            return *this;
        }

        OverlapIterator &operator--() {
            if (it_ == end_) {
                --it_;
            }
            while (it_ != end_ && !it_->first.overlaps_with(interval_)) {
                --it_;
            }
            return *this;
        }

        bool operator==(OverlapIterator const &other) const { return it_ == other.it_; }

        bool operator!=(OverlapIterator const &other) const { return it_ != other.it_; }

        T &operator*() { return it_->second; }

    private:
        Iterator it_;
        Iterator end_;
        Interval interval_;

        friend class IntervalTree;
    };

    OverlapIterator overlap(Interval const &interval) {
        auto it = btree_.lower_bound(interval);

        if (it != btree_.begin()) {
            auto prev_it = std::prev(it);
            if (prev_it->first.overlaps_with(interval)) {
                it = prev_it;
            }
        }

        return OverlapIterator(it, btree_.end(), interval);
    }

    void overlap(Interval const &interval, std::vector<T> &out) {
        for (auto range : overlap(interval)) {
            out.push_back(range);
        }
    }

    void erase_intersecting(Interval const &interval) { erase_all(overlap(interval)); }

    void erase_all(OverlapIterator it) {
        auto curr = it.end();

        while (curr != it.begin()) {
            --curr;

            auto to_erase = curr.it_;

            btree_.erase(to_erase);
        }
    }

    void remove(Interval interval) { btree_.erase(interval); }

    std::vector<T> extract_all() {
        std::vector<T> out;
        for (auto const &[_, value] : btree_) {
            out.push_back(value);
        }

        return out;
    }

private:
    absl::btree_map<Interval, T> btree_;
};
