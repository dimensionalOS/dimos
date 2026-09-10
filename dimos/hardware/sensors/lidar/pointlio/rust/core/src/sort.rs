// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Verbatim ports of the unstable sorts the C++ leans on, so tie orders match bit for bit:
// libstdc++ (GCC 14) std::sort / std::nth_element, boost 1.87 pdqsort and spreadsort
// integer_sort (what pcl::VoxelGrid uses). All work on index ranges of one slice.

/// `std::sort(v.begin(), v.end(), less)`.
pub fn std_sort<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], less: &F) {
    if v.is_empty() {
        return;
    }
    introsort_loop(v, 0, v.len(), 2 * lg(v.len()), less);
    final_insertion_sort(v, 0, v.len(), less);
}

/// `std::nth_element(v.begin(), v.begin() + nth, v.end(), less)`.
pub fn nth_element<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], nth: usize, less: &F) {
    if v.is_empty() || nth >= v.len() {
        return;
    }
    introselect(v, 0, nth, v.len(), 2 * lg(v.len()), less);
}

/// `boost::sort::pdqsort(v.begin(), v.end(), less)` (non-branchless variant).
pub fn pdqsort<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], less: &F) {
    if v.is_empty() {
        return;
    }
    let log = lg(v.len());
    pdqsort_loop(v, 0, v.len(), less, log, true);
}

/// `boost::sort::spreadsort::integer_sort(begin, end, rshift)` with `less` the type's `<` on
/// the same u32 key `key` returns.
pub fn integer_sort<T: Copy, K: Fn(&T) -> u32, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    key: &K,
    less: &F,
) {
    if v.len() < MIN_SORT_SIZE {
        pdqsort(v, less);
    } else {
        let mut bin_sizes = vec![0usize; 1 << MAX_FINISHING_SPLITS];
        let mut bin_cache = Vec::new();
        spreadsort_rec(v, 0, v.len(), &mut bin_cache, 0, &mut bin_sizes, key, less);
    }
}

fn lg(n: usize) -> usize {
    usize::BITS as usize - 1 - n.leading_zeros() as usize
}

// ---- libstdc++ ----

const S_THRESHOLD: usize = 16;

fn introsort_loop<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    mut last: usize,
    mut depth: usize,
    less: &F,
) {
    while last - first > S_THRESHOLD {
        if depth == 0 {
            heap_select(v, first, last, last, less);
            sort_heap(v, first, last, less);
            return;
        }
        depth -= 1;
        let cut = unguarded_partition_pivot(v, first, last, less);
        introsort_loop(v, cut, last, depth, less);
        last = cut;
    }
}

fn introselect<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    nth: usize,
    mut last: usize,
    mut depth: usize,
    less: &F,
) {
    while last - first > 3 {
        if depth == 0 {
            heap_select(v, first, nth + 1, last, less);
            v.swap(first, nth);
            return;
        }
        depth -= 1;
        let cut = unguarded_partition_pivot(v, first, last, less);
        if cut <= nth {
            first = cut;
        } else {
            last = cut;
        }
    }
    insertion_sort(v, first, last, less);
}

fn final_insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &F,
) {
    if last - first > S_THRESHOLD {
        insertion_sort(v, first, first + S_THRESHOLD, less);
        unguarded_insertion_sort(v, first + S_THRESHOLD, last, less);
    } else {
        insertion_sort(v, first, last, less);
    }
}

fn insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &F,
) {
    for i in (first + 1)..last {
        if less(&v[i], &v[first]) {
            let val = v[i];
            v.copy_within(first..i, first + 1);
            v[first] = val;
        } else {
            unguarded_linear_insert(v, i, less);
        }
    }
}

fn unguarded_insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &F,
) {
    for i in first..last {
        unguarded_linear_insert(v, i, less);
    }
}

fn unguarded_linear_insert<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], mut last: usize, less: &F) {
    let val = v[last];
    let mut next = last - 1;
    while less(&val, &v[next]) {
        v[last] = v[next];
        last = next;
        next -= 1;
    }
    v[last] = val;
}

fn unguarded_partition_pivot<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &F,
) -> usize {
    let mid = first + (last - first) / 2;
    move_median_to_first(v, first, first + 1, mid, last - 1, less);
    unguarded_partition(v, first + 1, last, first, less)
}

fn move_median_to_first<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    result: usize,
    a: usize,
    b: usize,
    c: usize,
    less: &F,
) {
    if less(&v[a], &v[b]) {
        if less(&v[b], &v[c]) {
            v.swap(result, b);
        } else if less(&v[a], &v[c]) {
            v.swap(result, c);
        } else {
            v.swap(result, a);
        }
    } else if less(&v[a], &v[c]) {
        v.swap(result, a);
    } else if less(&v[b], &v[c]) {
        v.swap(result, c);
    } else {
        v.swap(result, b);
    }
}

fn unguarded_partition<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    pivot: usize,
    less: &F,
) -> usize {
    // The pivot slot lies outside [first, last), so a copy compares identically.
    let pivot = v[pivot];
    loop {
        // SAFETY: the median-of-3 pivot is a sentinel on both sides, so the scans stay in range.
        while less(unsafe { v.get_unchecked(first) }, &pivot) {
            first += 1;
        }
        last -= 1;
        while less(&pivot, unsafe { v.get_unchecked(last) }) {
            last -= 1;
        }
        if first >= last {
            return first;
        }
        v.swap(first, last);
        first += 1;
    }
}

// Heap primitives on v[first..]: `hole`/`len` are relative to `first` like libstdc++.
fn heap_select<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    middle: usize,
    last: usize,
    less: &F,
) {
    make_heap(v, first, middle, less);
    for i in middle..last {
        if less(&v[i], &v[first]) {
            pop_heap(v, first, middle, i, less);
        }
    }
}

fn make_heap<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, less: &F) {
    let len = last - first;
    if len < 2 {
        return;
    }
    let mut parent = (len - 2) / 2;
    loop {
        let value = v[first + parent];
        adjust_heap(v, first, parent, len, value, less);
        if parent == 0 {
            return;
        }
        parent -= 1;
    }
}

fn pop_heap<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    result: usize,
    less: &F,
) {
    let value = v[result];
    v[result] = v[first];
    adjust_heap(v, first, 0, last - first, value, less);
}

fn sort_heap<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], first: usize, mut last: usize, less: &F) {
    while last - first > 1 {
        last -= 1;
        pop_heap(v, first, last, last, less);
    }
}

fn adjust_heap<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    hole: usize,
    len: usize,
    value: T,
    less: &F,
) {
    let top = hole;
    let mut hole = hole;
    let mut second = hole;
    while second < (len - 1) / 2 {
        second = 2 * (second + 1);
        if less(&v[first + second], &v[first + second - 1]) {
            second -= 1;
        }
        v[first + hole] = v[first + second];
        hole = second;
    }
    if len & 1 == 0 && second == (len - 2) / 2 {
        second = 2 * (second + 1);
        v[first + hole] = v[first + second - 1];
        hole = second - 1;
    }
    // __push_heap
    let mut parent = hole.wrapping_sub(1) / 2;
    while hole > top && less(&v[first + parent], &value) {
        v[first + hole] = v[first + parent];
        hole = parent;
        parent = hole.wrapping_sub(1) / 2;
    }
    v[first + hole] = value;
}

// ---- boost pdqsort ----

const INSERTION_SORT_THRESHOLD: usize = 24;
const NINTHER_THRESHOLD: usize = 128;
const PARTIAL_INSERTION_SORT_LIMIT: usize = 8;

fn pdq_insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    begin: usize,
    end: usize,
    less: &F,
) {
    for cur in (begin + 1)..end {
        let mut sift = cur;
        let mut sift_1 = cur - 1;
        if less(&v[sift], &v[sift_1]) {
            let tmp = v[sift];
            loop {
                v[sift] = v[sift_1];
                sift -= 1;
                if sift == begin {
                    break;
                }
                sift_1 -= 1;
                if !less(&tmp, &v[sift_1]) {
                    break;
                }
            }
            v[sift] = tmp;
        }
    }
}

fn pdq_unguarded_insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    begin: usize,
    end: usize,
    less: &F,
) {
    for cur in (begin + 1)..end {
        let mut sift = cur;
        let mut sift_1 = cur - 1;
        if less(&v[sift], &v[sift_1]) {
            let tmp = v[sift];
            loop {
                v[sift] = v[sift_1];
                sift -= 1;
                sift_1 -= 1;
                if !less(&tmp, &v[sift_1]) {
                    break;
                }
            }
            v[sift] = tmp;
        }
    }
}

fn partial_insertion_sort<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    begin: usize,
    end: usize,
    less: &F,
) -> bool {
    if begin == end {
        return true;
    }
    let mut limit = 0;
    for cur in (begin + 1)..end {
        let mut sift = cur;
        let mut sift_1 = cur - 1;
        if less(&v[sift], &v[sift_1]) {
            let tmp = v[sift];
            loop {
                v[sift] = v[sift_1];
                sift -= 1;
                if sift == begin {
                    break;
                }
                sift_1 -= 1;
                if !less(&tmp, &v[sift_1]) {
                    break;
                }
            }
            v[sift] = tmp;
            limit += cur - sift;
        }
        if limit > PARTIAL_INSERTION_SORT_LIMIT {
            return false;
        }
    }
    true
}

fn sort2<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], a: usize, b: usize, less: &F) {
    if less(&v[b], &v[a]) {
        v.swap(a, b);
    }
}

fn sort3<T: Copy, F: Fn(&T, &T) -> bool>(v: &mut [T], a: usize, b: usize, c: usize, less: &F) {
    sort2(v, a, b, less);
    sort2(v, b, c, less);
    sort2(v, a, b, less);
}

fn partition_right<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    begin: usize,
    end: usize,
    less: &F,
) -> (usize, bool) {
    let pivot = v[begin];
    let mut first = begin;
    let mut last = end;
    let up = |first: &mut usize, v: &[T]| loop {
        *first += 1;
        if !less(&v[*first], &pivot) {
            break;
        }
    };
    let down = |last: &mut usize, v: &[T], guard: Option<usize>| loop {
        if guard.is_some_and(|g| g >= *last) {
            break;
        }
        *last -= 1;
        if less(&v[*last], &pivot) {
            break;
        }
    };
    up(&mut first, v);
    let guard = if first - 1 == begin {
        Some(first)
    } else {
        None
    };
    down(&mut last, v, guard);
    let already_partitioned = first >= last;
    while first < last {
        v.swap(first, last);
        up(&mut first, v);
        down(&mut last, v, None);
    }
    let pivot_pos = first - 1;
    v[begin] = v[pivot_pos];
    v[pivot_pos] = pivot;
    (pivot_pos, already_partitioned)
}

fn partition_left<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    begin: usize,
    end: usize,
    less: &F,
) -> usize {
    let pivot = v[begin];
    let mut first = begin;
    let mut last = end;
    let down = |last: &mut usize, v: &[T]| loop {
        *last -= 1;
        if !less(&pivot, &v[*last]) {
            break;
        }
    };
    let up = |first: &mut usize, v: &[T], guard: Option<usize>| loop {
        if guard.is_some_and(|g| *first >= g) {
            break;
        }
        *first += 1;
        if less(&pivot, &v[*first]) {
            break;
        }
    };
    down(&mut last, v);
    let guard = if last + 1 == end { Some(last) } else { None };
    up(&mut first, v, guard);
    while first < last {
        v.swap(first, last);
        down(&mut last, v);
        up(&mut first, v, None);
    }
    let pivot_pos = last;
    v[begin] = v[pivot_pos];
    v[pivot_pos] = pivot;
    pivot_pos
}

fn pdqsort_loop<T: Copy, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    mut begin: usize,
    end: usize,
    less: &F,
    mut bad_allowed: usize,
    mut leftmost: bool,
) {
    loop {
        let size = end - begin;
        if size < INSERTION_SORT_THRESHOLD {
            if leftmost {
                pdq_insertion_sort(v, begin, end, less);
            } else {
                pdq_unguarded_insertion_sort(v, begin, end, less);
            }
            return;
        }
        let s2 = size / 2;
        if size > NINTHER_THRESHOLD {
            sort3(v, begin, begin + s2, end - 1, less);
            sort3(v, begin + 1, begin + (s2 - 1), end - 2, less);
            sort3(v, begin + 2, begin + (s2 + 1), end - 3, less);
            sort3(v, begin + (s2 - 1), begin + s2, begin + (s2 + 1), less);
            v.swap(begin, begin + s2);
        } else {
            sort3(v, begin + s2, begin, end - 1, less);
        }
        if !leftmost && !less(&v[begin - 1], &v[begin]) {
            begin = partition_left(v, begin, end, less) + 1;
            continue;
        }
        let (pivot_pos, already_partitioned) = partition_right(v, begin, end, less);
        let l_size = pivot_pos - begin;
        let r_size = end - (pivot_pos + 1);
        let highly_unbalanced = l_size < size / 8 || r_size < size / 8;
        if highly_unbalanced {
            bad_allowed -= 1;
            if bad_allowed == 0 {
                make_heap(v, begin, end, less);
                sort_heap(v, begin, end, less);
                return;
            }
            if l_size >= INSERTION_SORT_THRESHOLD {
                v.swap(begin, begin + l_size / 4);
                v.swap(pivot_pos - 1, pivot_pos - l_size / 4);
                if l_size > NINTHER_THRESHOLD {
                    v.swap(begin + 1, begin + (l_size / 4 + 1));
                    v.swap(begin + 2, begin + (l_size / 4 + 2));
                    v.swap(pivot_pos - 2, pivot_pos - (l_size / 4 + 1));
                    v.swap(pivot_pos - 3, pivot_pos - (l_size / 4 + 2));
                }
            }
            if r_size >= INSERTION_SORT_THRESHOLD {
                v.swap(pivot_pos + 1, pivot_pos + (1 + r_size / 4));
                v.swap(end - 1, end - r_size / 4);
                if r_size > NINTHER_THRESHOLD {
                    v.swap(pivot_pos + 2, pivot_pos + (2 + r_size / 4));
                    v.swap(pivot_pos + 3, pivot_pos + (3 + r_size / 4));
                    v.swap(end - 2, end - (1 + r_size / 4));
                    v.swap(end - 3, end - (2 + r_size / 4));
                }
            }
        } else if already_partitioned
            && partial_insertion_sort(v, begin, pivot_pos, less)
            && partial_insertion_sort(v, pivot_pos + 1, end, less)
        {
            return;
        }
        pdqsort_loop(v, begin, pivot_pos, less, bad_allowed, leftmost);
        begin = pivot_pos + 1;
        leftmost = false;
    }
}

// ---- boost spreadsort integer_sort (u32 key, size_t Size_type, int_* constants) ----

const MAX_SPLITS: u32 = 11;
const MAX_FINISHING_SPLITS: u32 = MAX_SPLITS + 1;
const LOG_MEAN_BIN_SIZE: u32 = 2;
const LOG_MIN_SPLIT_COUNT: u32 = 9;
const LOG_FINISHING_COUNT: u32 = 31;
const MIN_SORT_SIZE: usize = 1000;

fn rough_log_2_size(input: u64) -> u32 {
    let mut result = 0;
    while result < 64 && (input >> result) != 0 {
        result += 1;
    }
    result
}

fn get_log_divisor(count: usize, log_range: u32) -> u32 {
    let mut log_divisor = log_range as i32 - rough_log_2_size(count as u64) as i32;
    if log_divisor <= 0 && log_range <= MAX_FINISHING_SPLITS {
        log_divisor = 0;
    } else {
        log_divisor += LOG_MEAN_BIN_SIZE as i32;
        if log_range as i32 - log_divisor > MAX_SPLITS as i32 {
            log_divisor = log_range as i32 - MAX_SPLITS as i32;
        }
    }
    log_divisor as u32
}

fn get_min_count(log_range: u32) -> usize {
    let min_size = LOG_MEAN_BIN_SIZE + LOG_MIN_SPLIT_COUNT;
    if LOG_FINISHING_COUNT < min_size && log_range <= min_size && log_range <= MAX_SPLITS {
        if log_range <= LOG_FINISHING_COUNT {
            return 1 << LOG_FINISHING_COUNT;
        }
        return 1 << log_range;
    }
    let base_iterations = MAX_SPLITS - LOG_MIN_SPLIT_COUNT;
    let base_range =
        ((base_iterations + 1) * (MAX_SPLITS + LOG_MIN_SPLIT_COUNT)) / 2 + LOG_MEAN_BIN_SIZE;
    if log_range < base_range {
        let mut result = LOG_MIN_SPLIT_COUNT;
        let mut offset = min_size;
        while offset < log_range {
            result += 1;
            offset += result;
        }
        if result + LOG_MEAN_BIN_SIZE >= 64 {
            return 1 << 63;
        }
        return 1 << (result + LOG_MEAN_BIN_SIZE);
    }
    let remainder = log_range - base_range;
    let bit_length = remainder.div_ceil(MAX_SPLITS) + base_iterations + min_size;
    if bit_length >= 64 {
        return 1 << 63;
    }
    1 << bit_length
}

// Returns true when already sorted; otherwise the (max, min) positions.
fn is_sorted_or_find_extremes<T, F: Fn(&T, &T) -> bool>(
    v: &[T],
    first: usize,
    last: usize,
    less: &F,
) -> Option<(usize, usize)> {
    let mut current = first;
    while !less(&v[current + 1], &v[current]) {
        current += 1;
        if current == last - 1 {
            return None;
        }
    }
    let mut max = current;
    let mut min = first;
    current += 1;
    while current < last {
        if less(&v[max], &v[current]) {
            max = current;
        } else if less(&v[current], &v[min]) {
            min = current;
        }
        current += 1;
    }
    Some((max, min))
}

#[allow(clippy::too_many_arguments)]
fn spreadsort_rec<T: Copy, K: Fn(&T) -> u32, F: Fn(&T, &T) -> bool>(
    v: &mut [T],
    first: usize,
    last: usize,
    bin_cache: &mut Vec<usize>,
    cache_offset: usize,
    bin_sizes: &mut [usize],
    key: &K,
    less: &F,
) {
    let Some((max, min)) = is_sorted_or_find_extremes(v, first, last, less) else {
        return;
    };
    let rshift = |t: &T, off: u32| key(t) >> off;
    let log_divisor = get_log_divisor(
        last - first,
        rough_log_2_size(rshift(&v[max], 0).wrapping_sub(rshift(&v[min], 0)) as u64),
    );
    let div_min = rshift(&v[min], log_divisor);
    let div_max = rshift(&v[max], log_divisor);
    let bin_count = div_max.wrapping_sub(div_min) as usize + 1;
    // size_bins
    bin_sizes[..bin_count].fill(0);
    let cache_end = cache_offset + bin_count;
    if cache_end > bin_cache.len() {
        bin_cache.resize(cache_end, 0);
    }
    let bin = |t: &T| (rshift(t, log_divisor).wrapping_sub(div_min)) as usize;
    for t in &v[first..last] {
        bin_sizes[bin(t)] += 1;
    }
    let bins = &mut bin_cache[cache_offset..cache_end];
    bins[0] = first;
    for u in 0..bin_count - 1 {
        bins[u + 1] = bins[u] + bin_sizes[u];
    }
    let mut next_bin_start = first;
    for ii in 0..bin_count - 1 {
        // swap_loop / inner_swap_loop
        next_bin_start += bin_sizes[ii];
        let mut current = bins[ii];
        while current < next_bin_start {
            loop {
                let target_bin = bin(&v[current]);
                if target_bin == ii {
                    break;
                }
                let b = bins[target_bin];
                bins[target_bin] += 1;
                let b_bin = bin(&v[b]);
                let tmp = if b_bin != ii {
                    let c = bins[b_bin];
                    bins[b_bin] += 1;
                    let tmp = v[c];
                    v[c] = v[b];
                    tmp
                } else {
                    v[b]
                };
                v[b] = v[current];
                v[current] = tmp;
            }
            current += 1;
        }
        bins[ii] = next_bin_start;
    }
    bins[bin_count - 1] = last;
    if log_divisor == 0 {
        return;
    }
    let max_count = get_min_count(log_divisor);
    let mut last_pos = first;
    for u in cache_offset..cache_end {
        let end = bin_cache[u];
        let count = end - last_pos;
        if count >= 2 {
            if count < max_count {
                pdqsort(&mut v[last_pos..end], less);
            } else {
                spreadsort_rec(v, last_pos, end, bin_cache, cache_end, bin_sizes, key, less);
            }
        }
        last_pos = bin_cache[u];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(seed: &mut u64) -> u64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *seed >> 33
    }

    fn check_sorted(v: &[(u32, u32)]) {
        assert!(v.windows(2).all(|w| w[0].0 <= w[1].0));
    }

    #[test]
    fn all_sorts_sort() {
        let mut seed = 5;
        let less = |a: &(u32, u32), b: &(u32, u32)| a.0 < b.0;
        for n in [0usize, 1, 2, 3, 5, 17, 24, 25, 130, 999, 1000, 5000, 40000] {
            for spread in [3u64, 100, 1 << 20, 1 << 31] {
                let base: Vec<(u32, u32)> = (0..n)
                    .map(|i| ((lcg(&mut seed) % spread) as u32, i as u32))
                    .collect();
                let mut a = base.clone();
                std_sort(&mut a, &less);
                check_sorted(&a);
                let mut b = base.clone();
                pdqsort(&mut b, &less);
                check_sorted(&b);
                let mut c = base.clone();
                integer_sort(&mut c, &|t: &(u32, u32)| t.0, &less);
                check_sorted(&c);
                let mut m: Vec<_> = base.iter().map(|t| t.1).collect();
                m.sort();
                for s in [&a, &b, &c] {
                    let mut got: Vec<_> = s.iter().map(|t| t.1).collect();
                    got.sort();
                    assert_eq!(got, m, "permutation");
                }
                if n > 0 {
                    let mut d = base.clone();
                    let nth = n / 3;
                    nth_element(&mut d, nth, &less);
                    assert_eq!(d[nth].0, a[nth].0);
                    assert!(d[..nth].iter().all(|t| t.0 <= a[nth].0));
                    assert!(d[nth + 1..].iter().all(|t| t.0 >= a[nth].0));
                }
            }
        }
        // Sorted and reversed inputs hit the already-sorted / partition_left / heap fallbacks.
        for n in [50usize, 3000] {
            let asc: Vec<(u32, u32)> = (0..n).map(|i| (i as u32, i as u32)).collect();
            let mut desc = asc.clone();
            desc.reverse();
            for base in [&asc, &desc] {
                let mut a = base.clone();
                std_sort(&mut a, &less);
                let mut b = base.clone();
                pdqsort(&mut b, &less);
                let mut c = base.clone();
                integer_sort(&mut c, &|t: &(u32, u32)| t.0, &less);
                assert_eq!(a, asc);
                assert_eq!(b, asc);
                assert_eq!(c, asc);
            }
        }
        // Exhausting the depth limit exercises the heap paths.
        let mut e: Vec<(u32, u32)> = (0..500)
            .map(|i| ((lcg(&mut seed) % 50) as u32, i))
            .collect();
        introsort_loop(&mut e, 0, 500, 0, &less);
        check_sorted(&e);
        let mut f: Vec<(u32, u32)> = (0..500)
            .map(|i| ((lcg(&mut seed) % 50) as u32, i))
            .collect();
        pdqsort_loop(&mut f, 0, 500, &less, 1, true);
        check_sorted(&f);
    }
}
