#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

#include "esphome/core/defines.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/remote_base/remote_base.h"
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

namespace esphome::arteconfort_433 {

/// Timing of one ArteConfort frame, all values in microseconds.
///
///   frame = [sync_mark][sync_space] bit(N-1) ... bit(0)
///   bit 1 = mark(long)  + space(short)
///   bit 0 = mark(short) + space(long)
///
/// Remotes whose "short" mark differs from their "short" space (e.g. Sulion: bit 1 = 1113/500 us,
/// bit 0 = 300/1326 us) set the four `bit*_us` fields; 0 means "derive from short_us / long_us".
///
/// The real remote (measured) sends 30 bits back to back and then idles for 5000 us (that gap is
/// added by the hub as the wait between repetitions, `gap_us`), so no sync field is used by default.
/// `sync_*` remain available for other remotes. If sync_space_us == 0 the sync mark is merged with
/// the first data mark (what firmware <= v5.8 put on the air: a 6120 us mark).
struct FrameTiming {
  uint32_t sync_mark_us{0};
  uint32_t sync_space_us{0};
  uint32_t short_us{380};
  uint32_t long_us{1130};
  uint32_t one_mark_us{0};    // bit 1 mark  (0 = long_us)
  uint32_t one_space_us{0};   // bit 1 space (0 = short_us)
  uint32_t zero_mark_us{0};   // bit 0 mark  (0 = short_us)
  uint32_t zero_space_us{0};  // bit 0 space (0 = long_us)
  uint32_t trailer_mark_us{0};  // extra mark after the last bit (0 = none); its space is the idle gap
  uint32_t one_mark() const { return one_mark_us != 0 ? one_mark_us : long_us; }
  uint32_t one_space() const { return one_space_us != 0 ? one_space_us : short_us; }
  uint32_t zero_mark() const { return zero_mark_us != 0 ? zero_mark_us : short_us; }
  uint32_t zero_space() const { return zero_space_us != 0 ? zero_space_us : long_us; }
};

/// Appends one frame to `out`, merging adjacent segments of the same level so the
/// result is a clean alternating mark/space sequence. Hardware-free: unit-testable on a PC.
inline void encode_frame(remote_base::RemoteTransmitData *out, const FrameTiming &t, uint64_t code, uint8_t bits) {
  int32_t pending = 0;  // >0 = accumulating a mark, <0 = accumulating a space

  auto flush = [&]() {
    if (pending > 0)
      out->mark(static_cast<uint32_t>(pending));
    else if (pending < 0)
      out->space(static_cast<uint32_t>(-pending));
    pending = 0;
  };
  auto push = [&](bool mark, uint32_t len) {
    if (len == 0)
      return;
    const int32_t v = mark ? static_cast<int32_t>(len) : -static_cast<int32_t>(len);
    if (pending != 0 && ((pending > 0) == mark)) {
      pending += v;
      return;
    }
    flush();
    pending = v;
  };

  out->reserve(2 + 2u * bits);
  push(true, t.sync_mark_us);
  push(false, t.sync_space_us);
  for (int i = static_cast<int>(bits) - 1; i >= 0; i--) {
    const bool bit = ((code >> i) & 1ULL) != 0;
    push(true, bit ? t.one_mark() : t.zero_mark());
    push(false, bit ? t.one_space() : t.zero_space());
  }
  push(true, t.trailer_mark_us);
  flush();
}

/// Result of decoding one received frame (see analyze_frame).
struct FrameAnalysis {
  bool ok{false};
  const char *error{nullptr};  // set when !ok (string literal)
  uint64_t code{0};
  uint8_t bits{0};
  bool sync_merged{false};     // sync mark runs straight into the first data mark
  uint32_t first_mark_us{0};   // first pulse exactly as received
  uint32_t sync_mark_us{0};    // estimated sync mark (for sync_mark_us in the YAML)
  uint32_t sync_space_us{0};   // 0 when merged
  uint32_t short_us{0};        // measured, averaged over all short pulses
  uint32_t long_us{0};         // measured, averaged over all long pulses
  uint32_t one_mark_us{0};     // measured per category (bit 1 mark/space, bit 0 mark/space)
  uint32_t one_space_us{0};
  uint32_t zero_mark_us{0};
  uint32_t zero_space_us{0};
  bool asymmetric{false};      // mark and space "short"/"long" lengths differ noticeably
  uint32_t trailer_us{0};      // plain layout: length of the lone final mark after the last bit (0 = none)
  bool plain{false};           // no sync field: the frame is just the bits (a lone last mark is the final bit)
  bool folded{false};          // the mark before the gap was folded in as the last bit (30-bit layout)
  uint32_t gap_us{0};          // idle gap after the last bit (only when folded)
};

/// Decodes one received frame without knowing its timing in advance.
///
/// `raw` uses ESPHome's convention: positive = mark, negative = space (microseconds).
/// `expected_bits` (0 = unknown) is only needed to tell "sync mark merged with the first bit"
/// apart from "sync followed by a short space": the element count differs in each case.
///
/// Short/long pulses are separated with a threshold derived from the data itself, so it also
/// works on remotes whose timing differs from the defaults. A bit is read from its mark and
/// from its space; the two must agree. The first mark (when merged with the sync) and the last
/// space (when swallowed by the inter-frame gap) are recovered from the other half of the pair.
/// Header-only and hardware-free so it can be unit-tested on a PC.
inline FrameAnalysis analyze_frame(const remote_base::RawTimings &raw, uint8_t expected_bits = 0) {
  FrameAnalysis r;
  auto fail = [&r](const char *msg) {
    r.ok = false;
    r.error = msg;
    return r;
  };

  size_t n = raw.size();
  if (n < 6)
    return fail("too few pulses");
  if (raw[0] <= 0)
    return fail("frame does not start with a mark");

  // Reference "long" length: 75th percentile once the (at most 3) outliers are removed.
  std::vector<uint32_t> lens;
  lens.reserve(n);
  for (int32_t v : raw)
    lens.push_back(static_cast<uint32_t>(std::abs(v)));
  std::vector<uint32_t> sorted(lens);
  std::sort(sorted.begin(), sorted.end());
  const size_t keep = sorted.size() > 6 ? sorted.size() - 3 : sorted.size();
  const uint32_t p75 = sorted[(keep * 3) / 4];
  auto is_big = [p75](uint32_t len) { return len > (p75 * 5) / 2; };

  // A trailing big space is just the gap before the next repetition.
  if (raw[n - 1] < 0 && is_big(lens[n - 1]))
    n--;
  if (n < 6)
    return fail("too few pulses");

  // Sync is either a long mark (merged with, or followed by, a space) or a normal mark followed
  // by a long space (the real remote: ~380 us mark + ~6.1 ms space).
  // Third layout ("plain"): no sync field at all, the frame is just `expected_bits` bits and the space of
  // the last one is the idle gap that ended the reception (a lone final mark). Needs the bit count.
  bool plain = false;
  uint32_t trailer = 0;
  if (!is_big(lens[0]) && !(raw[1] < 0 && is_big(lens[1]))) {
    const size_t b = expected_bits;
    if (b == 0 || !(n == 2 * b || n == 2 * b - 1 || n == 2 * b + 1))
      return fail("no sync pulse (neither a long first mark nor a long first space)");
    plain = true;
    if (n == 2 * b + 1) {
      // `bits` pairs followed by a lone final mark (a stop pulse); its space is the idle gap.
      if (raw[n - 1] <= 0)
        return fail("levels do not alternate");
      trailer = lens[n - 1];
      n--;
    }
  }

  // Layout: separate sync space, or sync mark merged with the first data mark?
  bool merged;
  if (plain) {
    merged = false;
  } else if (raw[1] < 0 && is_big(lens[1])) {
    merged = false;
  } else if (expected_bits > 0) {
    const size_t b = expected_bits;
    if (n == 2 * b || n == 2 * b - 1)
      merged = true;
    else if (n == 2 * b + 2 || n == 2 * b + 1)
      merged = false;
    else
      return fail("pulse count does not match the configured number of bits");
  } else {
    merged = true;
  }

  // Pairs: (mark, space). Merged layout: pair 0 is (merged first mark, raw[1]) and the next
  // complete pair starts at raw[2]. Separate layout: raw[0], raw[1] are sync mark/space and the
  // first data pair also starts at raw[2].
  const size_t data_start = plain ? 0 : 2;
  struct Pair {
    int32_t mark;   // 0 = unknown (merged with sync)
    int32_t space;  // 0 = unknown (swallowed by the gap)
  };
  std::vector<Pair> pairs;
  if (merged)
    pairs.push_back({0, raw[1]});
  for (size_t i = data_start; i < n; i += 2) {
    Pair p{raw[i], i + 1 < n ? raw[i + 1] : 0};
    pairs.push_back(p);
  }
  if (pairs.empty() || pairs.size() > 64)
    return fail("unsupported number of bits");
  if (merged && raw[1] >= 0)
    return fail("levels do not alternate");
  for (size_t k = merged ? 1 : 0; k < pairs.size(); k++) {
    if (pairs[k].mark <= 0 || pairs[k].space > 0)
      return fail("levels do not alternate");
  }

  // Short/long threshold from the data pulses themselves (every bit has one of each).
  std::vector<uint32_t> d;
  for (size_t k = 0; k < pairs.size(); k++) {
    if (pairs[k].mark != 0)
      d.push_back(static_cast<uint32_t>(pairs[k].mark));
    if (pairs[k].space != 0)
      d.push_back(static_cast<uint32_t>(-pairs[k].space));
  }
  for (uint32_t v : d) {
    if (is_big(v))
      return fail("unexpected long pulse inside the data");
  }
  if (d.size() < 4)
    return fail("too few data pulses");
  std::sort(d.begin(), d.end());
  const uint32_t thr = (d[d.size() / 4] + d[(d.size() * 3) / 4]) / 2;
  uint64_t s_sum = 0, l_sum = 0;
  uint32_t s_cnt = 0, l_cnt = 0;
  for (uint32_t v : d) {
    if (v < thr) {
      s_sum += v;
      s_cnt++;
    } else {
      l_sum += v;
      l_cnt++;
    }
  }
  if (s_cnt == 0 || l_cnt == 0)
    return fail("no distinction between short and long pulses");
  const uint32_t short_us = static_cast<uint32_t>(s_sum / s_cnt);
  const uint32_t long_us = static_cast<uint32_t>(l_sum / l_cnt);
  if (long_us * 2 < short_us * 3)
    return fail("short and long pulses are not clearly different");

  uint64_t code = 0;
  uint64_t m1_sum = 0, m0_sum = 0, s1_sum = 0, s0_sum = 0;
  uint32_t m1_cnt = 0, m0_cnt = 0, s1_cnt = 0, s0_cnt = 0;
  for (size_t k = 0; k < pairs.size(); k++) {
    int bit_from_mark = -1, bit_from_space = -1;
    if (pairs[k].mark != 0)
      bit_from_mark = static_cast<uint32_t>(pairs[k].mark) > thr ? 1 : 0;
    if (pairs[k].space != 0)
      bit_from_space = static_cast<uint32_t>(-pairs[k].space) > thr ? 0 : 1;
    if (bit_from_mark < 0 && bit_from_space < 0)
      return fail("a bit has neither mark nor space");
    if (bit_from_mark >= 0 && bit_from_space >= 0 && bit_from_mark != bit_from_space)
      return fail("mark and space of a bit disagree (noisy frame)");
    const int bit = bit_from_mark >= 0 ? bit_from_mark : bit_from_space;
    code = (code << 1) | static_cast<uint64_t>(bit);
    if (pairs[k].mark != 0) {
      (bit ? m1_sum : m0_sum) += static_cast<uint32_t>(pairs[k].mark);
      (bit ? m1_cnt : m0_cnt)++;
    }
    if (pairs[k].space != 0) {
      (bit ? s1_sum : s0_sum) += static_cast<uint32_t>(-pairs[k].space);
      (bit ? s1_cnt : s0_cnt)++;
    }
  }
  r.one_mark_us = m1_cnt ? static_cast<uint32_t>(m1_sum / m1_cnt) : long_us;
  r.one_space_us = s1_cnt ? static_cast<uint32_t>(s1_sum / s1_cnt) : short_us;
  r.zero_mark_us = m0_cnt ? static_cast<uint32_t>(m0_sum / m0_cnt) : short_us;
  r.zero_space_us = s0_cnt ? static_cast<uint32_t>(s0_sum / s0_cnt) : long_us;
  {
    auto differ = [](uint32_t a, uint32_t b) {
      const uint32_t hi = a > b ? a : b;
      const uint32_t lo = a > b ? b : a;
      return (hi - lo) * 100 > hi * 15;  // more than 15 % apart
    };
    r.asymmetric = differ(r.one_mark_us, r.zero_space_us) || differ(r.one_space_us, r.zero_mark_us);
  }

  r.ok = true;
  r.error = nullptr;
  r.code = code;
  r.bits = static_cast<uint8_t>(pairs.size());
  r.sync_merged = merged;
  r.first_mark_us = lens[0];
  r.short_us = short_us;
  r.long_us = long_us;
  if (plain) {
    r.plain = true;
    r.trailer_us = trailer;
    r.sync_mark_us = 0;
    r.sync_space_us = 0;
    return r;
  }
  // Layout used by the real remote: [last bit's mark][gap][bits ...] repeated, i.e. every frame is
  // really `expected_bits` bits and the bit just before the gap belongs to the NEXT frame's end.
  // When the caller expects one more bit than the pairs we found, fold that mark in as the last bit.
  if (!merged && expected_bits > 0 && pairs.size() + 1 == expected_bits && pairs.size() < 64) {
    const uint32_t m = lens[0];
    const bool is_long = m > thr;
    const uint32_t ref = is_long ? r.one_mark_us : r.zero_mark_us;
    if (m * 100 >= ref * 65 && m * 100 <= ref * 135) {
      r.code = (code << 1) | (is_long ? 1ULL : 0ULL);
      r.bits = static_cast<uint8_t>(pairs.size() + 1);
      r.folded = true;
      const uint32_t own_space = is_long ? r.one_space_us : r.zero_space_us;
      r.gap_us = lens[1] > own_space ? lens[1] - own_space : lens[1];
      r.sync_merged = false;
      r.first_mark_us = lens[0];
      r.short_us = short_us;
      r.long_us = long_us;
      r.sync_mark_us = 0;
      r.sync_space_us = 0;
      return r;
    }
  }

  if (merged) {
    const bool first_bit = ((code >> (pairs.size() - 1)) & 1ULL) != 0;
    const uint32_t first_bit_mark = first_bit ? r.one_mark_us : r.zero_mark_us;
    r.sync_mark_us = lens[0] > first_bit_mark ? lens[0] - first_bit_mark : lens[0];
    r.sync_space_us = 0;
  } else {
    r.sync_mark_us = lens[0];
    r.sync_space_us = lens[1];
  }
  return r;
}

struct Pattern {
  const char *id;  // string literal emitted by the codegen (lives forever)
  uint64_t code;
};

class ArteConfort433 : public Component, public remote_base::RemoteReceiverListener {
 public:
  void dump_config() override;

  void set_transmitter(remote_base::RemoteTransmitterBase *tx) { this->tx_ = tx; }
  void set_receiving(bool receiving) { this->receiving_ = receiving; }
  void set_bits(uint8_t bits) { this->bits_ = bits; }
  void set_repeat(uint8_t repeat) { this->repeat_ = repeat; }
  void set_gap_us(uint32_t gap_us) { this->gap_us_ = gap_us; }
  void set_sync_mark_us(uint32_t v) { this->timing_.sync_mark_us = v; }
  void set_sync_space_us(uint32_t v) { this->timing_.sync_space_us = v; }
  void set_short_us(uint32_t v) { this->timing_.short_us = v; }
  void set_long_us(uint32_t v) { this->timing_.long_us = v; }
  void set_trailer_mark_us(uint32_t v) { this->timing_.trailer_mark_us = v; }
  /// Name of the protocol preset this hub was configured with (for logging only).
  void set_protocol(const char *name) { this->protocol_ = name; }
  /// Receive-only hubs may try every known protocol (see PROTOCOLS in the .cpp).
  void set_auto_detect(bool v) { this->auto_detect_ = v; }
  /// The last frame successfully decoded by on_receive (ok == false until one was seen). For tests.
  const FrameAnalysis &last_analysis() const { return this->last_analysis_; }
  const char *last_protocol() const { return this->last_protocol_; }
  void set_one_mark_us(uint32_t v) { this->timing_.one_mark_us = v; }
  void set_one_space_us(uint32_t v) { this->timing_.one_space_us = v; }
  void set_zero_mark_us(uint32_t v) { this->timing_.zero_mark_us = v; }
  void set_zero_space_us(uint32_t v) { this->timing_.zero_space_us = v; }
  void add_pattern(const char *id, uint64_t code) { this->patterns_.push_back({id, code}); }

  /// Called (with the pattern name) after every successful transmission of a known pattern,
  /// whether it came from a button, the fan entity or an action. Used to keep entities in sync.
  void add_on_send_callback(std::function<void(const char *)> &&callback) {
    this->send_callbacks_.push_back(std::move(callback));
  }

  bool has_pattern(const char *id) const;
  bool send_pattern(const char *id);
  /// bits == 0 / repeat == 0 mean "use the configured default".
  bool send_code(uint64_t code, uint8_t bits = 0, uint8_t repeat = 0);

  /// Receiver side (only active when `receiver_id` is configured): logs every decoded frame.
  bool on_receive(remote_base::RemoteReceiveData data) override;

 protected:
  const char *find_pattern_name_(uint64_t code) const;

  remote_base::RemoteTransmitterBase *tx_{nullptr};
  FrameTiming timing_{};
  std::vector<Pattern> patterns_;
  std::vector<std::function<void(const char *)>> send_callbacks_;
  uint32_t gap_us_{5000};
  const char *protocol_{"arteconfort"};
  bool auto_detect_{false};
  FrameAnalysis last_analysis_{};
  const char *last_protocol_{nullptr};
  // Receiver bookkeeping used to estimate the idle gap between two repetitions of the same code.
  bool have_last_rx_{false};
  uint64_t last_rx_code_{0};
  uint32_t last_rx_ms_{0};
  uint8_t bits_{30};
  uint8_t repeat_{4};
  bool receiving_{false};
};

#ifdef USE_BUTTON
class ArteConfort433Button : public button::Button {
 public:
  void set_parent(ArteConfort433 *parent) { this->parent_ = parent; }
  void set_pattern(const char *pattern) { this->pattern_ = pattern; }

 protected:
  void press_action() override;

  ArteConfort433 *parent_{nullptr};
  const char *pattern_{nullptr};
};
#endif

template<typename... Ts> class SendPatternAction : public Action<Ts...>, public Parented<ArteConfort433> {
 public:
  TEMPLATABLE_VALUE(std::string, pattern)
  void play(const Ts &...x) override { this->parent_->send_pattern(this->pattern_.value(x...).c_str()); }
};

template<typename... Ts> class SendCodeAction : public Action<Ts...>, public Parented<ArteConfort433> {
 public:
  TEMPLATABLE_VALUE(uint64_t, code)
  TEMPLATABLE_VALUE(uint8_t, bits)
  TEMPLATABLE_VALUE(uint8_t, repeat)
  void play(const Ts &...x) override {
    this->parent_->send_code(this->code_.value(x...), this->bits_.value_or(x..., 0), this->repeat_.value_or(x..., 0));
  }
};

}  // namespace esphome::arteconfort_433
