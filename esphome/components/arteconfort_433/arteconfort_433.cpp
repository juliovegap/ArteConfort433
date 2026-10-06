#include "arteconfort_433.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::arteconfort_433 {

static const char *const TAG = "arteconfort_433";

void ArteConfort433::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ArteConfort 433:\n"
                "  Protocol: %s%s\n"
                "  Bits per code: %u\n"
                "  Repeat: %u, gap: %u us\n"
                "  Sync: mark %u us, space %u us\n"
                "  Bit 1: mark %u us, space %u us\n"
                "  Bit 0: mark %u us, space %u us\n"
                "  Trailer mark: %u us\n"
                "  Patterns: %u\n"
                "  Transmitter: %s, receiver/analyzer: %s",
                this->protocol_, this->auto_detect_ ? " (auto-detect when receiving)" : "", this->bits_, this->repeat_,
                (unsigned) this->gap_us_, (unsigned) this->timing_.sync_mark_us,
                (unsigned) this->timing_.sync_space_us, (unsigned) this->timing_.one_mark(),
                (unsigned) this->timing_.one_space(), (unsigned) this->timing_.zero_mark(),
                (unsigned) this->timing_.zero_space(), (unsigned) this->timing_.trailer_mark_us, (unsigned) this->patterns_.size(), this->tx_ != nullptr ? "yes" : "no",
                this->receiving_ ? "yes" : "no");
}

bool ArteConfort433::has_pattern(const char *id) const {
  if (id == nullptr)
    return false;
  for (const auto &p : this->patterns_) {
    if (strcmp(p.id, id) == 0)
      return true;
  }
  return false;
}

const char *ArteConfort433::find_pattern_name_(uint64_t code) const {
  for (const auto &p : this->patterns_) {
    if (p.code == code)
      return p.id;
  }
  return nullptr;
}

bool ArteConfort433::send_pattern(const char *id) {
  if (id == nullptr) {
    return false;
  }
  for (const auto &p : this->patterns_) {
    if (strcmp(p.id, id) == 0) {
      return this->send_code(p.code, 0, 0);
    }
  }
  ESP_LOGW(TAG, "Pattern '%s' not found", id);
  return false;
}

bool ArteConfort433::send_code(uint64_t code, uint8_t bits, uint8_t repeat) {
  if (this->tx_ == nullptr) {
    ESP_LOGE(TAG, "No transmitter configured (add transmitter_id)");
    return false;
  }
  if (bits == 0)
    bits = this->bits_;
  if (repeat == 0)
    repeat = this->repeat_;
  if (bits > 64) {
    ESP_LOGW(TAG, "Invalid bit count: %u", bits);
    return false;
  }
  if (bits < 64 && (code >> bits) != 0) {
    ESP_LOGW(TAG, "Code 0x%llX does not fit in %u bits", (unsigned long long) code, bits);
    return false;
  }

  ESP_LOGD(TAG, "Sending 0x%llX (%u bits) x%u", (unsigned long long) code, bits, repeat);

  auto call = this->tx_->transmit();
  call.get_data()->set_carrier_frequency(0);  // pure OOK: the CC1101 generates the carrier
  encode_frame(call.get_data(), this->timing_, code, bits);
  call.set_send_times(repeat);
  call.set_send_wait(this->gap_us_);
  call.perform();

  if (bits == this->bits_ && !this->send_callbacks_.empty()) {
    const char *name = this->find_pattern_name_(code);
    if (name != nullptr) {
      for (auto &cb : this->send_callbacks_)
        cb(name);
    }
  }
  return true;
}

static std::string timing_yaml_(const FrameAnalysis &a) {
  char buf[160];
  if (a.asymmetric) {
    snprintf(buf, sizeof(buf), "bit1_mark_us: %u, bit1_space_us: %u, bit0_mark_us: %u, bit0_space_us: %u",
             (unsigned) a.one_mark_us, (unsigned) a.one_space_us, (unsigned) a.zero_mark_us, (unsigned) a.zero_space_us);
  } else {
    snprintf(buf, sizeof(buf), "short_us: %u, long_us: %u", (unsigned) a.short_us, (unsigned) a.long_us);
  }
  return buf;
}

static void log_timing_(const FrameAnalysis &a) {
  if (a.asymmetric) {
    ESP_LOGI(TAG, "  Asymmetric timing: bit 1 = %u/%u us, bit 0 = %u/%u us (mark/space)", (unsigned) a.one_mark_us,
             (unsigned) a.one_space_us, (unsigned) a.zero_mark_us, (unsigned) a.zero_space_us);
  } else {
    ESP_LOGI(TAG, "  Short ~%u us, long ~%u us", (unsigned) a.short_us, (unsigned) a.long_us);
  }
}

namespace {

// Distinct codes seen in one reception, with how many times each one repeated.
struct Seen {
  uint64_t code;
  uint8_t bits;
  unsigned count;
  FrameAnalysis analysis;
};

struct Decoded {
  FrameAnalysis a;
  std::vector<Seen> seen;
  unsigned undecoded{0};
};

// Known protocols a receive-only hub tries in `auto` mode: name + number of data bits per frame.
struct ProtocolInfo {
  const char *name;
  uint8_t bits;
};
const ProtocolInfo PROTOCOLS[] = {{"arteconfort", 30}, {"sulion", 32}};

Decoded decode_reception(const remote_base::RawTimings &raw, uint8_t bits) {
  Decoded d;
  d.a = analyze_frame(raw, bits);
  auto note = [&d](const FrameAnalysis &f) {
    for (auto &x : d.seen) {
      if (x.code == f.code && x.bits == f.bits) {
        x.count++;
        return;
      }
    }
    d.seen.push_back({f.code, f.bits, 1, f});
  };
  if (d.a.ok) {
    note(d.a);
    return d;
  }
  if (raw.size() <= 2 * static_cast<size_t>(bits) + 8)
    return d;
  // Several repetitions can arrive glued together (gap shorter than the receiver's idle time):
  // cut at every long mark / long space and decode each piece.
  std::vector<uint32_t> lens;
  for (int32_t v : raw)
    lens.push_back(static_cast<uint32_t>(std::abs(v)));
  std::sort(lens.begin(), lens.end());
  const uint32_t median = lens[lens.size() / 2];
  std::vector<size_t> cuts;  // start of each candidate frame
  for (size_t i = 0; i < raw.size(); i++) {
    const uint32_t len = static_cast<uint32_t>(std::abs(raw[i]));
    if (len <= median * 4)
      continue;
    if (raw[i] > 0)
      cuts.push_back(i);  // long mark: frame starts here
    else if (i > 0)
      cuts.push_back(i - 1);  // long space: the frame starts with the mark just before it
  }
  for (size_t c = 0; c < cuts.size(); c++) {
    const size_t end = c + 1 < cuts.size() ? cuts[c + 1] : raw.size();
    if (end - cuts[c] < 6)
      continue;
    remote_base::RawTimings piece(raw.begin() + cuts[c], raw.begin() + end);
    FrameAnalysis f = analyze_frame(piece, bits);
    if (!f.ok) {
      d.undecoded++;
      continue;
    }
    // Folded piece = [mark][gap][bits-1 data bits]. The data bits are followed by their own last bit, which is the mark
    // that starts the NEXT piece. The mark at the start of this piece belongs to the frame before (and at the very start
    // of a reception it may even be a leftover of the previous reception), so it must not be used when a next piece exists.
    if (f.folded && c + 1 < cuts.size() && raw[cuts[c + 1]] > 0 && f.one_mark_us > 0 && f.zero_mark_us > 0) {
      const bool is_one = std::abs(raw[cuts[c + 1]]) > static_cast<int32_t>((f.one_mark_us + f.zero_mark_us) / 2);
      f.code = (f.code & ~static_cast<uint64_t>(1)) | (is_one ? 1u : 0u);
    }
    note(f);
  }
  // The code seen most often is the real one (a noisy piece may decode to something else).
  unsigned best = 0;
  for (auto &x : d.seen) {
    if (x.count > best) {
      best = x.count;
      d.a = x.analysis;
    }
  }
  return d;
}

}  // namespace

bool ArteConfort433::on_receive(remote_base::RemoteReceiveData data) {
  const remote_base::RawTimings &raw = data.get_raw_data();

  // Which protocols to try: all of them in `auto` mode, otherwise the configured one.
  Decoded dec;
  const char *proto = this->protocol_;
  uint8_t proto_bits = this->bits_;
  bool matched = false;
  if (this->auto_detect_) {
    for (const ProtocolInfo &pi : PROTOCOLS) {
      Decoded t = decode_reception(raw, pi.bits);
      if (t.a.ok) {
        dec = std::move(t);
        proto = pi.name;
        proto_bits = pi.bits;
        matched = true;
        break;
      }
    }
  } else {
    dec = decode_reception(raw, this->bits_);
    matched = dec.a.ok;
  }
  if (!matched) {
    ESP_LOGD(TAG, "Not an ArteConfort / Sulion frame (%d pulses)%s%s", (int) data.size(), dec.a.error != nullptr ? ": " : "",
             dec.a.error != nullptr ? dec.a.error : "");
    return false;  // let the raw dumper / other listeners see it too
  }
  const FrameAnalysis &a = dec.a;
  this->last_analysis_ = a;
  this->last_protocol_ = proto;
  if (dec.seen.size() > 1 || dec.undecoded > 0) {
    // A button that sends several different frames, or damaged repetitions, explains buttons that
    // "decode fine" but do not work when replayed: show all of them.
    ESP_LOGW(TAG, "Reception of %d pulses contains %u distinct code(s); %u piece(s) could not be decoded:", (int) data.size(),
             (unsigned) dec.seen.size(), dec.undecoded);
    for ([[maybe_unused]] auto &x : dec.seen)
      ESP_LOGW(TAG, "  0x%llX (%u bits) x%u", (unsigned long long) x.code, x.bits, x.count);
  } else if (!dec.seen.empty() && dec.seen[0].count > 1) {
    ESP_LOGI(TAG, "Reception of %d pulses = %u repetitions of the same frame", (int) data.size(), dec.seen[0].count);
  }

  [[maybe_unused]] const char *name = this->find_pattern_name_(a.code);
  ESP_LOGI(TAG, "Received 0x%llX (%u bits)%s%s%s", (unsigned long long) a.code, a.bits, name != nullptr ? " = pattern '" : "",
           name != nullptr ? name : "", name != nullptr ? "'" : "");
  if (this->auto_detect_) {
    ESP_LOGI(TAG, "  Protocol: %s (use `protocol: %s` in the hub)", proto, proto);
  } else if (a.bits != proto_bits) {
    ESP_LOGW(TAG, "  Decoded %u bits but 'bits: %u' is configured", a.bits, proto_bits);
  }
  if (a.plain) {
    // Estimate the gap from the time between two receptions of the same code (resolution ~1-2 ms).
    uint32_t dur = 0;
    for (int32_t v : raw)
      dur += static_cast<uint32_t>(std::abs(v));
    const uint32_t now = millis();
    char gap_txt[64] = "unknown (repeat the capture while holding a key)";
    if (this->have_last_rx_ && this->last_rx_code_ == a.code && now - this->last_rx_ms_ < 500) {
      const uint32_t period_us = (now - this->last_rx_ms_) * 1000;
      // With a trailing mark the reception ends right before the gap; otherwise the last space is part of it.
      const uint32_t last_space = a.trailer_us != 0 ? 0 : ((a.code & 1ULL) ? a.one_space_us : a.zero_space_us);
      const uint32_t used = dur + last_space;
      snprintf(gap_txt, sizeof(gap_txt), "%u", (unsigned) (period_us > used ? period_us - used : 0));
    }
    this->have_last_rx_ = true;
    this->last_rx_code_ = a.code;
    this->last_rx_ms_ = now;
    if (a.trailer_us != 0) {
      ESP_LOGI(TAG, "  %u bits + a final %u us mark, no sync field; idle gap after it", a.bits, (unsigned) a.trailer_us);
    } else {
      ESP_LOGI(TAG, "  %u bits, no sync field; the space after the last mark is the idle gap between repetitions", a.bits);
    }
    log_timing_(a);
    char trailer_txt[40] = "";
    if (a.trailer_us != 0)
      snprintf(trailer_txt, sizeof(trailer_txt), "trailer_mark_us: %u, ", (unsigned) a.trailer_us);
    ESP_LOGI(TAG, "  YAML: bits: %u, gap_us: %s (estimate), %ssync_mark_us: 0, sync_space_us: 0, %s", a.bits, gap_txt,
             trailer_txt, timing_yaml_(a).c_str());
    return false;
  }
  if (a.folded) {
    ESP_LOGI(TAG, "  %u bits, then an idle gap of ~%u us (the mark before it is the last bit)", a.bits, (unsigned) a.gap_us);
    log_timing_(a);
    ESP_LOGI(TAG, "  YAML: bits: %u, gap_us: %u, sync_mark_us: 0, sync_space_us: 0, %s", a.bits, (unsigned) a.gap_us,
             timing_yaml_(a).c_str());
    return false;
  }
  if (a.sync_merged) {
    ESP_LOGI(TAG, "  First pulse %u us = sync mark (~%u us) running straight into the first data mark (no sync space)",
             (unsigned) a.first_mark_us, (unsigned) a.sync_mark_us);
  } else {
    ESP_LOGI(TAG, "  Sync: mark %u us, then space %u us", (unsigned) a.sync_mark_us, (unsigned) a.sync_space_us);
  }
  log_timing_(a);
  ESP_LOGI(TAG, "  YAML: sync_mark_us: %u, sync_space_us: %u, %s", (unsigned) a.sync_mark_us, (unsigned) a.sync_space_us,
           timing_yaml_(a).c_str());
  return false;
}

#ifdef USE_BUTTON
void ArteConfort433Button::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->send_pattern(this->pattern_);
  }
}
#endif

}  // namespace esphome::arteconfort_433
