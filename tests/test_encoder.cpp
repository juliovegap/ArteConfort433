// Host-side unit tests for the ArteConfort frame encoder (real ESPHome headers).
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <utility>

#include "esphome/components/arteconfort_433/arteconfort_433.h"

using namespace esphome;
using namespace esphome::arteconfort_433;
using Seg = std::vector<std::pair<bool, uint32_t>>;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      std::printf("FAIL line %d: %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

// Literal transcription of the v5.8 send_code() pin waveform, adjacent levels merged.
static Seg legacy_v58_frame(uint64_t code, int bits) {
  Seg seg;
  seg.push_back({true, 6120});
  for (int i = bits - 1; i >= 0; i--) {
    if ((code >> i) & 1ULL) { seg.push_back({true, 1130}); seg.push_back({false, 380}); }
    else { seg.push_back({true, 380}); seg.push_back({false, 1130}); }
  }
  Seg merged;
  for (auto &s : seg) {
    if (!merged.empty() && merged.back().first == s.first) merged.back().second += s.second;
    else merged.push_back(s);
  }
  return merged;
}

static Seg encode(const FrameTiming &t, uint64_t code, uint8_t bits) {
  remote_base::RemoteTransmitData d;
  encode_frame(&d, t, code, bits);
  Seg out;
  for (int32_t v : d.get_data()) out.push_back({v > 0, static_cast<uint32_t>(v > 0 ? v : -v)});
  return out;
}

static bool decode(const Seg &s, const FrameTiming &t, int bits, uint64_t *code) {
  if (s.size() != static_cast<size_t>(2 + 2 * bits)) return false;
  if (!(s[0].first && s[0].second == t.sync_mark_us)) return false;
  if (!(!s[1].first && s[1].second == t.sync_space_us)) return false;
  uint64_t v = 0;
  for (int i = 0; i < bits; i++) {
    auto m = s[2 + 2 * i], sp = s[3 + 2 * i];
    if (!m.first || sp.first) return false;
    if (m.second == t.long_us && sp.second == t.short_us) v = (v << 1) | 1;
    else if (m.second == t.short_us && sp.second == t.long_us) v = (v << 1);
    else return false;
  }
  *code = v;
  return true;
}

static const uint64_t CODES[] = {0x1EA0DD91ULL, 0x1EA0DDE8ULL, 0x1EA0DDC8ULL, 0x1EA0DDA9ULL, 0x1EA0DD89ULL, 0x1EA0DD6AULL,
                                 0x1EA0DD4AULL, 0x1EA0DD0BULL, 0x1EA0DD2BULL, 0x1EA0DC95ULL, 0x1EA0DC37ULL, 0x1EA0DC76ULL,
                                 0x1EA0DCB5ULL, 0x1EA0DDB1ULL, 0x1EA0DD72ULL, 0x1EA0DDD0ULL, 0x1EA0DCF4ULL, 0x1EA0DD33ULL};

int main() {
  FrameTiming legacy{6120, 0, 380, 1130};  // v5.8 waveform (sync merged with the first mark)
  for (uint64_t c : CODES)
    CHECK(encode(legacy, c, 29) == legacy_v58_frame(c, 29), "legacy mismatch 0x%llX", (unsigned long long) c);

  FrameTiming sep{6120, 5740, 380, 1130};
  for (const FrameTiming *t : {&legacy, &sep})
    for (uint64_t c : CODES) {
      auto s = encode(*t, c, 29);
      for (size_t i = 0; i < s.size(); i++) {
        CHECK(s[i].second > 0, "zero-length segment");
        if (i) CHECK(s[i].first != s[i - 1].first, "no alternation at %zu", i);
      }
    }

  for (uint64_t c : CODES) {
    uint64_t back = 0;
    CHECK(decode(encode(sep, c, 29), sep, 29, &back) && back == c, "roundtrip 0x%llX", (unsigned long long) c);
  }

  {
    uint32_t total = 0;
    for (auto &s : encode(legacy, 0x1EA0DD91ULL, 29)) total += s.second;
    CHECK(total == 6120u + 29u * 1510u, "duration %u", total);
  }

  {
    uint64_t back = 0;
    CHECK(decode(encode(sep, 1, 1), sep, 1, &back) && back == 1, "1-bit");
    CHECK(decode(encode(sep, 0, 1), sep, 1, &back) && back == 0, "1-bit zero");
    uint64_t big = 0x8000000000000001ULL;
    CHECK(decode(encode(sep, big, 64), sep, 64, &back) && back == big, "64-bit");
    CHECK(decode(encode(sep, 0x5, 3), sep, 3, &back) && back == 0x5, "MSB first");
  }

  {
    FrameTiming t{0, 0, 380, 1130};
    auto s = encode(t, 0x2, 2);  // bits 1,0
    CHECK(s.size() == 4 && s[0].second == 1130 && s[1].second == 380 && s[2].second == 380 && s[3].second == 1130,
          "no-sync frame wrong (%zu items)", s.size());
  }


  // ======================= analyzer (receiver side) =======================
  using Raw = remote_base::RawTimings;
  auto raw_of = [](const FrameTiming &t, uint64_t c, uint8_t b) {
    remote_base::RemoteTransmitData d;
    encode_frame(&d, t, c, b);
    return Raw(d.get_data());
  };
  auto near = [](uint32_t got, uint32_t want, double pct) { return std::abs((double) got - (double) want) <= want * pct / 100.0; };

  // A) default (merged) layout: every code decodes, sync is reported as merged, timing is recovered.
  for (uint64_t c : CODES) {
    auto a = analyze_frame(raw_of(legacy, c, 29), 29);
    CHECK(a.ok && a.code == c && a.bits == 29, "A decode 0x%llX (%s)", (unsigned long long) c, a.error ? a.error : "");
    CHECK(a.ok && a.sync_merged, "A should be merged");
    CHECK(a.ok && near(a.sync_mark_us, 6120, 2) && near(a.short_us, 380, 2) && near(a.long_us, 1130, 2),
          "A timing: sync %u short %u long %u", a.sync_mark_us, a.short_us, a.long_us);
    auto b = analyze_frame(raw_of(legacy, c, 29), 0);  // bits unknown: still merged (no big sync space)
    CHECK(b.ok && b.code == c && b.sync_merged, "A unknown bits");
  }

  // B) separate big sync space.
  for (uint64_t c : CODES) {
    for (uint8_t eb : {(uint8_t) 0, (uint8_t) 29}) {
      auto a = analyze_frame(raw_of(sep, c, 29), eb);
      CHECK(a.ok && a.code == c && !a.sync_merged && a.sync_mark_us == 6120 && a.sync_space_us == 5740, "B 0x%llX eb=%u (%s)",
            (unsigned long long) c, eb, a.error ? a.error : "");
    }
  }

  // C) separate but SHORT sync space: only distinguishable using the expected bit count.
  {
    FrameTiming shortsync{6120, 380, 380, 1130};
    for (uint64_t c : CODES) {
      auto a = analyze_frame(raw_of(shortsync, c, 29), 29);
      CHECK(a.ok && a.code == c && !a.sync_merged && a.sync_space_us == 380, "C 0x%llX (%s)", (unsigned long long) c, a.error ? a.error : "");
    }
    auto bad = analyze_frame(raw_of(shortsync, CODES[0], 29), 28);  // wrong bit count -> must refuse, not guess
    CHECK(!bad.ok, "C wrong expected bits must fail");
  }

  // D) gap handling: trailing gap element, and last space swallowed by the gap.
  for (const FrameTiming *t : {&legacy, &sep}) {
    for (uint64_t c : CODES) {
      Raw r = raw_of(*t, c, 29);
      Raw with_gap = r;
      with_gap.push_back(-10000);
      auto a = analyze_frame(with_gap, 29);
      CHECK(a.ok && a.code == c, "D trailing gap 0x%llX", (unsigned long long) c);
      Raw swallowed = r;
      swallowed.pop_back();  // last space is part of the idle gap, never reported
      auto b = analyze_frame(swallowed, 29);
      CHECK(b.ok && b.code == c, "D swallowed last space 0x%llX (%s)", (unsigned long long) c, b.error ? b.error : "");
    }
  }

  // E) realistic receiver: random jitter (+-10%) and OOK bias (marks longer, spaces shorter).
  {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> u(-0.10, 0.10);
    int bad = 0, total = 0;
    for (const FrameTiming *t : {&legacy, &sep}) {
      for (uint64_t c : CODES) {
        for (int trial = 0; trial < 100; trial++) {
          Raw r = raw_of(*t, c, 29);
          for (auto &v : r) {
            double len = std::abs(v) * (1.0 + u(rng)) + (v > 0 ? 70 : -70);
            v = static_cast<int32_t>(v > 0 ? len : -len);
          }
          auto a = analyze_frame(r, 29);
          total++;
          if (!(a.ok && a.code == c)) bad++;
        }
      }
    }
    CHECK(bad == 0, "E jitter/bias: %d of %d frames misdecoded", bad, total);
  }

  // F) must reject garbage instead of inventing a code.
  {
    CHECK(!analyze_frame(Raw{}, 29).ok, "F empty");
    CHECK(!analyze_frame(Raw{300, -400, 500}, 29).ok, "F too short");
    Raw starts_with_space = raw_of(sep, CODES[0], 29);
    starts_with_space[0] = -starts_with_space[0];
    CHECK(!analyze_frame(starts_with_space, 29).ok, "F starts with space");
    FrameTiming nosync{0, 0, 380, 1130};
    {
      // No sync field is now a supported ("plain") layout, but only when the bit count is known and matches.
      auto p = analyze_frame(raw_of(nosync, CODES[0], 29), 29);
      CHECK(p.ok && p.plain && p.code == CODES[0], "F plain layout with the right bit count");
      CHECK(!analyze_frame(raw_of(nosync, CODES[0], 29), 0).ok, "F plain layout needs the bit count");
      CHECK(!analyze_frame(raw_of(nosync, CODES[0], 29), 28).ok, "F plain layout with a wrong bit count");
    }
    Raw flipped = raw_of(sep, CODES[0], 29);  // make one mark disagree with its space
    flipped[2] = (std::abs(flipped[2]) > 700) ? 380 : 1130;
    CHECK(!analyze_frame(flipped, 29).ok, "F mark/space disagreement");
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> len(80, 700);
    int accepted = 0;
    for (int trial = 0; trial < 200; trial++) {  // pure noise, alternating levels
      Raw n;
      for (int i = 0; i < 40; i++) n.push_back(i % 2 == 0 ? len(rng) : -len(rng));
      if (analyze_frame(n, 29).ok) accepted++;
    }
    CHECK(accepted == 0, "F noise accepted %d times", accepted);
  }

  // G) a different remote (other timing) is decoded without configuration.
  {
    FrameTiming other{5000, 0, 300, 900};
    auto a = analyze_frame(raw_of(other, 0x155AAULL, 17), 17);
    CHECK(a.ok && a.code == 0x155AAULL && a.bits == 17 && near(a.short_us, 300, 2) && near(a.long_us, 900, 2),
          "G other remote (%s)", a.error ? a.error : "");
  }

  // H) sizes: 8 bits and 64 bits.
  {
    uint64_t big = 0xA5C3F00F5AA5C33CULL;
    for (const FrameTiming *t : {&legacy, &sep}) {
      auto a = analyze_frame(raw_of(*t, big, 64), 64);
      CHECK(a.ok && a.code == big && a.bits == 64, "H 64 bit (%s)", a.error ? a.error : "");
      auto b = analyze_frame(raw_of(*t, 0xB6, 8), 8);
      CHECK(b.ok && b.code == 0xB6 && b.bits == 8, "H 8 bit (%s)", b.error ? b.error : "");
    }
  }

  // I) the suggested YAML values really reproduce the received waveform (receiver -> transmitter loop).
  for (const FrameTiming *t : {&legacy, &sep}) {
    Raw r = raw_of(*t, CODES[3], 29);
    auto a = analyze_frame(r, 29);
    FrameTiming suggested{a.sync_mark_us, a.sync_space_us, a.short_us, a.long_us};
    Raw again = raw_of(suggested, a.code, a.bits);
    bool same = again.size() == r.size();
    for (size_t i = 0; same && i < r.size(); i++)
      same = (r[i] > 0) == (again[i] > 0) && near(std::abs(again[i]), std::abs(r[i]), 2);
    CHECK(same, "I suggested timing does not reproduce the waveform");
  }

  // ======= real remote (captured): 30 bits, then a 5000 us gap; repetitions glued together =======
  // Every frame is `30 bits + gap`; the gap is added to the last bit's space. The receiver sees
  // [mark of bit 30][gap][29 bits]..., so the analyzer must fold that first mark back as the last bit.
  {
    using Raw = remote_base::RawTimings;
    const FrameTiming real;  // defaults: no sync field
    CHECK(real.sync_mark_us == 0 && real.sync_space_us == 0, "defaults: no sync field");
    struct Case { uint64_t code29; bool flag; };
    for (Case c : {Case{0x1EA0DD72ULL, false}, Case{0x1EA0DDD0ULL, true}, Case{0x1EA0DC95ULL, true}}) {
      const uint64_t word = (c.code29 << 1) | (c.flag ? 1ULL : 0ULL);
      Raw train;
      for (int k = 0; k < 4; k++) {
        remote_base::RemoteTransmitData d;
        encode_frame(&d, real, word, 30);
        Raw f(d.get_data());
        CHECK(f.size() == 60 && f.back() < 0, "30 bits = 60 elements, ends in a space (%zu)", f.size());
        f.back() -= 5000;  // the idle gap merges with the last space
        train.insert(train.end(), f.begin(), f.end());
      }
      // The middle frames, cut the way the receiver delivers them: [mark][gap][29 bits]
      Raw piece(train.begin() + 58, train.begin() + 118);
      auto a = analyze_frame(piece, 30);
      CHECK(a.ok && a.code == word && a.bits == 30 && a.folded, "0x%llX flag=%d: %s", (unsigned long long) word, (int) c.flag, a.error ? a.error : "");
      CHECK(a.ok && std::abs((int) a.gap_us - 5000) < 100, "gap %u", a.gap_us);
      // With the configured 29 bits (old setup) it still decodes the 29-bit part, never garbage.
      auto b = analyze_frame(piece, 29);
      CHECK(!b.ok || (b.code == c.code29 && !b.folded), "29-bit view");
    }
  }

  // ======= generic asymmetric timing (synthetic): bit 1 = 1115/500 us, bit 0 = 300/1325 us (mark/space), 30 bits + 5000 us =======
  {
    using Raw = remote_base::RawTimings;
    FrameTiming sul;
    sul.one_mark_us = 1115;
    sul.one_space_us = 500;
    sul.zero_mark_us = 300;
    sul.zero_space_us = 1325;
    const uint64_t word = 0x27F14701ULL;  // arbitrary word
    remote_base::RemoteTransmitData d;
    encode_frame(&d, sul, word, 30);
    Raw f(d.get_data());
    CHECK(f.size() == 60, "30 bits = 60 elements (%zu)", f.size());
    CHECK(f[0] == 1115 && f[1] == -500, "bit 1 encoded as 1115/500 (%d/%d)", (int) f[0], (int) f[1]);
    CHECK(f[2] == 300 && f[3] == -1325, "bit 0 encoded as 300/1325 (%d/%d)", (int) f[2], (int) f[3]);
    Raw train;
    for (int k = 0; k < 4; k++) {
      Raw g = f;
      g.back() -= 5000;  // last bit is 1 -> 500 + 5000 = 5500, as seen on the air
      train.insert(train.end(), g.begin(), g.end());
    }
    CHECK(train[59] == -5500, "gap merged into the last space (%d)", (int) train[59]);
    Raw piece(train.begin() + 58, train.begin() + 118);
    auto a = analyze_frame(piece, 30);
    CHECK(a.ok && a.code == word && a.folded && a.bits == 30, "decode %s", a.error ? a.error : "");
    CHECK(a.ok && a.asymmetric, "asymmetric timing detected");
    CHECK(a.ok && std::abs((int) a.one_mark_us - 1115) < 40 && std::abs((int) a.one_space_us - 500) < 40 &&
              std::abs((int) a.zero_mark_us - 300) < 40 && std::abs((int) a.zero_space_us - 1325) < 40,
          "measured %u/%u %u/%u", a.one_mark_us, a.one_space_us, a.zero_mark_us, a.zero_space_us);
    CHECK(a.ok && std::abs((int) a.gap_us - 5000) < 100, "gap %u", a.gap_us);
    // ArteConfort stays symmetric.
    FrameTiming real;
    remote_base::RemoteTransmitData d2;
    encode_frame(&d2, real, 0x3D41BBA1ULL, 30);
    Raw f2(d2.get_data());
    f2.back() -= 5000;
    Raw p2(f2.end() - 2, f2.end());
    Raw both(f2);
    both.insert(both.end(), f2.begin(), f2.end());
    Raw piece2(both.begin() + 58, both.begin() + 118);
    auto b = analyze_frame(piece2, 30);
    CHECK(b.ok && !b.asymmetric && b.code == 0x3D41BBA1ULL, "ArteConfort not flagged as asymmetric (%s)", b.error ? b.error : "");
  }

  // ======= Sulion remote, clean capture: no sync field, 32 bits + a final short mark (stop pulse) =======
  {
    using Raw = remote_base::RawTimings;
    Raw real{1190,-440,1180,-420,383,-1248,387,-1227,1189,-432,388,-1230,1191,-429,361,-1251,390,-1232,387,-1231,1190,-430,1191,-428,1191,-431,386,-1229,389,-1229,388,-1228,391,-1233,390,-1228,388,-1227,390,-1229,389,-1233,1192,-431,389,-1229,389,-1229,1192,-435,1193,-430,1187,-428,1188,-429,389,-1233,1193,-430,1192,-429,387,-1228,389};
    CHECK(real.size() == 65, "captured record has 65 elements");
    auto a = analyze_frame(real, 32);
    CHECK(a.ok && a.plain && a.bits == 32, "plain layout decodes: %s", a.error ? a.error : "");
    CHECK(a.ok && a.code == 0xCA3804F6ULL, "code 0x%llX", (unsigned long long) a.code);
    CHECK(a.ok && std::abs((int) a.trailer_us - 388) < 40, "trailer mark %u", a.trailer_us);
    CHECK(a.ok && (a.code >> 16) == 0xCA38, "first 16 bits are 0xCA38");
    CHECK(a.ok && std::abs((int) a.one_mark_us - 1191) < 40 && std::abs((int) a.one_space_us - 430) < 40 &&
              std::abs((int) a.zero_mark_us - 388) < 40 && std::abs((int) a.zero_space_us - 1231) < 40,
          "timing %u/%u %u/%u", a.one_mark_us, a.one_space_us, a.zero_mark_us, a.zero_space_us);
    CHECK(a.ok && !a.asymmetric, "not flagged asymmetric");
    CHECK(!analyze_frame(real, 0).ok, "plain layout needs the bit count");
    CHECK(!analyze_frame(real, 30).ok, "wrong bit count rejected");

    // The emitter (Sulion preset values) must reproduce the capture element by element.
    FrameTiming sul;
    sul.short_us = 380; sul.long_us = 1240;
    sul.one_mark_us = 1190; sul.one_space_us = 430; sul.zero_mark_us = 380; sul.zero_space_us = 1240;
    sul.trailer_mark_us = 380;
    remote_base::RemoteTransmitData d;
    encode_frame(&d, sul, 0xCA3804F6ULL, 32);
    Raw f(d.get_data());
    CHECK(f.size() == 65, "32 bits + trailer = 65 elements (%zu)", f.size());
    bool close = f.size() == real.size();
    for (size_t i = 0; close && i < f.size(); i++)
      close = (f[i] > 0) == (real[i] > 0) && std::abs(std::abs(f[i]) - std::abs(real[i])) < 70;
    CHECK(close, "emitted waveform matches the captured one within 70 us");
    auto c = analyze_frame(f, 32);
    CHECK(c.ok && c.plain && c.code == 0xCA3804F6ULL && c.trailer_us == 380, "synthetic round trip: %s", c.error ? c.error : "");
  }

  std::printf(failures ? "\n%d FAILURE(S)\n" : "\nALL ENCODER + ANALYZER TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
