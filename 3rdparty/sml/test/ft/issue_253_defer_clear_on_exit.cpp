// cppcheck-suppress missingIncludeSystem
#include <boost/sml.hpp>
// cppcheck-suppress missingIncludeSystem
#include <deque>
// cppcheck-suppress missingIncludeSystem
#include <queue>

namespace sml = boost::sml;
using namespace sml;

struct e253_1 {};
struct e253_to_s2 {};
struct e253_to_s1 {};
struct e253_enter {};
struct e253_exit {};

// Test 1: stale deferred event must NOT fire after sub-SM re-entry + round-trip
test defer_cleared_on_sub_sm_exit = [] {
  static int fires = 0;

  struct Nested {
    auto operator()() const noexcept {
      return make_transition_table(
         *"s1"_s + event<e253_to_s2>         = "s2"_s,
          "s2"_s + event<e253_1> / defer,
          "s1"_s + event<e253_1> / [] { fires++; },
          "s2"_s + event<e253_to_s1>         = "s1"_s
      );
    }
  };
  struct Outer {
    auto operator()() const noexcept {
      return make_transition_table(
         *"Init"_s + event<e253_enter>          = state<Nested>,
          state<Nested> + event<e253_exit>      = "Init"_s
      );
    }
  };

  sm<Outer, defer_queue<std::deque>, process_queue<std::queue>> m;

  // Session 1: enter sub-SM, defer e253_1 in s2, then exit
  m.process_event(e253_enter{});   // Init → Nested/s1
  m.process_event(e253_to_s2{});   // s1 → s2
  m.process_event(e253_1{});       // deferred in s2
  m.process_event(e253_exit{});    // Nested → Init; sub-SM defer queue cleared

  // Session 2: re-enter, do round-trip s1→s2→s1 — stale e253_1 must NOT fire
  m.process_event(e253_enter{});   // Init → Nested/s1 (fresh start)
  fires = 0;
  m.process_event(e253_to_s2{});
  m.process_event(e253_to_s1{});   // BUG was: fires == 1 here
  expect(fires == 0);
};

// Test 2: deferring still works correctly within a single sub-SM lifetime
test defer_works_within_sub_sm = [] {
  static int fires = 0;

  struct Nested {
    auto operator()() const noexcept {
      return make_transition_table(
         *"s1"_s + event<e253_to_s2>         = "s2"_s,
          "s2"_s + event<e253_1> / defer,
          "s1"_s + event<e253_1> / [] { fires++; },
          "s2"_s + event<e253_to_s1>         = "s1"_s
      );
    }
  };
  struct Outer {
    auto operator()() const noexcept {
      return make_transition_table(
         *"Init"_s + event<e253_enter>          = state<Nested>,
          state<Nested> + event<e253_exit>      = "Init"_s
      );
    }
  };

  sm<Outer, defer_queue<std::deque>, process_queue<std::queue>> m;

  m.process_event(e253_enter{});
  m.process_event(e253_to_s2{});
  fires = 0;
  m.process_event(e253_1{});      // deferred in s2
  m.process_event(e253_to_s1{});  // s2→s1: deferred e253_1 MUST fire
  expect(fires == 1);
};

// Test 3: re-entry without any preceding deferral produces no spurious fires
test no_spurious_fire_without_defer = [] {
  static int fires = 0;

  struct Nested {
    auto operator()() const noexcept {
      return make_transition_table(
         *"s1"_s + event<e253_to_s2>         = "s2"_s,
          "s2"_s + event<e253_1> / defer,
          "s1"_s + event<e253_1> / [] { fires++; },
          "s2"_s + event<e253_to_s1>         = "s1"_s
      );
    }
  };
  struct Outer {
    auto operator()() const noexcept {
      return make_transition_table(
         *"Init"_s + event<e253_enter>          = state<Nested>,
          state<Nested> + event<e253_exit>      = "Init"_s
      );
    }
  };

  sm<Outer, defer_queue<std::deque>, process_queue<std::queue>> m;

  m.process_event(e253_enter{});
  m.process_event(e253_exit{});   // exit without any deferral

  m.process_event(e253_enter{});  // re-enter
  fires = 0;
  m.process_event(e253_to_s2{});
  m.process_event(e253_to_s1{});
  expect(fires == 0);
};
